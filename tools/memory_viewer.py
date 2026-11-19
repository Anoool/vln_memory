"""在本地浏览已发布的空间记忆、回看图像并保存人工标注。"""

import argparse
from collections import Counter
from itertools import islice
import json
from pathlib import Path
import sqlite3
from subprocess import TimeoutExpired

import numpy as np
from PIL import Image, ImageDraw
import streamlit as st
import yaml

from memory_rag import checked_observations, location_fields, read_rows
from viewer_annotations import AnnotationStore, ROOT
from viewer_map import entity_figure, related_entity_ids
from viewer_style import (
  PAGE_STYLE, attributes_html, description_html, entity_header_html, map_legend_html, markdown_text, section_html,
  stats_html,
)


DEFAULT_CONFIG = ROOT / "config/memory_viewer.yaml"
GALLERY_PAGE_SIZE = 6
GALLERY_THUMBNAIL_PX = (640, 360)
MAP_HEIGHT_PX = 560
SECTION_ENTITIES = "实体与标注"
SECTION_GALLERY = "全部图片"
RELATION_LABELS = {
  "inside": "位于内部", "near": "靠近", "on": "放在上面",
  "hanging_on": "挂在上面", "connects_regions": "连接区域",
}


def release_from_config(config_path):
  """从 YAML 读取发布目录；相对路径以配置文件所在目录为准。"""
  settings = yaml.safe_load(config_path.read_text(encoding="utf-8"))
  return (config_path.resolve().parent / settings["release"]).resolve()


@st.cache_data(show_spinner="正在打开记忆包…")
def load_memory(release_dir):
  """校验发布版本并索引实体、观测和有依据的关系；暂不解码图像。"""
  release = Path(release_dir)
  manifest = json.loads((release / "manifest.json").read_text(encoding="utf-8"))
  if (manifest["artifact_kind"], manifest["schema_version"]) != ("spatial_memory", "spatial_memory/1"):
    raise ValueError("需要已发布的 spatial_memory/1 目录。")
  entities = {}
  for row in read_rows(release, manifest, "entities"):
    entity_id = row["entity_id"]
    if not entity_id or entity_id in entities or row["geometry_revision"] != manifest["geometry_revision"]:
      raise ValueError(f"实体 ID 或坐标版本无效：{entity_id}")
    entities[entity_id] = row
  annotations = {row["entity_id"]: row for row in read_rows(release, manifest, "annotations")}
  observations = checked_observations(release, manifest)
  relations = {}
  for row in read_rows(release, manifest, "relations"):
    if row["recognition_state"] != "supported":
      continue
    if row["geometry_revision"] != manifest["geometry_revision"]:
      raise ValueError(f"关系坐标与发布版本不一致：{row['id']}")
    if any(obs_id not in observations for obs_id in row["evidence_observation_ids"]):
      raise ValueError(f"找不到关系引用的观测：{row['id']}")
    for entity_id in row["entity_ids"]:
      if entity_id not in entities:
        raise ValueError(f"找不到关系引用的实体：{entity_id}")
      relations.setdefault(entity_id, []).append(row)
  return manifest, entities, annotations, observations, relations


def media_path(release, relative):
  """将证据路径限制在当前发布包的 evidence/ 内。"""
  relative = Path(relative)
  path = (release / relative).resolve()
  if relative.is_absolute() or not path.is_relative_to(release / "evidence") or not path.is_file():
    raise ValueError(f"无法打开证据文件：{relative}")
  return path


def depth_preview(image):
  """把 16 位深度转成显示用灰度，原始零值显示为黑色。"""
  depth = np.asarray(image)
  if depth.ndim != 2:
    raise ValueError("需要单通道的原始深度图。")
  valid = depth > 0
  display = np.zeros(depth.shape, dtype=np.uint8)
  if valid.any():
    values = depth[valid]
    low, high = np.percentile(values, (2, 98))
    display[valid] = 1 + (254 * np.clip((values - low) / max(1, high - low), 0, 1)).astype(np.uint8)
  return Image.fromarray(display).convert("RGB")


def evidence_image(path, kind, selector):
  """读取一张图，按选区记录的图像类型叠加原记录的点、矩形或多边形。"""
  with Image.open(path) as source:
    if kind == "depth":
      image = depth_preview(source)
    else:
      image = source.convert("RGB")
  if selector and selector["image"] == kind:
    draw = ImageDraw.Draw(image)
    if selector["type"] == "bbox":
      x1, y1, x2, y2 = selector["xyxy_px"]
      draw.rectangle((x1, y1, x2 - 1, y2 - 1), outline="#ffac32", width=5)
    elif selector["type"] == "point":
      x, y = selector["pixel_xy"]
      draw.ellipse((x - 6, y - 6, x + 6, y + 6), outline="#ffac32", width=3)
    elif selector["type"] == "polygon":
      vertices = [tuple(point) for point in selector["vertices_xy_px"]]
      draw.polygon(vertices, outline="#ffac32", width=5)
  return image


def select_map_entity():
  """用浏览器发回的 Plotly 点选结果同步实体下拉框；忽略不带实体 ID 的连线点。"""
  points = st.session_state["entity_map"].selection.points
  for point in reversed(points):
    entity_id = point.get("customdata")
    if entity_id:
      st.session_state["selected_entity"] = entity_id
      return


def jump_to_entity(entity_id):
  """关系列表中的按钮回调：把选中实体切换为关系另一端的实体。"""
  st.session_state["selected_entity"] = entity_id


def relation_sentence(relation, names, selected):
  """把关系按原始方向写成 Markdown 句子；选中实体加粗，类型原文保留在代码样式中。"""
  parts = []
  for entity_id in relation["entity_ids"]:
    name = markdown_text(names[entity_id])
    if entity_id == selected:
      name = f"**{name}**"
    parts.append(name)
  label = RELATION_LABELS.get(relation["type"], relation["type"])
  return f"{' → '.join(parts)} · {markdown_text(label)} `{relation['type']}`"


def observation_caption(obs_id, observation):
  """把观测 ID、修订放在第一行，同步和配准状态放在第二行；状态按原文显示，不推断为已验证。"""
  return (
    f"观测 `{obs_id}` · 修订 {observation['revision']}  \n"
    f"采集同步 {markdown_text(observation['capture_sync_status'])} · "
    f"深度配准 {markdown_text(observation['depth_alignment'])}"
  )


def show_observation(release, obs_id, observation, selector=None, thumbnail=False, stacked=False):
  """呈现同一观测的 RGB 和深度：默认并排，stacked 为上下排列；缩略图仅在内存里生成。"""
  st.caption(observation_caption(obs_id, observation))
  if stacked:
    containers = (st.container(), st.container())
  else:
    containers = st.columns(2, gap="small")
  for container, kind, caption in zip(containers, ("rgb", "depth"), ("RGB", "深度预览 · 黑色为零值")):
    path = media_path(release, observation[f"{kind}_media"])
    if selector and selector["image"] == kind:
      caption += " · 黄框为定位选区"
    image = evidence_image(path, kind, selector)
    if thumbnail:
      image.thumbnail(GALLERY_THUMBNAIL_PX)
    container.image(image, caption=caption)


@st.dialog("原尺寸图像", width="large")
def show_observation_dialog(release, obs_id, observation, selector=None):
  """在弹窗里并排显示一组观测的原尺寸 RGB 和深度预览；传入 selector 时保留选区叠加。"""
  try:
    show_observation(release, obs_id, observation, selector=selector)
  except (OSError, ValueError, TypeError, KeyError) as error:
    st.warning(f"无法读取这组图像：{error}")


def evidence_options(release, entity, observations, relations):
  """列出实体可回看的证据：定位观测、各关系的支持帧、同修订的其他实体观测；每项为（标签, 观测 ID, 选区, 关系）。"""
  location = location_fields(release, entity, observations)
  options = []
  if location["observation_id"]:
    options.append(("定位证据", location["observation_id"], location["selector"], None))
  for index, row in enumerate(relations, 1):
    label = RELATION_LABELS.get(row["type"], row["type"])
    for frame, obs_id in enumerate(row["evidence_observation_ids"], 1):
      options.append((f"关系 {index} · {label} · 第 {frame} 帧", obs_id, None, row))
  for frame, ref in enumerate(entity["observation_refs"], 1):
    obs_id = ref["observation_id"]
    observation = observations.get(obs_id)
    if obs_id == location["observation_id"] or not observation:
      continue
    if int(observation["revision"]) == ref["observation_revision"]:
      options.append((f"实体观测 · 第 {frame} 帧", obs_id, ref.get("selector"), None))
  return options


def show_evidence(release, entity, observations, relations, names):
  """按不同证据来源选择原始观测，保留定位和关系各自的引用，并提供弹窗原图查看。"""
  options = evidence_options(release, entity, observations, relations)
  if not options:
    st.info("这个实体没有可回看的观测。")
    return
  labels = {index: option[0] for index, option in enumerate(options)}
  with st.container(horizontal=True, wrap=False, vertical_alignment="bottom"):
    choice = st.selectbox(
      f"证据来源（共 {len(options)} 项）", list(labels), format_func=labels.__getitem__,
      key=f"evidence_{entity['entity_id']}",
    )
    zoom_clicked = st.button(
      "放大", key=f"evidence_zoom_{entity['entity_id']}", icon=":material/zoom_in:", width="content",
    )
  _, obs_id, selector, relation = options[choice]
  if zoom_clicked:
    show_observation_dialog(release, obs_id, observations[obs_id], selector=selector)
  if relation:
    st.markdown(relation_sentence(relation, names, entity["entity_id"]))
    st.caption(f"关系依据：{relation['basis']}")
  show_observation(release, obs_id, observations[obs_id], selector, stacked=True)


def gallery_controls(observations):
  """在侧栏渲染图库页码和原图选择；返回（页码, 本页观测列表, 选中的观测 ID 或 None）。"""
  count = len(observations)
  pages = (count + GALLERY_PAGE_SIZE - 1) // GALLERY_PAGE_SIZE
  st.markdown(section_html("图库"), unsafe_allow_html=True)
  page = st.number_input(
    f"图库页码（共 {pages} 页）", min_value=1, max_value=pages, value=1, step=1, key="gallery_page",
  )
  start = (page - 1) * GALLERY_PAGE_SIZE
  items = list(islice(observations.items(), start, start + GALLERY_PAGE_SIZE))
  selected = st.selectbox(
    "在顶部固定一组原尺寸图像", [obs_id for obs_id, _ in items], index=None,
    placeholder="选择本页的一组观测…", key=f"gallery_original_{page}",
  )
  st.caption(f"共 {count} 组 RGB-D / {2 * count} 张图片 · 第 {page}/{pages} 页")
  return page, items, selected


def show_gallery_card(release, number, obs_id, observation):
  """渲染一张图库卡片：序号、放大按钮和 RGB-D 缩略图；读取失败时在卡片内提示。"""
  with st.container(border=True):
    with st.container(horizontal=True, wrap=False, vertical_alignment="center"):
      st.markdown(f"**#{number}**", width="content")
      st.space("stretch")
      if st.button("放大", key=f"zoom_{obs_id}", icon=":material/zoom_in:", width="content"):
        show_observation_dialog(release, obs_id, observation)
    try:
      show_observation(release, obs_id, observation, thumbnail=True)
    except (OSError, ValueError, TypeError, KeyError) as error:
      st.warning(f"无法读取这组图像：{error}")


def show_gallery(release, observations):
  """按发布顺序浏览全部观测，每页只解码显示中的图像。"""
  if not observations:
    st.info("这个记忆包没有图片。")
    return
  with st.sidebar:
    page, items, selected = gallery_controls(observations)
  if selected:
    with st.container(border=True):
      st.markdown("**已固定的原尺寸图像**")
      try:
        show_observation(release, selected, observations[selected])
      except (OSError, ValueError, TypeError, KeyError) as error:
        st.warning(f"无法读取这组图像：{error}")
  start = (page - 1) * GALLERY_PAGE_SIZE
  columns = st.columns(2)
  for index, (obs_id, observation) in enumerate(items):
    with columns[index % len(columns)]:
      show_gallery_card(release, start + index + 1, obs_id, observation)


def load_annotation_editor(config_path, manifest):
  """只在本机回环服务且工作库来源正确时启用编辑；返回（编辑入口, 工作库标注, 只读原因）。"""
  settings = yaml.safe_load(config_path.read_text(encoding="utf-8")).get("annotations")
  if settings is None:
    return None, {}, "当前配置未启用网页标注，页面为只读。"
  if st.get_option("server.address") not in ("127.0.0.1", "::1", "localhost"):
    return None, {}, "仅在绑定到本机地址的 Viewer 中开放编辑。"
  try:
    editor = AnnotationStore.from_config(config_path.resolve().parent, settings)
    return editor, editor.annotations(manifest), None
  except (OSError, ValueError, TypeError, KeyError, sqlite3.Error, yaml.YAMLError) as error:
    return None, {}, f"只能查看：{error}"


def show_annotation_form(editor, entity, annotation):
  """用已显示的标注修订提交三个有限字段，保存后重读并刷新名称。"""
  entity_id = entity["entity_id"]
  revision = annotation["annotation_revision"]
  st.caption(f"仅编辑以下三个字段 · 模型原标签：{markdown_text(entity['semantic_label'])}")
  if st.session_state.pop("saved_annotation", None) == entity_id:
    st.success("标注已保存。")
  with st.form(f"annotation_{entity_id}_{revision}", border=False):
    name = st.text_input("显示名称", value=annotation["display_name"] or "")
    alias_column, note_column = st.columns(2)
    aliases = alias_column.text_area("别名（每行一个）", value="\n".join(annotation["aliases"]), height=120)
    note = note_column.text_area("备注", value=annotation["note"] or "", height=120)
    submitted = st.form_submit_button("保存标注", type="primary")
  if submitted:
    try:
      editor.save(entity_id, revision, name, aliases, note)
    except (OSError, ValueError, TimeoutExpired) as error:
      st.error(f"未保存：{error}")
      return
    st.session_state["saved_annotation"] = entity_id
    st.rerun()


def show_annotation_tab(editor, working, readonly_reason, entity):
  """标注页签：可编辑时显示表单，否则说明只读原因或实体已在工作库中变化。"""
  if editor is None:
    st.info(readonly_reason)
    return
  active = working.get(entity["entity_id"])
  if active and active[0] == entity["revision"]:
    show_annotation_form(editor, entity, active[1])
  else:
    st.info("这个实体在工作库中已更新或撤回，发布新版后可继续标注。")


def show_relation_list(selected, relations, names):
  """逐条列出选中实体的已发布关系；右侧按钮跳转到关系另一端的实体。"""
  st.markdown(f"**已发布关系（{len(relations)}）**")
  if not relations:
    st.caption("这个实体没有已发布关系。")
    return
  for index, relation in enumerate(relations):
    other = selected
    for entity_id in relation["entity_ids"]:
      if entity_id != selected:
        other = entity_id
        break
    with st.container(border=True):
      with st.container(horizontal=True, wrap=False, vertical_alignment="center"):
        st.markdown(relation_sentence(relation, names, selected), width="stretch")
        st.button(
          "查看", key=f"jump_{selected}_{index}", on_click=jump_to_entity, args=(other,),
          disabled=other == selected, width="content", icon=":material/arrow_forward:",
        )


def show_entity_details(entity, selected, relations, names):
  """详情页签：实体描述、属性和关系列表。"""
  description = entity.get("description")
  attributes = entity.get("attributes") or {}
  if description:
    st.markdown(description_html(description), unsafe_allow_html=True)
  if attributes:
    st.markdown(attributes_html(attributes), unsafe_allow_html=True)
  if not description and not attributes:
    st.caption("这个实体没有描述或属性。")
  show_relation_list(selected, relations, names)


def display_names(entities, annotations):
  """返回实体显示名称（人工名称优先）和下拉框名称（重名时附加 ID 尾部）。"""
  names = {
    item: annotations.get(item, {}).get("display_name") or row["semantic_label"]
    for item, row in entities.items()
  }
  name_counts = Counter(names.values())
  option_names = names.copy()
  for entity_id, name in names.items():
    if name_counts[name] > 1:
      option_names[entity_id] = f"{name} · {entity_id[-8:]}"
  return names, option_names


def entity_controls(entities, option_names):
  """在侧栏渲染实体选择框；返回选中的实体 ID。"""
  st.markdown(section_html("实体"), unsafe_allow_html=True)
  selected = st.selectbox(
    "选择实体", list(entities), format_func=option_names.__getitem__, key="selected_entity",
    help="可输入名称筛选，也可直接在地图上点选。",
  )
  return selected


def show_entity_inspector(release, entity, observations, entity_relations, names, annotation_state):
  """右侧检查器：实体名称与摘要，下设证据、详情和标注三个页签。"""
  editor, working, readonly_reason = annotation_state
  selected = entity["entity_id"]
  st.markdown(entity_header_html(entity, names[selected], len(entity_relations)), unsafe_allow_html=True)
  evidence_tab, details_tab, annotation_tab = st.tabs(
    ["观测证据", f"详情与关系（{len(entity_relations)}）", "人工标注"],
  )
  with evidence_tab:
    try:
      show_evidence(release, entity, observations, entity_relations, names)
    except (OSError, ValueError, TypeError, KeyError) as error:
      st.error(f"无法读取观测证据：{error}")
  with details_tab:
    show_entity_details(entity, selected, entity_relations, names)
  with annotation_tab:
    show_annotation_tab(editor, working, readonly_reason, entity)


def show_entities(release, manifest, entities, annotations, observations, relations, config_path):
  """侧栏选择实体；主区左侧为俯视实体图，右侧为证据、详情与标注检查器。"""
  editor, working, readonly_reason = load_annotation_editor(config_path, manifest)
  merged_annotations = dict(annotations)
  for item, (revision, annotation) in working.items():
    if item in entities and revision == entities[item]["revision"]:
      merged_annotations[item] = annotation
  names, option_names = display_names(entities, merged_annotations)
  with st.sidebar:
    selected = entity_controls(entities, option_names)
  entity_relations = relations.get(selected, [])
  related = related_entity_ids(selected, entity_relations)
  map_column, inspector_column = st.columns([1.12, 1], gap="large")
  with map_column:
    st.markdown(map_legend_html(), unsafe_allow_html=True)
    st.plotly_chart(
      entity_figure(entities, names, selected, related, manifest["memory_version"], MAP_HEIGHT_PX),
      key="entity_map", on_select=select_map_entity, selection_mode="points",
      config={"displaylogo": False, "scrollZoom": False},
    )
  with inspector_column:
    show_entity_inspector(
      release, entities[selected], observations, entity_relations, names, (editor, working, readonly_reason),
    )


def show_sidebar_header(manifest, entities, observations):
  """侧栏顶部：标题、发布版本、三项计数和浏览方式切换；返回当前浏览方式。"""
  st.title("空间记忆浏览器")
  st.caption(
    f"发布版 `{manifest['memory_version']}`  \n地图 `{manifest['map_id']}` / `{manifest['frame_id']}`"
  )
  counts = (("实体", len(entities)), ("观测", len(observations)), ("关系", manifest["counts"]["relations"]))
  st.markdown(stats_html(counts), unsafe_allow_html=True)
  st.markdown(section_html("浏览方式"), unsafe_allow_html=True)
  section = st.segmented_control(
    "浏览方式", (SECTION_ENTITIES, SECTION_GALLERY), default=SECTION_ENTITIES, key="section",
    required=True, label_visibility="collapsed", width="stretch",
  )
  return section


def main():
  """打开当前发布版本：侧栏负责导航与选择，主区按浏览方式显示实体或图库。"""
  parser = argparse.ArgumentParser(description=__doc__)
  parser.add_argument("--config", type=Path, default=DEFAULT_CONFIG)
  args = parser.parse_args()
  st.set_page_config(
    page_title="空间记忆浏览器", page_icon=":material/map:", layout="wide", initial_sidebar_state="expanded",
  )
  st.markdown(PAGE_STYLE, unsafe_allow_html=True)
  try:
    release = release_from_config(args.config)
    manifest, entities, annotations, observations, relations = load_memory(str(release))
  except (OSError, ValueError, TypeError, KeyError, yaml.YAMLError) as error:
    st.title("空间记忆浏览器")
    st.error(f"无法读取记忆包：{error}")
    return
  with st.sidebar:
    section = show_sidebar_header(manifest, entities, observations)
  if section == SECTION_GALLERY:
    show_gallery(release, observations)
    return
  if not entities:
    st.info("这个记忆包没有实体。")
    return
  show_entities(release, manifest, entities, annotations, observations, relations, args.config)


if __name__ == "__main__":
  main()
