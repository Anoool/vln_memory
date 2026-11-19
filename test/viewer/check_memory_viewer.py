"""用 217 条真实观测的本地发布包验收 Viewer 的两种证据来源。"""

from pathlib import Path
import sys

from streamlit.testing.v1 import AppTest


ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "tools"))
from memory_rag import DEFAULT_CONFIG, location_fields
from memory_viewer import evidence_image, load_memory, media_path, release_from_config
from viewer_map import entity_figure, related_entity_ids
from viewer_style import entity_header_html


def check_map_and_header(manifest, entities, relations):
  """校验全部实体的地图避让标签和摘要标签规则，不遗漏边界点和长名称。"""
  names = {entity_id: row["semantic_label"] for entity_id, row in entities.items()}
  for entity_id in entities:
    related = related_entity_ids(entity_id, relations.get(entity_id, []))
    figure = entity_figure(entities, names, entity_id, related, manifest["memory_version"], 560)
    assert len(figure.layout.annotations) == 1
  chair_id = next(item for item, row in entities.items() if row["semantic_label"] == "light-gray shell task chair")
  chair_related = related_entity_ids(chair_id, relations.get(chair_id, []))
  chair_fig = entity_figure(entities, names, chair_id, chair_related, manifest["memory_version"], 560)
  chair_annotation = chair_fig.layout.annotations[0]
  assert not (chair_annotation.ax == 0 and chair_annotation.ay < 0)
  chair_header = entity_header_html(entities[chair_id], names[chair_id], len(chair_related))
  assert "原标签：" not in chair_header
  renamed_header = entity_header_html(entities[chair_id], "灰色办公椅", len(chair_related))
  assert "原标签：light-gray shell task chair" in renamed_header


def check():
  """点选背包，验证定位框及关系使用不同的同包观测。"""
  release = release_from_config(DEFAULT_CONFIG)
  manifest, entities, _, observations, relations = load_memory(str(release))
  check_map_and_header(manifest, entities, relations)
  selected = next(item for item, row in entities.items()
                  if row["semantic_label"] == "black backpack hung on chair backrest")
  location = location_fields(release, entities[selected], observations)
  relation = next(row for row in relations[selected] if row["type"] == "hanging_on")
  relation_id = relation["evidence_observation_ids"][0]
  assert relation_id != location["observation_id"]
  selector = location["selector"]
  painted = evidence_image(media_path(release, location["rgb_media"]), "rgb", selector)
  assert painted.getpixel(tuple(selector["xyxy_px"][:2])) == (255, 172, 50)

  app = AppTest.from_file(str(ROOT / "tools/memory_viewer.py")).run()
  assert not app.exception and not app.error
  stats = next(item.value for item in app.markdown if item.value.startswith('<div class="mv-stats">'))
  for count in (len(entities), len(observations), manifest["counts"]["relations"]):
    assert f"<b>{count}</b>" in stats
  app.selectbox(key="selected_entity").select(selected).run()
  assert not app.exception and not app.error
  assert any(location["observation_id"] in item.value for item in app.caption)
  evidence = app.selectbox(key=f"evidence_{selected}")
  relation_choice = next(index for index, label in enumerate(evidence.options) if "挂在上面" in label)
  evidence.select_index(relation_choice).run()
  assert not app.exception and not app.error
  assert any(relation_id in item.value for item in app.caption)
  assert any("hanging_on" in item.value for item in app.markdown)
  related = next(item for item in relation["entity_ids"] if item != selected)
  app.button(key=f"jump_{selected}_{relations[selected].index(relation)}").click().run()
  assert not app.exception and not app.error
  assert app.selectbox(key="selected_entity").value == related
  multiple = next(item for item, row in entities.items() if len(row["observation_refs"]) > 1)
  app.selectbox(key="selected_entity").select(multiple).run()
  evidence = app.selectbox(key=f"evidence_{multiple}")
  assert any("实体观测" in label for label in evidence.options)

  app.segmented_control(key="section").set_value("全部图片").run()
  assert not app.exception and not app.error
  assert any(f"{len(observations) * 2} 张图片" in item.value for item in app.caption)
  last_page = (len(observations) + 5) // 6
  app.number_input(key="gallery_page").set_value(last_page).run()
  assert not app.exception and not app.error
  last = next(reversed(observations))
  assert any(last in item.value for item in app.caption)
  app.selectbox(key=f"gallery_original_{last_page}").select(last).run()
  assert not app.exception and not app.error
  assert sum(last in item.value for item in app.caption) == 2


if __name__ == "__main__":
  check()
