"""MemoryViewer 的页面样式和 HTML/Markdown 片段；只负责显示，不读取发布数据。"""

from html import escape
import json


ENTITY_COLOR = "#4c86a8"
RELATED_COLOR = "#e8a33d"
SELECTED_COLOR = "#ce571e"
MARKDOWN_SPECIAL = "\\`*_{}[]()#+-.!|<>~$:"
PAGE_STYLE = """
<style>
  .block-container { padding: 1rem 2rem 1.2rem 2rem; max-width: 1760px; }
  [data-testid="stHeader"] { background: transparent; height: 2rem; }
  [data-testid="stSidebar"] { min-width: 330px; }
  [data-testid="stSidebarContent"] { background: #ffffff; }
  [data-testid="stSidebarHeader"] { height: 2rem; }
  [data-testid="stSidebarUserContent"] { padding-top: 0; }
  [data-testid="stSidebar"] h1 { font-size: 1.45rem !important; padding: 0 0 .1rem 0 !important; }
  .mv-stats { display: grid; grid-template-columns: repeat(3, 1fr); gap: .5rem; margin: .3rem 0 .4rem 0; }
  .mv-stat { border: 1px solid #d8dee4; border-radius: .5rem; padding: .45rem .6rem; background: #f8fafc; }
  .mv-stat b { display: block; font-size: 1.25rem; font-weight: 600; color: #1f2328; line-height: 1.3; }
  .mv-stat span { font-size: .75rem; color: #57606a; }
  [data-testid="stCaptionContainer"] { color: #4b5563; }
  [data-testid="stImage"] img { border-radius: 6px; }
  [data-testid="stImageCaption"] { color: #4b5563; margin-top: .2rem; }
  [data-testid="stTabs"] [data-baseweb="tab-list"] { gap: 1.2rem; }
  [data-testid="stTabs"] [data-baseweb="tab-panel"] { padding-top: .6rem; }
  [data-testid="stSelectboxVirtualDropdown"] {
    min-width: 440px !important; max-width: min(560px, 90vw) !important;
  }
  .mv-section { font-size: .78rem; font-weight: 600; letter-spacing: .04em; color: #6b7280; margin: .4rem 0 .2rem 0; }
  .mv-entity-name { font-size: 1.3rem; font-weight: 650; color: #1f2328; margin: 0 0 .3rem 0; line-height: 1.3; }
  .mv-chips { display: flex; flex-wrap: wrap; gap: .35rem; margin-bottom: .1rem; }
  .mv-chip {
    display: inline-block; padding: .12rem .55rem; border-radius: 999px; font-size: .78rem;
    background: #eef2f6; color: #334155; border: 1px solid #dde3ea;
  }
  .mv-chip-accent { background: #fdf0e8; color: #9a3f12; border-color: #f4d3bf; }
  .mv-legend { display: flex; flex-wrap: wrap; gap: .45rem 1rem; font-size: .8rem; color: #4b5563; margin: 0 0 .25rem 0; }
  .mv-legend span { display: inline-flex; align-items: center; gap: .35rem; }
  .mv-dot { width: .7rem; height: .7rem; border-radius: 50%; display: inline-block; border: 1.5px solid #ffffff; box-shadow: 0 0 0 1px #d0d7de; }
  .mv-line { width: 1.2rem; border-top: 2px dotted; display: inline-block; }
  .mv-description { font-size: .9rem; line-height: 1.6; color: #374151; margin: 0 0 .8rem 0; }
  .mv-attributes { display: grid; grid-template-columns: max-content 1fr; gap: .3rem 1rem; font-size: .86rem; margin: 0 0 .9rem 0; }
  .mv-attributes dt { color: #6b7280; }
  .mv-attributes dd { margin: 0; color: #1f2328; }
</style>
"""


def markdown_text(value):
  """转义 Markdown 控制字符，使实体名和标签按原文显示；返回可嵌入 Markdown 的字符串。"""
  escaped = []
  for character in str(value):
    if character in MARKDOWN_SPECIAL:
      escaped.append("\\")
    escaped.append(character)
  return "".join(escaped)


def section_html(title):
  """返回侧栏小节标题的 HTML；标题文本已转义。"""
  return f'<div class="mv-section">{escape(title)}</div>'


def stats_html(counts):
  """把（名称, 数量）序列渲染为侧栏的三栏计数卡片 HTML；名称已转义，数量按整数显示。"""
  cards = []
  for label, value in counts:
    cards.append(f'<div class="mv-stat"><b>{int(value)}</b><span>{escape(label)}</span></div>')
  return f'<div class="mv-stats">{"".join(cards)}</div>'


def entity_header_html(entity, name, relation_count):
  """生成实体名称和摘要标签的 HTML：有人工名称时附带原标签，另含坐标（米）、观测数和关系数。"""
  chips = []
  semantic_label = entity["semantic_label"]
  if name != semantic_label:
    chips.append(f'<span class="mv-chip mv-chip-accent">原标签：{escape(semantic_label)}</span>')
  position = entity["position_map"]
  if position is not None:
    chips.append(
      f'<span class="mv-chip">X {position[0]:.2f} · Y {position[1]:.2f} · Z {position[2]:.2f} 米</span>'
    )
  chips.append(f'<span class="mv-chip">{len(entity["observation_refs"])} 次观测</span>')
  chips.append(f'<span class="mv-chip">{relation_count} 条关系</span>')
  return f'<div class="mv-entity-name">{escape(name)}</div><div class="mv-chips">{"".join(chips)}</div>'


def map_legend_html():
  """返回地图图例的 HTML：普通实体、选中实体、关联实体和关系连线的颜色。"""
  return (
    '<div class="mv-legend">'
    f'<span><i class="mv-dot" style="background:{ENTITY_COLOR}"></i>实体</span>'
    f'<span><i class="mv-dot" style="background:{SELECTED_COLOR}"></i>选中</span>'
    f'<span><i class="mv-dot" style="background:{RELATED_COLOR}"></i>关联实体</span>'
    f'<span><i class="mv-line" style="border-color:{SELECTED_COLOR}"></i>已发布关系</span>'
    '<span>单位：米 · 未加载地图底图 · 拖动平移，双击复位</span>'
    '</div>'
  )


def description_html(description):
  """返回实体描述段落的 HTML；描述文本已转义。"""
  return f'<div class="mv-description">{escape(description)}</div>'


def attributes_html(attributes):
  """把实体属性字典渲染为两列定义列表 HTML；键中的下划线换成空格，值转义后显示。"""
  rows = []
  for key, value in attributes.items():
    if isinstance(value, str):
      text = value
    else:
      text = json.dumps(value, ensure_ascii=False)
    rows.append(f"<dt>{escape(key.replace('_', ' '))}</dt><dd>{escape(text)}</dd>")
  return f'<dl class="mv-attributes">{"".join(rows)}</dl>'
