"""MemoryViewer 的俯视实体图：用地图坐标（米）绘制实体，并高亮选中实体及其关系。"""

from html import escape

import plotly.graph_objects as go

from viewer_style import ENTITY_COLOR, RELATED_COLOR, SELECTED_COLOR


MAP_PADDING_M = 0.8
CALLOUT_MAX_CHARS = 28
CALLOUT_HEIGHT_PX = 26.0
CALLOUT_CLEARANCE_PX = 13.0
LEADER_CLEARANCE_PX = 14.0
ESTIMATED_PLOT_WIDTH_PX = 480.0
PLOT_VERTICAL_CHROME_PX = 72.0
HOVER_TEMPLATE = "<b>%{text}</b><br>X %{x:.2f} m · Y %{y:.2f} m<extra></extra>"
CALLOUT_CANDIDATES = (
  (28.0, 0.0, "left", "middle"),
  (-28.0, 0.0, "right", "middle"),
  (0.0, 32.0, "center", "top"),
  (0.0, -32.0, "center", "bottom"),
  (22.0, 26.0, "left", "top"),
  (-22.0, 26.0, "right", "top"),
  (22.0, -26.0, "left", "bottom"),
  (-22.0, -26.0, "right", "bottom"),
)


def related_entity_ids(selected, relations):
  """返回与选中实体有已发布关系的其他实体 ID，保持关系出现顺序且不重复。"""
  related = {}
  for row in relations:
    for entity_id in row["entity_ids"]:
      if entity_id != selected:
        related[entity_id] = True
  return list(related)


def map_axis_range(values):
  """按坐标（米）的最小、最大值两侧各留 MAP_PADDING_M，避免点贴在绘图边缘；无值时返回 None。"""
  if not values:
    return None
  return [min(values) - MAP_PADDING_M, max(values) + MAP_PADDING_M]


def callout_label(name):
  """把地图上的选中标签缩短到 CALLOUT_MAX_CHARS 个字符内；完整名称仍在悬停提示和标题显示。"""
  if len(name) <= CALLOUT_MAX_CHARS:
    return name
  return name[: CALLOUT_MAX_CHARS - 1].rstrip() + "…"


def label_width_px(label):
  """按 12px 字体估算标签框宽度（像素），用于避让邻近点和绘图边界。"""
  width = 18.0
  for character in label:
    if ord(character) > 127:
      width += 12.0
    else:
      width += 7.0
  return max(68.0, width)


def callout_box_px(ax_px, ay_px, xanchor, yanchor, width_px):
  """计算标签框相对于选中点的屏幕像素包围盒 (left, top, right, bottom)，+y 向下。"""
  if xanchor == "left":
    left = ax_px
    right = ax_px + width_px
  elif xanchor == "right":
    left = ax_px - width_px
    right = ax_px
  else:
    left = ax_px - width_px * 0.5
    right = ax_px + width_px * 0.5
  if yanchor == "top":
    top = ay_px
    bottom = ay_px + CALLOUT_HEIGHT_PX
  elif yanchor == "bottom":
    top = ay_px - CALLOUT_HEIGHT_PX
    bottom = ay_px
  else:
    top = ay_px - CALLOUT_HEIGHT_PX * 0.5
    bottom = ay_px + CALLOUT_HEIGHT_PX * 0.5
  return left, top, right, bottom


def point_in_box(px_x, px_y, box, pad_px):
  """判断相对坐标点 (px_x, px_y) 是否落在向外扩张 pad_px 的包围盒内。"""
  left, top, right, bottom = box
  return (left - pad_px) <= px_x <= (right + pad_px) and (top - pad_px) <= px_y <= (bottom + pad_px)


def point_near_leader(px_x, px_y, ax_px, ay_px):
  """判断点 (px_x, px_y) 是否靠近从原点到 (ax_px, ay_px) 的引线段。"""
  for fraction in (0.35, 0.65, 0.9):
    delta_x = px_x - ax_px * fraction
    delta_y = px_y - ay_px * fraction
    if (delta_x * delta_x + delta_y * delta_y) <= LEADER_CLEARANCE_PX * LEADER_CLEARANCE_PX:
      return True
  return False


def callout_score(ax_px, ay_px, box, plot_limits, neighbor_offsets, related_offsets):
  """评估候选标签框和引线的遮挡代价：越界最高，其次遮挡关联点或关系连线，最后普通点。"""
  left, top, right, bottom = box
  min_x, max_x, min_y, max_y = plot_limits
  overflow = max(0.0, min_x - left) + max(0.0, right - max_x) + max(0.0, min_y - top) + max(0.0, bottom - max_y)
  penalty = 0.0
  if overflow > 0.0:
    penalty += 500.0 + overflow * 10.0
  for px_x, px_y in related_offsets:
    if point_in_box(px_x, px_y, box, CALLOUT_CLEARANCE_PX):
      penalty += 140.0
    if point_near_leader(px_x, px_y, ax_px, ay_px):
      penalty += 120.0
    for fraction in (0.33, 0.5, 0.67):
      if point_in_box(px_x * fraction, px_y * fraction, box, 8.0):
        penalty += 60.0
  for px_x, px_y in neighbor_offsets:
    if point_in_box(px_x, px_y, box, CALLOUT_CLEARANCE_PX):
      penalty += 25.0
    if point_near_leader(px_x, px_y, ax_px, ay_px):
      penalty += 20.0
  return penalty


def choose_callout_placement(entities, located, selected, related, x_range, y_range, label, height_px):
  """在 8 个方向中挑选最不遮挡邻近点和边界的标签位置，返回 (ax, ay, xanchor, yanchor)。"""
  origin = entities[selected]["position_map"]
  plot_height_px = max(120.0, height_px - PLOT_VERTICAL_CHROME_PX)
  span_x = max(1e-3, x_range[1] - x_range[0])
  span_y = max(1e-3, y_range[1] - y_range[0])
  meters_per_px = max(span_x / ESTIMATED_PLOT_WIDTH_PX, span_y / plot_height_px)
  center_x = 0.5 * (x_range[0] + x_range[1])
  center_y = 0.5 * (y_range[0] + y_range[1])
  half_vis_x = 0.5 * ESTIMATED_PLOT_WIDTH_PX * meters_per_px
  half_vis_y = 0.5 * plot_height_px * meters_per_px
  plot_limits = (
    ((center_x - half_vis_x) - origin[0]) / meters_per_px + 6.0,
    ((center_x + half_vis_x) - origin[0]) / meters_per_px - 6.0,
    (origin[1] - (center_y + half_vis_y)) / meters_per_px + 6.0,
    (origin[1] - (center_y - half_vis_y)) / meters_per_px - 6.0,
  )
  related_set = set(related)
  neighbor_offsets = []
  related_offsets = []
  for entity_id in located:
    if entity_id == selected:
      continue
    position = entities[entity_id]["position_map"]
    px_x = (position[0] - origin[0]) / meters_per_px
    px_y = (origin[1] - position[1]) / meters_per_px
    if entity_id in related_set:
      related_offsets.append((px_x, px_y))
    else:
      neighbor_offsets.append((px_x, px_y))
  width_px = label_width_px(label)
  best = CALLOUT_CANDIDATES[0]
  best_score = None
  for index, candidate in enumerate(CALLOUT_CANDIDATES):
    ax_px, ay_px, xanchor, yanchor = candidate
    box = callout_box_px(ax_px, ay_px, xanchor, yanchor, width_px)
    score = callout_score(ax_px, ay_px, box, plot_limits, neighbor_offsets, related_offsets) + index * 0.1
    if best_score is None or score < best_score:
      best = candidate
      best_score = score
  return best


def relation_line_trace(entities, selected, related):
  """生成从选中实体指向各关联实体的虚线段；无可连线的关联时返回 None。"""
  origin = entities[selected]["position_map"]
  if origin is None:
    return None
  line_x = []
  line_y = []
  for entity_id in related:
    target = entities[entity_id]["position_map"]
    if target is None:
      continue
    line_x.extend((origin[0], target[0], None))
    line_y.extend((origin[1], target[1], None))
  if not line_x:
    return None
  return go.Scatter(
    x=line_x, y=line_y, mode="lines", hoverinfo="skip",
    line=dict(color=SELECTED_COLOR, width=1.6, dash="dot"), opacity=0.65,
  )


def marker_trace(entities, names, entity_ids, color, size, border_width):
  """按给定的实体 ID 列表生成散点轨迹；无实体时返回 None，每个点的 customdata 为实体 ID。"""
  if not entity_ids:
    return None
  x_values = []
  y_values = []
  labels = []
  for entity_id in entity_ids:
    position = entities[entity_id]["position_map"]
    x_values.append(position[0])
    y_values.append(position[1])
    labels.append(escape(names[entity_id]))
  return go.Scatter(
    x=x_values, y=y_values, text=labels, customdata=entity_ids, mode="markers",
    marker=dict(color=color, size=size, opacity=0.92, line=dict(color="white", width=border_width)),
    hovertemplate=HOVER_TEMPLATE,
  )


def add_selected_callout(figure, entities, names, located, selected, related, x_range, y_range, height_px):
  """在选中实体旁加白底引线标签，自动避让邻近点、关系连线和图框边缘；未定位时不添加。"""
  position = entities[selected]["position_map"]
  if position is None or x_range is None or y_range is None:
    return
  label = callout_label(names[selected])
  ax_px, ay_px, xanchor, yanchor = choose_callout_placement(
    entities, located, selected, related, x_range, y_range, label, height_px,
  )
  figure.add_annotation(
    x=position[0], y=position[1], text=escape(label), showarrow=True,
    ax=ax_px, ay=ay_px, xanchor=xanchor, yanchor=yanchor,
    arrowcolor=SELECTED_COLOR, arrowwidth=1.5, arrowhead=0, standoff=10,
    bgcolor="rgba(255,255,255,0.94)", bordercolor=SELECTED_COLOR, borderwidth=1, borderpad=4,
    font=dict(color=SELECTED_COLOR, size=12),
  )


def entity_figure(entities, names, selected, related, version, height_px):
  """按分层顺序绘制关系连线、普通实体、关联实体和选中实体；坐标轴等比例并保持缩放状态。"""
  located = []
  normal_ids = []
  related_ids = []
  related_set = set(related)
  x_values = []
  y_values = []
  for entity_id, row in entities.items():
    if row["position_map"] is None:
      continue
    located.append(entity_id)
    x_values.append(row["position_map"][0])
    y_values.append(row["position_map"][1])
    if entity_id == selected:
      continue
    if entity_id in related_set:
      related_ids.append(entity_id)
    else:
      normal_ids.append(entity_id)
  figure = go.Figure()
  line_trace = relation_line_trace(entities, selected, related)
  if line_trace is not None:
    figure.add_trace(line_trace)
  normal_trace = marker_trace(entities, names, normal_ids, ENTITY_COLOR, 10, 1.2)
  if normal_trace is not None:
    figure.add_trace(normal_trace)
  related_trace = marker_trace(entities, names, related_ids, RELATED_COLOR, 13, 2.0)
  if related_trace is not None:
    figure.add_trace(related_trace)
  if entities[selected]["position_map"] is not None:
    figure.add_trace(marker_trace(entities, names, [selected], SELECTED_COLOR, 18, 2.5))
  x_range = map_axis_range(x_values)
  y_range = map_axis_range(y_values)
  add_selected_callout(figure, entities, names, located, selected, related, x_range, y_range, height_px)
  figure.update_layout(
    height=height_px, margin=dict(l=8, r=8, t=8, b=8), showlegend=False,
    template="plotly_white", plot_bgcolor="#fafbfc", paper_bgcolor="rgba(0,0,0,0)",
    clickmode="event+select", dragmode="pan", uirevision=version,
    hoverlabel=dict(bgcolor="white", font_size=13),
  )
  grid = dict(showgrid=True, gridcolor="#e6e9ee", zeroline=False, showline=True, linecolor="#d0d7de", mirror=True)
  figure.update_xaxes(title_text="X（米）", range=x_range, **grid)
  figure.update_yaxes(title_text="Y（米）", range=y_range, scaleanchor="x", scaleratio=1, **grid)
  return figure
