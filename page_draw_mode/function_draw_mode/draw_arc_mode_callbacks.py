from dash import Input, Output, State, callback, callback_context, no_update
import plotly.graph_objects as go
import numpy as np
import math
from page_draw_mode.function_draw_mode.save_lines import save_lines_to_json
from page_home.shared_data import all_arcs

def circle_from_3_points(P1, P2, P3):
    x1, y1 = P1
    x2, y2 = P2
    x3, y3 = P3
    A = x1 * (y2 - y3) - y1 * (x2 - x3) + x2 * y3 - x3 * y2
    B = (x1**2 + y1**2) * (y3 - y2) + (x2**2 + y2**2) * (y1 - y3) + (x3**2 + y3**2) * (y2 - y1)
    C = (x1**2 + y1**2) * (x2 - x3) + (x2**2 + y2**2) * (x3 - x1) + (x3**2 + y3**2) * (x1 - x2)
    D = (x1**2 + y1**2) * (x3 * y2 - x2 * y3) + (x2**2 + y2**2) * (x1 * y3 - x3 * y1) + (x3**2 + y3**2) * (x2 * y1 - x1 * y2)
    if A == 0:
        return None
    center_x = -B / (2 * A)
    center_y = -C / (2 * A)
    radius = np.sqrt((B**2 + C**2 - 4 * A * D) / (4 * A**2))
    return center_x, center_y, radius

def draw_arc(center_x, center_y, start_angle, end_angle, radius, color="red"):
    n_points = 50
    angles = np.linspace(start_angle, end_angle, n_points)
    x = center_x + radius * np.cos(angles)
    y = center_y + radius * np.sin(angles)
    return go.Scatter(
        x=x,
        y=y,
        mode="lines",
        line=dict(color=color, width=2),
        showlegend=False,
    )

@callback(
    Output("draw-arc-method-modal", "is_open"),
    Input("draw-arc-button", "n_clicks"),
    State("draw-arc-mode", "data"),
    State("draw-arc-method-modal", "is_open"),
    prevent_initial_call=True,
)
def open_draw_arc_method_modal(n_clicks, draw_arc_mode, is_open):
    ctx = callback_context
    if not ctx.triggered:
        return no_update

    button_id = ctx.triggered[0]['prop_id'].split('.')[0]
    if button_id == 'draw-arc-button' and draw_arc_mode:
        return False
    return not is_open

@callback(
    Output("draw-arc-method", "data"),
    Output("draw-arc-method-modal", "is_open", allow_duplicate=True),
    Input("manual-draw-arc-button", "n_clicks"),
    Input("coordinate-draw-arc-button", "n_clicks"),
    State("draw-arc-method-modal", "is_open"),
    prevent_initial_call=True,
)
def set_draw_arc_method(manual_clicks, coordinate_clicks, is_open):
    ctx = callback_context
    if not ctx.triggered:
        return no_update, no_update

    button_id = ctx.triggered[0]['prop_id'].split('.')[0]
    if button_id == "manual-draw-arc-button":
        return "manual", False
    elif button_id == "coordinate-draw-arc-button":
        return "coordinate", False
    return "", is_open

@callback(
    Output("coordinate-arc-modal", "is_open"),
    Input("draw-arc-method", "data"),
    State("coordinate-arc-modal", "is_open"),
    prevent_initial_call=True,
)
def open_coordinate_arc_modal(draw_arc_method, is_open):
    if draw_arc_method == "coordinate":
        return True
    return False

@callback(
    Output("map-image-draw-mode", "figure", allow_duplicate=True),
    Output("coordinate-arc-modal", "is_open", allow_duplicate=True),  
    Input("draw-arc-button-coordinate", "n_clicks"),
    State("point1-x", "value"),
    State("point1-y", "value"),
    State("point2-x", "value"),
    State("point2-y", "value"),
    State("point3-x", "value"),
    State("point3-y", "value"),
    State("map-image-draw-mode", "figure"),
    prevent_initial_call=True,
)
def draw_arc_coordinate(n_clicks, p1x, p1y, p2x, p2y, p3x, p3y, figure):
    from page_home.shared_data import all_lines, all_arcs
    if n_clicks is None:
        return no_update, no_update
    try:
        p1 = (float(p1x), float(p1y))
        p2 = (float(p2x), float(p2y))
        p3 = (float(p3x), float(p3y))
    except (ValueError, TypeError):
        print("Invalid coordinates entered for arc.")
        return figure, no_update
    circle_params = circle_from_3_points(p1, p2, p3)
    if circle_params is None:
        print("Could not determine circle from these points.")
        return figure, no_update
    center_x, center_y, radius = circle_params
    angle_p1 = math.atan2(p1[1] - center_y, p1[0] - center_x)
    angle_p2 = math.atan2(p2[1] - center_y, p2[0] - center_x)
    angle_p3 = math.atan2(p3[1] - center_y, p3[0] - center_x)
    if (angle_p1 <= angle_p3 <= angle_p2) or (angle_p1 >= angle_p3 and angle_p3 <= angle_p2):
        start_point, end_point = p1, p3
        start_angle, end_angle = angle_p3, angle_p1
    else:
        start_point, end_point = p3, p1
        start_angle, end_angle = angle_p1, angle_p3
    arc = draw_arc(center_x, center_y, start_angle, end_angle, radius)
    figure["data"].append(arc)
    all_arcs.append({
        "type": "arc",
        "start_x": start_point[0],  
        "start_y": start_point[1],
        "end_x": end_point[0],  
        "end_y": end_point[1],
        "center_x": center_x,  
        "center_y": center_y,
        "radius": radius,
        "start_angle": start_angle,
        "end_angle": end_angle,
    })

    return figure, False

@callback(
    Output("arc-coordinates", "data"),
    Input("map-image-draw-mode", "clickData"),
    State("draw-arc-method", "data"),
    State("draw-arc-mode", "data"),

    prevent_initial_call=True,
)
def store_start_point_arc(clickData, draw_arc_method, draw_arc_mode):
    if draw_arc_method == "manual" and clickData and draw_arc_mode:
        x = clickData["points"][0]["x"]
        y = clickData["points"][0]["y"]
        return {"center_x": x, "center_y": y}
    return {}

@callback(
    Output("map-image-draw-mode", "figure", allow_duplicate=True),
    Input("map-image-draw-mode", "relayoutData"),
    Input("draw-arc-method", "data"),
    State("arc-coordinates", "data"),
    State("map-image-draw-mode", "figure"),
    State("draw-arc-mode", "data"),
    prevent_initial_call=True,
)
def draw_arc_on_release(relayoutData, draw_arc_method, arc_coordinates, figure, draw_arc_mode):
    if draw_arc_method == "manual" and relayoutData and arc_coordinates and "center_x" in arc_coordinates and draw_arc_mode:
        if 'xaxis.range[0]' in relayoutData and 'yaxis.range[0]' in relayoutData and 'xaxis.range[1]' in relayoutData and 'yaxis.range[1]' in relayoutData:
            center_x = arc_coordinates["center_x"]
            center_y = arc_coordinates["center_y"]
            end_x = relayoutData['xaxis.range[1]']
            end_y = relayoutData['yaxis.range[0]']
            radius = np.sqrt((end_x - center_x) ** 2 + (end_y - center_y) ** 2)
            start_angle = 0
            end_angle = 2 * np.pi
            arc = draw_arc(center_x, center_y, start_angle, end_angle, radius, color="purple")

            figure["data"].append(arc)
            all_arcs.append({
                "type": "arc",
                "center_x": center_x,
                "center_y": center_y,
                "radius": radius,
                "start_angle": start_angle,
                "end_angle": end_angle,
            })

            return figure
        else:
            return no_update
    else:
        return no_update

@callback(
    Output("arc-coordinates", "data", allow_duplicate=True),
    Input("map-image-draw-mode", "figure"),
    prevent_initial_call=True,
)
def clear_start_point_arc(figure):
    return {}

@callback(
    Output("draw-arc-mode", "data"),
    Input("draw-arc-button", "n_clicks"),
    State("draw-arc-mode", "data"),
    prevent_initial_call=True,
)
def toggle_draw_arc_mode(n_clicks, current_state):
    return not current_state

@callback(
    Output("map-image-draw-mode", "dragmode", allow_duplicate=True),
    Input("draw-line-mode", "data"),
    Input("draw-arc-mode", "data"),
    prevent_initial_call=True,
)
def update_drag_mode(draw_line_mode, draw_arc_mode):
    ctx = callback_context
    triggered_id = ctx.triggered[0]['prop_id'].split('.')[0] if ctx.triggered else None

    if triggered_id == "draw-line-mode":
        if draw_line_mode:
            return "drawline"
        else:
            return "pan"
    elif triggered_id == "draw-arc-mode":
        if draw_arc_mode:
            return "drawarc"
        else:
            return "pan"

    return "pan"

@callback(
    Output("draw-arc-button", "style"),
    Input("draw-arc-mode", "data"),
    State("button-style-store", "data"),
    prevent_initial_call=True
)
def update_button_arc_style(is_active, button_style_store):
    default_style = button_style_store["draw_arc_button"]
    active_button_style = {
        "padding": "8px 16px",
        "border": "1px solid #2ecc71",
        "color": "white",
        "background-color": "#2ecc71",
        "border-radius": "5px",
        "transition": "all 0.3s ease-in-out",
        "cursor": "pointer",
    }
    if is_active:
        return active_button_style
    else:
        return default_style
    
@callback(
    Output("draw-mode-output", "children", allow_duplicate=True),
    Input("save-lines-button", "n_clicks"),
    prevent_initial_call=True,
)
def save_lines(n_clicks):
    from page_home.shared_data import all_lines, all_arcs
    if n_clicks:
        all_drawn_objects = all_lines + all_arcs
        save_lines_to_json(all_drawn_objects)
        return "Lines and arcs saved successfully!"
    return ""

@callback(
    Output("draw-mode-output", "children", allow_duplicate=True),
    Input("clear-all-lines-button", "n_clicks"),
    prevent_initial_call=True,
)
def clear_all_lines(n_clicks):
    from page_home.shared_data import all_lines, all_arcs
    if n_clicks:
        all_lines = []
        all_arcs = []
        clear_all_lines = []
        save_lines_to_json(clear_all_lines)
        return "All lines are cleaned"
    return ""
