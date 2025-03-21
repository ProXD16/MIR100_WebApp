# draw_line_mode_callbacks.py
from dash import Input, Output, State, callback, callback_context, no_update
import plotly.graph_objects as go
import json
from function_draw_mode.save_lines import save_lines_to_json
from page_home.shared_data import all_lines, all_arcs
import time

DEBOUNCE_TIMEOUT = 0.1
last_relayout_time = 0


# Callback to open the draw method modal
@callback(
    Output("draw-method-modal", "is_open"),
    Input("draw-line-button", "n_clicks"),
    State("draw-line-mode", "data"),
    State("draw-method-modal", "is_open"),
    prevent_initial_call=True,
)
def open_draw_method_modal(n_clicks, draw_line_mode, is_open):
    ctx = callback_context
    if not ctx.triggered:
        return no_update

    button_id = ctx.triggered[0]['prop_id'].split('.')[0]
    if button_id == 'draw-line-button' and draw_line_mode:
        return False
    return not is_open


# Callback to set the draw method (manual or coordinate) and close the modal
@callback(
    Output("draw-method", "data"),
    Output("draw-method-modal", "is_open", allow_duplicate=True),
    Input("manual-draw-button", "n_clicks"),
    Input("coordinate-draw-button", "n_clicks"),
    State("draw-method-modal", "is_open"),
    prevent_initial_call=True,
)
def set_draw_method(manual_clicks, coordinate_clicks, is_open):
    ctx = callback_context
    if not ctx.triggered:
        return no_update, no_update
    button_id = ctx.triggered[0]['prop_id'].split('.')[0]
    if button_id == "manual-draw-button":
        return "manual", False
    elif button_id == "coordinate-draw-button":
        return "coordinate",
    return "", is_open


# Callback to open coordinate modal when "coordinate" draw method is selected
@callback(
    Output("coordinate-modal", "is_open"),
    Input("draw-method", "data"),
    State("coordinate-modal", "is_open"),
    prevent_initial_call=True,
)
def open_coordinate_modal(draw_method, is_open):
    if draw_method == "coordinate":
        return True
    return False


# Callback to draw line based on coordinates entered in the modal
@callback(
    Output("map-image-draw-mode", "figure"),
    Output("coordinate-modal", "is_open", allow_duplicate=True),
    Input("draw-button", "n_clicks"),
    State("start-x", "value"),
    State("start-y", "value"),
    State("end-x", "value"),
    State("end-y", "value"),
    State("map-image-draw-mode", "figure"),
    prevent_initial_call=True,
)
def draw_line_coordinate(n_clicks, start_x, start_y, end_x, end_y, figure):
    from page_home.shared_data import all_lines, all_arcs
    if n_clicks is None:
        return no_update, no_update

    try:
        start_x = float(start_x)
        start_y = float(start_y)
        end_x = float(end_x)
        end_y = float(end_y)
    except (ValueError, TypeError):
        print("Invalid coordinates entered.")
        return figure, no_update

    # Add line to the figure
    line_data = go.Scatter(
        x=[start_x, end_x],
        y=[start_y, end_y],
        mode="lines",
        line=dict(color="blue", width=2),
        showlegend=False,
    )
    figure["data"].append(line_data)
    all_lines.append({"type": "line", "x": [start_x, end_x], "y": [start_y, end_y]})
    return figure, False


# Callback to store the start point when the user clicks on the graph
@callback(
    Output("line-coordinates", "data"),
    Input("map-image-draw-mode", "clickData"),
    State("draw-method", "data"),
    State("draw-line-mode", "data"),
    prevent_initial_call=True,
)
def store_start_point(clickData, draw_method, draw_line_mode):
    print("store_start_point called")
    print("draw_method:", draw_method)
    print("clickData:", clickData)
    print("draw_line_mode:", draw_line_mode)

    if draw_method == "manual" and clickData and draw_line_mode:
        x = clickData["points"][0]["x"]
        y = clickData["points"][0]["y"]
        return {"start_x": x, "start_y": y}
    return {}


@callback(
    Output("map-image-draw-mode", "figure", allow_duplicate=True),
    Input("map-image-draw-mode", "relayoutData"),
    Input("draw-method", "data"),
    State("line-coordinates", "data"),
    State("map-image-draw-mode", "figure"),
    State("draw-line-mode", "data"),
    prevent_initial_call=True,
)
def draw_line_on_release(relayoutData, draw_method, line_coordinates, figure, draw_line_mode):
    global last_relayout_time

    current_time = time.time()
    if current_time - last_relayout_time < DEBOUNCE_TIMEOUT:
        print("Debounced relayout event.")
        return no_update
    last_relayout_time = current_time

    if relayoutData:
        if 'shapes' in relayoutData:
            all_lines.clear()
            if "shapes" in figure["layout"]:
                new_lines = []
                for shape in figure["layout"]["shapes"]:
                    if shape["type"] == "line":
                        new_lines.append(
                            {
                                "type": "line",
                                "x": [shape["x0"], shape["x1"]],
                                "y": [shape["y0"], shape["y1"]],
                            }
                        )
                all_lines.extend(new_lines)
            return figure
        if draw_method == "manual" and draw_line_mode:
            x_start, y_start, x_end, y_end = None, None, None, None

            if "shapes" in relayoutData and len(relayoutData["shapes"]) > 0:
                shape = relayoutData["shapes"][-1]
                x_start = shape["x0"]
                y_start = shape["y0"]
                x_end = shape["x1"]
                y_end = shape["y1"]
            elif (
                'xaxis.range[0]' in relayoutData
                and 'yaxis.range[0]' in relayoutData
                and 'xaxis.range[1]' in relayoutData
                and 'yaxis.range[1]' in relayoutData
                and line_coordinates
                and "start_x" in line_coordinates
            ):
                x_start = line_coordinates["start_x"]
                y_start = line_coordinates["start_y"]
                x_end = relayoutData['xaxis.range[1]']
                y_end = relayoutData['yaxis.range[0]']
            if (
                x_start is not None
                and y_start is not None
                and x_end is not None
                and y_end is not None
            ):
                line_data = go.Scatter(
                    x=[x_start, x_end],
                    y=[y_start, y_end],
                    mode="lines",
                    line=dict(color="green", width=2),
                    showlegend=False,
                )
                figure["data"].append(line_data)
                all_lines.append(
                    {"type": "line", "x": [x_start, x_end], "y": [y_start, y_end]}
                )
                return figure
            else:
                return figure
        else:
            return no_update
    else:
        return no_update


# Callback to clear the stored start point after drawing
@callback(
    Output("line-coordinates", "data", allow_duplicate=True),
    Input("map-image-draw-mode", "figure"),
    prevent_initial_call=True,
)
def clear_start_point(figure):
    return {}


@callback(
    Output("draw-line-mode", "data"),
    Input("draw-line-button", "n_clicks"),
    State("draw-line-mode", "data"),
    prevent_initial_call=True,
)
def toggle_draw_line_mode(n_clicks, current_state):
    return not current_state


@callback(
    Output("map-image-draw-mode", "dragmode"),
    Input("draw-line-mode", "data"),
    prevent_initial_call=True,
)
def update_drag_mode(draw_line_mode):
    if draw_line_mode:
        return "drawline"
    else:
        return "pan"


@callback(
    Output("draw-line-button", "style"),
    Input("draw-line-mode", "data"),
    State("button-style-store", "data"),
)
def update_button_style(is_active, button_style_store):
    default_style = button_style_store["draw_line_button"]
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
    