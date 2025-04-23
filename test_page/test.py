from dash import Dash, html, dcc, Input, Output, State
import dash_bootstrap_components as dbc
import plotly.graph_objects as go

# Assume the create_draw_mode_layout() function is defined as in the previous response.  It returns the layout.
from draw_mode import create_draw_mode_layout, read_map_info
import json, dash  # Import the json module
# Initialize Dash app
app = Dash(__name__, external_stylesheets=[dbc.themes.BOOTSTRAP, dbc.icons.FONT_AWESOME])
app.layout = create_draw_mode_layout()

# --- HELPER FUNCTIONS ---
def update_button_style(button_id, button_style, active_button_style, button_style_store, current_id):
    """Updates button styles based on which button is active."""
    new_style_store = button_style_store.copy()
    for id in ["draw-line-button", "draw-arc-button", "draw-spline3-button", "draw-spline5-button"]:
        if id == button_id:
            if button_style_store[id]["background-color"] == active_button_style["background-color"]:
                  new_style_store[id] = button_style
            else:
                new_style_store[id] = active_button_style
        else:
            new_style_store[id] = button_style

    output_style = new_style_store[current_id]
    return output_style, new_style_store

def draw_line_on_graph(figure, start_x, start_y, end_x, end_y, line_color='blue', line_width=2):
    """Draws a line on the given Plotly figure."""
    shape = {
        'type': 'line',
        'x0': start_x,
        'y0': start_y,
        'x1': end_x,
        'y1': end_y,
        'line': {'color': line_color, 'width': line_width},
    }
    figure['layout']['shapes'] = figure['layout']['shapes'] + (shape,)
    return figure
def draw_spline3_on_graph(figure, p1_x, p1_y, p2_x, p2_y, p3_x, p3_y, p4_x, p4_y, line_color='blue', line_width=2, num_points=50):
    import numpy as np
    """Draws a cubic Bezier spline on the given Plotly figure."""
    t = np.linspace(0, 1, num_points)  # Parameter from 0 to 1
    x = (1-t)**3 * p1_x + 3*(1-t)**2 * t * p2_x + 3*(1-t) * t**2 * p3_x + t**3 * p4_x
    y = (1-t)**3 * p1_y + 3*(1-t)**2 * t * p2_y + 3*(1-t) * t**2 * p3_y + t**3 * p4_y

    figure['data'].append(go.Scatter(x=x, y=y, mode='lines', line={'color': line_color, 'width': line_width}))
    return figure
def draw_spline5_on_graph(figure, p1_x, p1_y, p2_x, p2_y, p3_x, p3_y, p4_x, p4_y, p5_x, p5_y, p6_x, p6_y, line_color='blue', line_width=2, num_points=50):
    import numpy as np
    """Draws a quintic Bezier spline on the given Plotly figure."""
    t = np.linspace(0, 1, num_points)

    x = (1-t)**5 * p1_x + 5*(1-t)**4 * t * p2_x + 10*(1-t)**3 * t**2 * p3_x + 10*(1-t)**2 * t**3 * p4_x + 5*(1-t) * t**4 * p5_x + t**5 * p6_x
    y = (1-t)**5 * p1_y + 5*(1-t)**4 * t * p2_y + 10*(1-t)**3 * t**2 * p3_y + 10*(1-t)**2 * t**3 * p4_y + 5*(1-t) * t**4 * p5_y + t**5 * p6_y

    figure['data'].append(go.Scatter(x=x, y=y, mode='lines', line={'color': line_color, 'width': line_width}))
    return figure

# --- CALLBACKS ---

@app.callback(
    [
        Output("draw-method-modal", "is_open"),
        Output('draw-line-mode', 'data'),
        Output('button-style-store', 'data'),
        Output('draw-line-button', 'style')
    ],
    [Input("draw-line-button", "n_clicks")],
    [State("draw-method-modal", "is_open"),
     State('button-style-store', 'data'),
     State('draw-line-button', 'style')]
)
def toggle_draw_method_modal(n_clicks, is_open, button_style_store, button_style):
    if n_clicks:
        active_button_style = {
            "background-color": "#2ecc71",
            "color": "white",
            "box-shadow": "0 2px 5px rgba(0, 0, 0, 0.3)",
        }
        button_style = {
            "padding": "10px 20px",
            "border": "none",
            "color": "#FFFFFF",
            "background-color": "#5DADE2",  # Softer blue
            "border-radius": "8px",
            "transition": "background-color 0.3s ease",
            "cursor": "pointer",
            "box-shadow": "0 2px 5px rgba(0, 0, 0, 0.2)",  # Subtle shadow
        }

        current_id = "draw-line-button"
        output_style, new_style_store = update_button_style(current_id, button_style, active_button_style, button_style_store, current_id)
        return not is_open, True, new_style_store, output_style
    return is_open, False, button_style_store, button_style

@app.callback(
    [
        Output("draw-arc-method-modal", "is_open"),
        Output('draw-arc-mode', 'data'),
        Output('button-style-store', 'data'),
        Output('draw-arc-button', 'style')
    ],
    [Input("draw-arc-button", "n_clicks")],
    [State("draw-arc-method-modal", "is_open"),
     State('button-style-store', 'data'),
     State('draw-arc-button', 'style')]
)
def toggle_draw_arc_method_modal(n_clicks, is_open, button_style_store, button_style):
    if n_clicks:
        active_button_style = {
            "background-color": "#2ecc71",
            "color": "white",
            "box-shadow": "0 2px 5px rgba(0, 0, 0, 0.3)",
        }
        button_style = {
            "padding": "10px 20px",
            "border": "none",
            "color": "#FFFFFF",
            "background-color": "#5DADE2",  # Softer blue
            "border-radius": "8px",
            "transition": "background-color 0.3s ease",
            "cursor": "pointer",
            "box-shadow": "0 2px 5px rgba(0, 0, 0, 0.2)",  # Subtle shadow
        }

        current_id = "draw-arc-button"
        output_style, new_style_store = update_button_style(current_id, button_style, active_button_style, button_style_store, current_id)
        return not is_open, True, new_style_store, output_style
    return is_open, False, button_style_store, button_style

@app.callback(
    [
        Output("draw-spline3-method-modal", "is_open"),
        Output('draw-spline3-mode', 'data'),
        Output('button-style-store', 'data'),
        Output('draw-spline3-button', 'style')
    ],
    [Input("draw-spline3-button", "n_clicks")],
    [State("draw-spline3-method-modal", "is_open"),
     State('button-style-store', 'data'),
     State('draw-spline3-button', 'style')]
)
def toggle_draw_spline3_method_modal(n_clicks, is_open, button_style_store, button_style):
    if n_clicks:
        active_button_style = {
            "background-color": "#2ecc71",
            "color": "white",
            "box-shadow": "0 2px 5px rgba(0, 0, 0, 0.3)",
        }
        button_style = {
            "padding": "10px 20px",
            "border": "none",
            "color": "#FFFFFF",
            "background-color": "#5DADE2",  # Softer blue
            "border-radius": "8px",
            "transition": "background-color 0.3s ease",
            "cursor": "pointer",
            "box-shadow": "0 2px 5px rgba(0, 0, 0, 0.2)",  # Subtle shadow
        }
        current_id = "draw-spline3-button"
        output_style, new_style_store = update_button_style(current_id, button_style, active_button_style, button_style_store, current_id)
        return not is_open, True, new_style_store, output_style
    return is_open, False, button_style_store, button_style

@app.callback(
    [
        Output("draw-spline5-method-modal", "is_open"),
        Output('draw-spline5-mode', 'data'),
        Output('button-style-store', 'data'),
        Output('draw-spline5-button', 'style')
    ],
    [Input("draw-spline5-button", "n_clicks")],
    [State("draw-spline5-method-modal", "is_open"),
     State('button-style-store', 'data'),
     State('draw-spline5-button', 'style')]
)
def toggle_draw_spline5_method_modal(n_clicks, is_open, button_style_store, button_style):
    if n_clicks:
        active_button_style = {
            "background-color": "#2ecc71",
            "color": "white",
            "box-shadow": "0 2px 5px rgba(0, 0, 0, 0.3)",
        }
        button_style = {
            "padding": "10px 20px",
            "border": "none",
            "color": "#FFFFFF",
            "background-color": "#5DADE2",  # Softer blue
            "border-radius": "8px",
            "transition": "background-color 0.3s ease",
            "cursor": "pointer",
            "box-shadow": "0 2px 5px rgba(0, 0, 0, 0.2)",  # Subtle shadow
        }
        current_id = "draw-spline5-button"
        output_style, new_style_store = update_button_style(current_id, button_style, active_button_style, button_style_store, current_id)
        return not is_open, True, new_style_store, output_style
    return is_open, False, button_style_store, button_style

@app.callback(
    [Output("coordinate-modal", "is_open"),
     Output("draw-method", "data")],
    [Input("manual-draw-button", "n_clicks"),
     Input("coordinate-draw-button", "n_clicks"),
     Input("cancel-button", "n_clicks")],
    [State("coordinate-modal", "is_open")],
    prevent_initial_call=True
)
def toggle_coordinate_modal(manual_clicks, coordinate_clicks, cancel_clicks, is_open):
    ctx = dash.callback_context
    if not ctx.triggered:
        return is_open, 'manual'
    else:
        button_id = ctx.triggered[0]['prop_id'].split('.')[0]
    if button_id == "manual-draw-button":
        return False, 'manual'
    elif button_id == "coordinate-draw-button":
        return True, 'coordinate'
    else:
        return False, 'manual'

@app.callback(
    [Output("coordinate-arc-modal", "is_open"),
     Output("draw-arc-method", "data")],
    [Input("manual-draw-arc-button", "n_clicks"),
     Input("coordinate-draw-arc-button", "n_clicks"),
     Input("cancel-arc-button", "n_clicks")],
    [State("coordinate-arc-modal", "is_open")],
    prevent_initial_call=True
)
def toggle_coordinate_arc_modal(manual_clicks, coordinate_clicks, cancel_clicks, is_open):
    ctx = dash.callback_context
    if not ctx.triggered:
        return is_open, 'manual'
    else:
        button_id = ctx.triggered[0]['prop_id'].split('.')[0]
    if button_id == "manual-draw-arc-button":
        return False, 'manual'
    elif button_id == "coordinate-draw-arc-button":
        return True, 'coordinate'
    else:
        return False, 'manual'
@app.callback(
    Output('coordinate-spline3-modal', 'is_open'),
    [Input('manual-draw-spline3-button', 'n_clicks'),
     Input('coordinate-draw-spline3-button', 'n_clicks'),
     Input('cancel-spline3-button', 'n_clicks')],
    [State('coordinate-spline3-modal', 'is_open')],
    prevent_initial_call=True
)
def toggle_spline3_coordinate_modal(manual_clicks, coordinate_clicks, cancel_clicks, is_open):
    ctx = dash.callback_context
    if not ctx.triggered:
        return is_open
    else:
        button_id = ctx.triggered[0]['prop_id'].split('.')[0]
    if button_id == 'manual-draw-spline3-button':
        return False
    elif button_id == 'coordinate-draw-spline3-button':
        return True
    else:  # cancel button
        return False

@app.callback(
    Output('coordinate-spline5-modal', 'is_open'),
    [Input('manual-draw-spline5-button', 'n_clicks'),
     Input('coordinate-draw-spline5-button', 'n_clicks'),
     Input('cancel-spline5-button', 'n_clicks')],
    [State('coordinate-spline5-modal', 'is_open')],
    prevent_initial_call=True
)
def toggle_spline5_coordinate_modal(manual_clicks, coordinate_clicks, cancel_clicks, is_open):
    ctx = dash.callback_context
    if not ctx.triggered:
        return is_open
    else:
        button_id = ctx.triggered[0]['prop_id'].split('.')[0]
    if button_id == 'manual-draw-spline5-button':
        return False
    elif button_id == 'coordinate-draw-spline5-button':
        return True
    else:  # cancel button
        return False

@app.callback(
    Output('map-image-draw-mode', 'figure'),
    [Input('draw-button', 'n_clicks'),
     Input('draw-arc-button-coordinate', 'n_clicks'),
     Input('draw-spline3-button-coordinate', 'n_clicks'),
     Input('draw-spline5-button-coordinate', 'n_clicks'),
     Input('clear-lines-button', 'n_clicks'),
     Input('clear-all-lines-button', 'n_clicks')],
    [State('start-x', 'value'), State('start-y', 'value'), State('end-x', 'value'), State('end-y', 'value'),
     State('point1-x', 'value'), State('point1-y', 'value'), State('point2-x', 'value'), State('point2-y', 'value'), State('point3-x', 'value'), State('point3-y', 'value'),
     State('spline3-point1-x', 'value'), State('spline3-point1-y', 'value'), State('spline3-point2-x', 'value'), State('spline3-point2-y', 'value'), State('spline3-point3-x', 'value'), State('spline3-point3-y', 'value'), State('spline3-point4-x', 'value'), State('spline3-point4-y', 'value'),
     State('spline5-point1-x', 'value'), State('spline5-point1-y', 'value'), State('spline5-point2-x', 'value'), State('spline5-point2-y', 'value'), State('spline5-point3-x', 'value'), State('spline5-point3-y', 'value'), State('spline5-point4-x', 'value'), State('spline5-point4-y', 'value'), State('spline5-point5-x', 'value'), State('spline5-point5-y', 'value'), State('spline5-point6-x', 'value'), State('spline5-point6-y', 'value'),
     State('map-image-draw-mode', 'figure')] ,
    prevent_initial_call=True
)
def update_graph(line_clicks, arc_clicks, spline3_clicks, spline5_clicks, clear_clicks, clear_all_clicks,
                 start_x, start_y, end_x, end_y,
                 p1_x, p1_y, p2_x, p2_y, p3_x, p3_y,
                 spline3_p1_x, spline3_p1_y, spline3_p2_x, spline3_p2_y, spline3_p3_x, spline3_p3_y, spline3_p4_x, spline3_p4_y,
                 spline5_p1_x, spline5_p1_y, spline5_p2_x, spline5_p2_y, spline5_p3_x, spline5_p3_y, spline5_p4_x, spline5_p4_y, spline5_p5_x, spline5_p5_y, spline5_p6_x, spline5_p6_y,
                 figure):
    ctx = dash.callback_context
    if not ctx.triggered:
        return figure
    else:
        button_id = ctx.triggered[0]['prop_id'].split('.')[0]
    if button_id == 'draw-button':
        if all([start_x, start_y, end_x, end_y]):
            figure = draw_line_on_graph(figure, start_x, start_y, end_x, end_y)
    elif button_id == 'draw-arc-button-coordinate':
          # Drawing an arc using three points is more complex and requires a library or custom calculation.  This is a placeholder.
        if all([p1_x, p1_y, p2_x, p2_y, p3_x, p3_y]):
            #Placeholder - replace with actual arc drawing logic
            figure = draw_line_on_graph(figure, p1_x, p1_y, p2_x, p2_y, line_color='green') #Example
            figure = draw_line_on_graph(figure, p2_x, p2_y, p3_x, p3_y, line_color='green') #Example
    elif button_id == 'draw-spline3-button-coordinate':
        if all([spline3_p1_x, spline3_p1_y, spline3_p2_x, spline3_p2_y, spline3_p3_x, spline3_p3_y, spline3_p4_x, spline3_p4_y]):
            figure = draw_spline3_on_graph(figure, spline3_p1_x, spline3_p1_y, spline3_p2_x, spline3_p2_y, spline3_p3_x, spline3_p3_y, spline3_p4_x, spline3_p4_y)
    elif button_id == 'draw-spline5-button-coordinate':
        if all([spline5_p1_x, spline5_p1_y, spline5_p2_x, spline5_p2_y, spline5_p3_x, spline5_p3_y, spline5_p4_x, spline5_p4_y, spline5_p5_x, spline5_p5_y, spline5_p6_x, spline5_p6_y]):
            figure = draw_spline5_on_graph(figure, spline5_p1_x, spline5_p1_y, spline5_p2_x, spline5_p2_y, spline5_p3_x, spline5_p3_y, spline5_p4_x, spline5_p4_y, spline5_p5_x, spline5_p5_y, spline5_p6_x, spline5_p6_y)
    elif button_id == 'clear-lines-button':
        figure['layout']['shapes'] = []  # Clear shapes (lines, arcs)
        figure['data'] = figure['data'][:len(figure['data'])-1]
    elif button_id == 'clear-all-lines-button':
        map_info = read_map_info('static/map_image.info')
        grid_size = 1
        grid_color = "#E0E0E0"
        axis_color = "#333333"
        grid_lines = []
        max_x = map_info['width'] * map_info['resolution']
        max_y = map_info['height'] * map_info['resolution']
        for i in range(0, int(max_x), grid_size):
            grid_lines.append(
                go.Scatter(
                    x=[i, i],
                    y=[0, max_y],
                    mode="lines",
                    line=dict(color=grid_color, width=1),
                    hoverinfo="none",
                    showlegend=False,
                )
            )
            grid_lines.append(
                go.Scatter(
                    x=[0, max_x],
                    y=[i, i],
                    mode="lines",
                    line=dict(color=grid_color, width=1),
                    hoverinfo="none",
                    showlegend=False,
                )
            )
        axis_lines = [
            go.Scatter(
                x=[0, max_x],
                y=[0, 0],
                mode="lines",
                line=dict(color=axis_color, width=2),
                hoverinfo="none",
                showlegend=False,
            ),
            go.Scatter(
                x=[0, 0],
                y=[0, max_y],
                mode="lines",
                line=dict(color=axis_color, width=2),
                hoverinfo="none",
                showlegend=False,
            ),
        ]
        figure['data'] = grid_lines + axis_lines
        figure['layout']['shapes'] = []  # Clear shapes (lines, arcs)


    return figure

@app.callback(
    Output('line-coordinates', 'data'),
    Input('map-image-draw-mode', 'relayoutData'),
    State('line-coordinates', 'data'),
    State('draw-line-mode', 'data'),
    prevent_initial_call=True
)
def store_line_coordinates(relayoutData, existing_coordinates, draw_line_mode):
    if draw_line_mode == True:
        if relayoutData and 'shapes' in relayoutData:
            return relayoutData
        else:
            return existing_coordinates
    else:
        return {}

@app.callback(
    Output('map-image-draw-mode', 'figure'),
    Input('line-coordinates', 'data'),
    State('map-image-draw-mode', 'figure'),
    State('draw-line-mode', 'data'),
    prevent_initial_call=True
)
def draw_lines(line_coordinates, figure, draw_line_mode):
    if draw_line_mode == True:
         if line_coordinates and 'shapes' in line_coordinates:
            figure['layout']['shapes'] = line_coordinates['shapes']
         return figure
    else:
        return figure

if __name__ == "__main__":
    app.run_server(debug=True)