from dash import html, dcc
import dash_bootstrap_components as dbc
import plotly.graph_objects as go

def read_map_info(file_path):
    """
    Đọc thông tin bản đồ từ tệp .info
    """
    map_info = {}
    with open(file_path, 'r') as file:
        for line in file:
            key, value = line.strip().split(': ')
            if key in ['resolution', 'origin_x', 'origin_y']:
                map_info[key] = float(value)
            else:
                map_info[key] = int(float(value))  
    return map_info

def real_to_map_coords(x, y, map_info):
    """
    Chuyển đổi tọa độ thực tế sang tọa độ bản đồ
    """
    map_x = (x - map_info['origin_x']) / map_info['resolution']
    map_y = (y - map_info['origin_y']) / map_info['resolution']
    return map_x, map_y

def create_draw_mode_layout():
    map_info = read_map_info('static/map_image.info')
    button_style = {
        "padding": "8px 16px",
        "border": "1px solid #3498db",
        "color": "#3498db",
        "background-color": "white",
        "border-radius": "5px",
        "transition": "all 0.3s ease-in-out",
        "cursor": "pointer",
    }
    button_hover_style = {
        "background-color": "#3498db",
        "color": "white",
    }
    active_button_style = {
        "padding": "8px 16px",
        "border": "1px solid #2ecc71", 
        "color": "white",
        "background-color": "#2ecc71",  
        "border-radius": "5px",
        "transition": "all 0.3s ease-in-out",
        "cursor": "pointer",
    }
    grid_size = 1  
    grid_color = "#CCCCCC"  
    axis_color = "#000000" 
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
    map_image = go.layout.Image(
        source="/static/map_image.png",  
        xref="x",
        yref="y",
        x=0,
        y=max_y, 
        sizex=max_x, 
        sizey=max_y, 
        sizing="stretch",
        layer="below",  
    )

    # # Load existing lines from JSON
    # existing_drawn_objects = load_lines_from_json() # Load as drawn objects
    # if existing_drawn_objects is None:
    #     existing_drawn_objects = []  # Treat None as an empty list


    # # Create scatter traces for existing lines and arcs
    # existing_traces = []
    # for item in existing_drawn_objects:
    #     if isinstance(item, dict):  # Check if item is a dictionary
    #         if "type" in item:  # Check if the 'type' key exists
    #             if item["type"] == "line":
    #                 existing_traces.append(
    #                     go.Scatter(
    #                         x=item["x"],
    #                         y=item["y"],
    #                         mode="lines",
    #                         line=dict(color="purple", width=2),
    #                         showlegend=False,
    #                     )
    #                 )
    #             elif item["type"] == "arc":
    #                 n_points = 50
    #                 angles = np.linspace(item["start_angle"], item["end_angle"], n_points)
    #                 x = item["center_x"] + item["radius"] * np.cos(angles)
    #                 y = item["center_y"] + item["radius"] * np.sin(angles)
    #                 existing_traces.append(
    #                     go.Scatter(
    #                         x=x,
    #                         y=y,
    #                         mode="lines",
    #                         line=dict(color="purple", width=2),
    #                         showlegend=False,
    #                     )
    #                 )
    #             else:
    #                 print(f"Warning: Unknown type: {item['type']} in item: {item}")
    #         else:
    #             print(f"Warning: 'type' key missing in item: {item}")
    #     else:
    #         print(f"Warning: Expected a dictionary, but got: {type(item)}, value: {item}")

    map_layout = go.Layout(
        xaxis=dict(showgrid=True, range=[0, max_x], fixedrange=False),
        yaxis=dict(showgrid=True, range=[0, max_y], scaleanchor="x", scaleratio=1, fixedrange=False),
        plot_bgcolor="white",
        margin=dict(l=0, r=0, t=0, b=0),
        images=[map_image],
        shapes=[],  
        dragmode="drawline"
    ) 
    # map_figure = go.Figure(data=grid_lines + axis_lines + existing_traces, layout=map_layout)
    map_figure = go.Figure(data=grid_lines + axis_lines , layout=map_layout)
    draw_mode_layout = html.Div(
        [
            html.H3("Draw Mode", className="mb-3", style={"color": "#2C3E50"}),
            html.P("This is the draw mode page. Edit and draw the map.", className="text-muted"),
            dbc.ButtonGroup(
                [
                    dbc.Button(
                        [html.I(className="fas fa-pen-nib me-2"), "Draw Line"],
                        id="draw-line-button",
                        style=button_style,
                        className="me-1",
                        n_clicks=0,
                    ),
                    dbc.Button(
                        [html.I(className="fas fa-vector-square me-2"), "Draw Arc"],
                        id="draw-arc-button",
                        style=button_style,
                        n_clicks=0,
                    ),
                ],
                className="mb-3",
            ),
             dbc.Button(
                [html.I(className="fas fa-save me-2"), "Save Lines"],
                id="save-lines-button",
                color="success",
                className="mb-3",
            ),
            dbc.Button(
                [html.I(className="fas fa-eraser me-2"), "Clear Lines"],
                id="clear-lines-button",
                color="warning",
                className="mb-3",
            ),
             dbc.Button(
                [html.I(className="fas fa-trash me-2"), "Clear Data"],
                id="clear-all-lines-button",
                color="danger",
                className="mb-3",
            ),

            dcc.Store(id='draw-line-mode', data=False), 
            dcc.Store(id='draw-arc-mode', data=False),
            dcc.Store(id='line-coordinates', data={}),  
            dcc.Store(id='arc-coordinates', data={}),  
            dcc.Store(id='draw-method', data='manual'), 
            dcc.Store(id='draw-arc-method', data='manual'),

            dcc.Graph(
                id="map-image-draw-mode",
                figure=map_figure,
                style={"width": "100%", "height": "600px", "border": "2px solid #34495E"},
                config={"scrollZoom": True, 'modeBarButtonsToRemove': ['select2d', 'lasso2d', 'drawline', 'drawrect', 'erasedata', 'drawclosedpath', 'drawcircle', 'drawrect']},  # Cho phép zoom bằng scroll, remove select2d and lasso2d and disable default drawing tool
            ),
            html.P("The map is ready for your work.", className="text-info mt-2"),
            html.Div(id='draw-mode-output'), 
            dbc.Modal(  
                [
                    dbc.ModalHeader(dbc.ModalTitle("Choose Drawing Method (Line)")),
                    dbc.ModalBody(
                        [
                            dbc.Button("Manual Draw", id="manual-draw-button", color="primary", className="me-2"),
                            dbc.Button("Enter Coordinates", id="coordinate-draw-button", color="secondary"),
                        ]
                    ),
                ],
                id="draw-method-modal",
                is_open=False,
            ),
            dbc.Modal(  
                [
                    dbc.ModalHeader(dbc.ModalTitle("Choose Drawing Method (Arc)")),
                    dbc.ModalBody(
                        [
                            dbc.Button("Manual Draw", id="manual-draw-arc-button", color="primary", className="me-2"),
                            dbc.Button("Enter Coordinates", id="coordinate-draw-arc-button", color="secondary"),
                        ]
                    ),
                ],
                id="draw-arc-method-modal",
                is_open=False,
            ),
            dbc.Modal(  
                [
                    dbc.ModalHeader(dbc.ModalTitle("Enter Coordinates (Line)")),
                    dbc.ModalBody(
                        [
                            dbc.Label("Start X:"),
                            dbc.Input(type="number", id="start-x"),
                            dbc.Label("Start Y:"),
                            dbc.Input(type="number", id="start-y"),
                            dbc.Label("End X:"),
                            dbc.Input(type="number", id="end-x"),
                            dbc.Label("End Y:"),
                            dbc.Input(type="number", id="end-y"),
                        ]
                    ),
                    dbc.ModalFooter(
                        [
                            dbc.Button("Draw", id="draw-button", color="primary"),
                            dbc.Button("Cancel", id="cancel-button"),
                        ]
                    ),
                ],
                id="coordinate-modal",
                is_open=False,
            ),
            dbc.Modal(  
                [
                    dbc.ModalHeader(dbc.ModalTitle("Enter Coordinates (Arc - 3 Points)")),
                    dbc.ModalBody(
                        [
                            dbc.Label("Point 1 X:"),
                            dbc.Input(type="number", id="point1-x"),
                            dbc.Label("Point 1 Y:"),
                            dbc.Input(type="number", id="point1-y"),
                            dbc.Label("Point 2 X:"),
                            dbc.Input(type="number", id="point2-x"),
                            dbc.Label("Point 2 Y:"),
                            dbc.Input(type="number", id="point2-y"),
                            dbc.Label("Point 3 X:"),
                            dbc.Input(type="number", id="point3-x"),
                            dbc.Label("Point 3 Y:"),
                            dbc.Input(type="number", id="point3-y"),
                        ]
                    ),
                    dbc.ModalFooter(
                        [
                            dbc.Button("Draw", id="draw-arc-button-coordinate", color="primary"),
                            dbc.Button("Cancel", id="cancel-arc-button"),
                        ]
                    ),
                ],
                id="coordinate-arc-modal",
                is_open=False,
            ),
            html.Div(id="pixel-coordinates"),
            html.Div(id="real-world-coordinates"),
            dcc.Store(id="button-style-store", data={"draw_line_button": button_style, "draw_arc_button": button_style}),  

        ],
        style={
            "padding": "20px",
            "flex": "1",
            "background": "#ECF0F1",
            "marginLeft": "250px",
            "marginTop": "50px",
            "overflowY": "auto",
        },
    )
    return draw_mode_layout