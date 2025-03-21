from dash import Dash, dcc, html
import dash_bootstrap_components as dbc

app = Dash(__name__, external_stylesheets=[dbc.themes.BOOTSTRAP])

class MapSection:
    def create_map_section(self):
        return html.Div(
            [
                html.H3("HOME", className="mb-3", style={"color": "#2C3E50"}),
                dbc.Row(
                    [
                        dbc.Col(
                            html.Div(
                                [
                                    html.Button("Add Markers", id="add-markers-btn", className="btn btn-primary me-2"),
                                    # Popover cho Add Markers
                                    dbc.Popover(
                                        [
                                            dbc.PopoverHeader("Select an option"),
                                            dbc.PopoverBody(
                                                [
                                                    html.Button("Add Positions", id="add-positions-btn", 
                                                              className="btn btn-outline-primary w-100 mb-2"),
                                                    html.Button("Add Dockers", id="add-dockers-btn", 
                                                              className="btn btn-outline-primary w-100"),
                                                ]
                                            ),
                                        ],
                                        id="popover",
                                        target="add-markers-btn",
                                        trigger="click",
                                        placement="left",
                                    ),
                                    html.Button("Delete Marker", id="delete-marker-btn", className="btn btn-primary me-2"),
                                    dbc.Modal(
                                        [
                                            dbc.ModalHeader(dbc.ModalTitle("Delete Marker")),
                                            dbc.ModalBody(
                                                [
                                                    dcc.Dropdown(id="marker-dropdown", placeholder="Choose Marker to Delete"),
                                                ]
                                            ),
                                            dbc.ModalFooter(
                                                [
                                                    html.Button("Delete", id="confirm-delete-btn", className="btn btn-danger me-2"),
                                                    html.Button("Cancel", id="close-delete-modal", className="btn btn-secondary"),
                                                ]
                                            ),
                                        ],
                                        id="delete-marker-modal",
                                        is_open=False,
                                    ),
                                    html.Button("Add Mission", id="add-mission-marker-btn", className="btn btn-primary me-2"),
                                    dbc.Modal(
                                        [
                                            dbc.ModalHeader(dbc.ModalTitle("Add Mission From Markers")),
                                            dbc.ModalBody(
                                                [
                                                    dcc.Dropdown(id="mission-marker-dropdown", placeholder="Choose Marker to Add Mission"),
                                                ]
                                            ),
                                            dbc.ModalFooter(
                                                [
                                                    html.Button("Append", id="append-mission-btn", className="btn btn-success me-2"),
                                                    html.Button("Clear and Append", id="clear-and-append-btn", className="btn btn-danger me-2"),
                                                    html.Button("Cancel", id="close-delete-mission-modal", className="btn btn-secondary"),
                                                ]
                                            ),
                                        ],
                                        id="add-mission-marker-modal",
                                        is_open=False,
                                    ),
                                ],
                                className="mb-3"
                            ),
                            width=12,
                        ),
                    ]
                ),
                # Modal cho Add Position
                dbc.Modal(
                    [
                        dbc.ModalHeader(dbc.ModalTitle("Add Position")),
                        dbc.ModalBody(
                            [
                                dbc.Form(
                                    [
                                        dbc.Row(
                                            [
                                                dbc.Label("X:", width=2),
                                                dbc.Col(
                                                    dbc.Input(type="number", id="x-input", placeholder="Enter X coordinate"),
                                                    width=10,
                                                ),
                                            ],
                                            className="mb-3",
                                        ),
                                        dbc.Row(
                                            [
                                                dbc.Label("Y:", width=2),
                                                dbc.Col(
                                                    dbc.Input(type="number", id="y-input", placeholder="Enter Y coordinate"),
                                                    width=10,
                                                ),
                                            ],
                                            className="mb-3",
                                        ),
                                        dbc.Row(
                                            [
                                                dbc.Label("Z:", width=2),
                                                dbc.Col(
                                                    dbc.Input(type="number", id="z-input", placeholder="Enter Z coordinate"),
                                                    width=10,
                                                ),
                                            ],
                                            className="mb-3",
                                        ),
                                        dbc.Row(
                                            [
                                                dbc.Label("W:", width=2),
                                                dbc.Col(
                                                    dbc.Input(type="number", id="w-input", placeholder="Enter W coordinate"),
                                                    width=10,
                                                ),
                                            ],
                                            className="mb-3",
                                        ),
                                    ]
                                ),
                            ]
                        ),
                        dbc.ModalFooter(
                            [
                                html.Button("Use Robot Position", id="use-robot-btn", className="btn btn-secondary me-2"),
                                html.Button("Add Position", id="add-position-btn", className="btn btn-primary me-2"),
                                html.Button("Cancel", id="cancel-btn", className="btn btn-danger"),
                            ]
                        ),
                    ],
                    id="position-modal",
                    is_open=False,
                ),
                # Modal cho Add Docker
                dbc.Modal(
                    [
                        dbc.ModalHeader(dbc.ModalTitle("Add Docker")),
                        dbc.ModalBody(
                            [
                                dbc.Form(
                                    [
                                        dbc.Row(
                                            [
                                                dbc.Label("X:", width=2),
                                                dbc.Col(
                                                    dbc.Input(type="number", id="docker-x", placeholder="Enter X coordinate"),
                                                    width=10,
                                                ),
                                            ],
                                            className="mb-3",
                                        ),
                                        dbc.Row(
                                            [
                                                dbc.Label("Y:", width=2),
                                                dbc.Col(
                                                    dbc.Input(type="number", id="docker-y", placeholder="Enter Y coordinate"),
                                                    width=10,
                                                ),
                                            ],
                                            className="mb-3",
                                        ),
                                        dbc.Row(
                                            [
                                                dbc.Label("Z:", width=2),
                                                dbc.Col(
                                                    dbc.Input(type="number", id="docker-z", placeholder="Enter Z coordinate"),
                                                    width=10,
                                                ),
                                            ],
                                            className="mb-3",
                                        ),
                                        dbc.Row(
                                            [
                                                dbc.Label("W:", width=2),
                                                dbc.Col(
                                                    dbc.Input(type="number", id="docker-w", placeholder="Enter W coordinate"),
                                                    width=10,
                                                ),
                                            ],
                                            className="mb-3",
                                        ),
                                    ]
                                ),
                            ]
                        ),
                        dbc.ModalFooter(
                            [
                                html.Button("Use Robot Position", id="use-robot-docker-btn", className="btn btn-secondary me-2"),
                                html.Button("Add Docker", id="add-docker-btn", className="btn btn-primary me-2"),
                                html.Button("Cancel", id="cancel-btn", className="btn btn-danger"),
                            ]
                        ),
                    ],
                    id="docker-modal",
                    is_open=False,
                ),
                html.Div(
                    [
                        html.Img(
                            id="map-image",
                            src="/static/map_image.png",
                            style={
                                "width": "800px", 
                                "height": "600px",
                                "border": "2px solid #34495E",
                                "object-fit": "contain", 
                                "position": "absolute", 
                                "z-index": "1", 
                            },
                        ),
                        html.Img(
                            id="lidar-f-image",
                            src="/static/f_scan_image.png",
                            style={
                                "width": "800px",  
                                "height": "600px",
                                "border": "2px solid #34495E",
                                "object-fit": "contain",
                                "position": "absolute", 
                                "top": "0", 
                                "left": "0", 
                                "z-index": "2", 
                            },
                        ),
                        html.Img(
                            id="lidar-b-image",
                            src="/static/b_scan_image.png",
                            style={
                                "width": "800px", 
                                "height": "600px",
                                "border": "2px solid #34495E",
                                "object-fit": "contain",
                                "position": "absolute",  
                                "top": "0",  
                                "left": "0",
                                "z-index": "2", 
                            },
                        ),
                        html.Img(
                            id="path-image",
                            src="/static/path_image.png",
                            style={
                                "width": "800px",  
                                "height": "600px",
                                "border": "2px solid #34495E",
                                "object-fit": "contain",
                                "position": "absolute", 
                                "top": "0",  
                                "left": "0",  
                                "z-index": "3", 
                            },
                        ),
                        html.Img(
                            id="robot-image",
                            src="/static/robot_image.png",
                            style={
                                "width": "800px", 
                                "height": "600px",
                                "border": "2px solid #34495E",
                                "object-fit": "contain",
                                "position": "absolute",  
                                "top": "0", 
                                "left": "0",  
                                "z-index": "7",  
                            },
                        ),
                        html.Img(
                            id="lines-image",
                            src="/static/line_image.png",
                            style={
                                "width": "800px", 
                                "height": "600px",
                                "border": "2px solid #34495E",
                                "object-fit": "contain",
                                "position": "absolute",  
                                "top": "0", 
                                "left": "0",  
                                "z-index": "6",  
                            },
                        ),
                        html.Img(
                            id="markers-image",
                            src="/static/all_markers.png",
                            style={
                                "width": "800px", 
                                "height": "600px",
                                "border": "2px solid #34495E",
                                "object-fit": "contain",
                                "position": "absolute",  
                                "top": "0", 
                                "left": "0",  
                                "z-index": "4",  
                            },
                        ),
                        html.Img(
                            id="dockers-image",
                            src="/static/all_dockers.png",
                            style={
                                "width": "800px", 
                                "height": "600px",
                                "border": "2px solid #34495E",
                                "object-fit": "contain",
                                "position": "absolute",  
                                "top": "0", 
                                "left": "0",  
                                "z-index": "5",  
                            },
                        ),
                    ],
                    style={
                        "position": "relative", 
                        "width": "800px", 
                        "height": "600px",
                    },
                ),
                html.P("The map is ready for your work.", className="text-info mt-2"),
                html.Div(id="content-area"),  
                dcc.Interval(
                    id='interval-component',
                    interval=1*1000,  
                    n_intervals=0
                )
            ],
            style={
                "padding": "20px",
                "flex": "1",
                "background": "#ECF0F1",
                "marginLeft": "250px",
                "marginTop": "50px",
            },
        )