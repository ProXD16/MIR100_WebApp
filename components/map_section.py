import rospy
from geometry_msgs.msg import Twist
from dash import Dash, dcc, html, Output, Input, State
from dash.dependencies import ClientsideFunction
import dash_bootstrap_components as dbc
import dash_daq as daq
import math

app = Dash(__name__, external_stylesheets=[dbc.themes.BOOTSTRAP])

button_style = {
    "padding": "10px 20px",
    "border": "none",
    "color": "#FFFFFF",
    "backgroundColor": "#5DADE2",
    "border-radius": "8px",
    "transition": "background-color 0.3s ease",
    "cursor": "pointer",
    "box-shadow": "0 2px 5px rgba(0, 0, 0, 0.2)",
}
button_primary_style = {
    "width": "20%",
    "padding": "10px 20px",
    "border": "none",
    "color": "#FFFFFF",
    "backgroundColor": "#5DADE2",
    "border-radius": "8px",
    "transition": "background-color 0.3s ease",
    "cursor": "pointer",
    "box-shadow": "0 2px 5px rgba(0, 0, 0, 0.2)",
}
button_secondary_style = {
    "width": "100%",
    "padding": "10px 20px",
    "border": "none",
    "color": "#FFFFFF",
    "backgroundColor": "#5DADE2",
    "border-radius": "8px",
    "transition": "background-color 0.3s ease",
    "cursor": "pointer",
    "box-shadow": "0 2px 5px rgba(0, 0, 0, 0.2)",
}
button_hover_style = {
    "border-radius": "8px",
    "border": "none",
    "cursor": "pointer",
    "padding": "10px 20px",
    "width": "100%",
    "background-color": "#5DADE2",
    "color": "#FFFFFF",
    "box-shadow": "0 2px 5px rgba(0, 0, 0, 0.2)",
}
button_default_manual_style = {
    "border-radius": "8px",
    "border": "none",
    "cursor": "pointer",
    "padding": "10px 20px",
    "width": "100%",
    "background-color": "#D3D3D3",
    "color": "#FFFFFF",
    "box-shadow": "0 2px 5px rgba(0, 0, 0, 0.2)",
}
button_active_manual_style = {
    "border-radius": "8px",
    "border": "none",
    "cursor": "pointer",
    "padding": "10px 20px",
    "width": "100%",
    "background-color": "#5DADE2",
    "color": "#FFFFFF",
    "box-shadow": "0 2px 5px rgba(0, 0, 0, 0.2)",
}

def create_charging_status_component():
    return html.Div(
        id='charging-status-container',
        style={
            "border": "1px solid #ccc",
            "padding": "10px",
            "margin": "10px",
            "backgroundColor": "#f9f9f9",
            "borderRadius": "5px",
            "textAlign": "center",
            "width": "100%",
        },
        children=[
            html.Div(
                id='status-message',
                children=[
                    html.I(id='status-icon', className="fas fa-spinner fa-spin me-2"),
                    html.Span(id='status-text', children="Charging... Waiting for new mission...")
                ]
            ),
            html.Div(
                id='status-details',
                style={"marginTop": "10px"},
                children=[
                    html.Span(id='status-action'),
                    html.Span(":"),
                    html.Span(id='status-position'),
                    html.Div(
                        style={'display': 'flex', 'justify-content': 'space-between'},
                        children=[
                            dbc.Badge(id='status-badge', color="success", className="me-1"),
                            html.Button(
                                "X",
                                id='cancel-button',
                                style={
                                    "backgroundColor": "#dc3545",
                                    "color": "white",
                                    "border": "none",
                                    "padding": "5px 10px",
                                    "borderRadius": "5px",
                                    "cursor": "pointer",
                                }
                            ),
                        ]
                    )
                ]
            )
        ]
    )

def create_plc_info_component():
    return html.Div(
        style={
            "border": "1px solid #ccc",
            "padding": "10px",
            "margin": "10px",
            "backgroundColor": "#e1f5fe",
            "borderRadius": "5px",
            "textAlign": "center",
            "width": "100%",
        },
        children=[
            html.Div("PLC register 160"),
            html.Div("..."),
            html.Div("Current value: 1", style={"marginTop": "5px"}),
        ],
    )

def create_manual_control_component():
    return html.Div(
        style={
            "border": "1px solid #ccc",
            "padding": "10px",
            "margin": "10px",
            "backgroundColor": "#f9f9f9",
            "borderRadius": "5px",
            "textAlign": "center",
            "width": "100%",
            "height": "437px",
        },
        children=[
            html.Button(
                "MANUAL CONTROL",
                id="manual-control",
                className="btn btn-secondary",
                style=button_default_manual_style
            ),
            html.Div(
                id="joystick-container",
                style={"display": "none"},
                children=[
                    daq.Joystick(
                        id='joystick',
                        label="Control Joystick",
                        size=150,
                        style={'margin': 'auto'}
                    ),
                    html.Label("Speed Scale:", style={'marginTop': '10px'}),
                    html.Div([
                        dcc.Slider(
                            id='speed-scale',
                            min=0.1,
                            max=1.0,
                            step=0.1,
                            value=0.5,
                            marks={i/10: str(i/10) for i in range(1, 11)},
                            updatemode='drag'
                        )
                    ], style={'width': '80%', 'margin': '10px auto'}),
                    daq.BooleanSwitch(
                        id='emergency-stop',
                        on=False,
                        label="Emergency Stop",
                        labelPosition="top",
                        color="#ff0000",
                        style={'margin': '10px auto'}
                    ),
                    html.Div(
                        id='joystick-output',
                        style={
                            'marginTop': '10px',
                            'fontSize': '14px',
                            'color': '#34495e',
                            'textAlign': 'center'
                        }
                    ),
                    dcc.Store(id='joystick-data', data={'angle': 0, 'force': 0}),
                    dcc.Interval(id='interval-joystick', interval=50, n_intervals=0, disabled=True),
                ]
            ),
        ],
    )

class MapSection:
    def create_map_section(self):
        return html.Div(
            style={'backgroundColor': '#FFFFFF', 'height': '100vh'},
            children=[
                html.Div(
                    [
                        html.H3("HOME", className="mb-3", style={"color": "#34495E", "fontWeight": "bold"}),
                        dbc.Row(
                            [
                                dbc.Col(
                                    [
                                        html.Div(
                                            [
                                                html.Button("Add Markers", id="add-markers-btn",
                                                            className="fas fa-map-marker-alt me-2", style=button_primary_style),
                                                dbc.Popover(
                                                    [
                                                        dbc.PopoverHeader("Select an option"),
                                                        dbc.PopoverBody(
                                                            [
                                                                html.Button("Add Positions", id="add-positions-btn",
                                                                            className="fas fa-compass me-2",
                                                                            style=button_hover_style),
                                                                html.Button("Add Dockers", id="add-dockers-btn",
                                                                            className="fas fa-bolt me-2",
                                                                            style=button_hover_style),
                                                            ]
                                                        ),
                                                    ],
                                                    id="popover",
                                                    target="add-markers-btn",
                                                    trigger="click",
                                                    placement="left",
                                                ),
                                                html.Button("Delete Marker", id="delete-marker-btn",
                                                            className="fas fa-trash me-2", style=button_primary_style),
                                                dbc.Modal(
                                                    [
                                                        dbc.ModalHeader(dbc.ModalTitle("Delete Marker")),
                                                        dbc.ModalBody(
                                                            [
                                                                dcc.Dropdown(id="marker-dropdown",
                                                                             placeholder="Choose Marker to Delete"),
                                                            ]
                                                        ),
                                                        dbc.ModalFooter(
                                                            [
                                                                html.Button("Delete", id="confirm-delete-btn",
                                                                            className="btn btn-danger me-2",
                                                                            style=button_style),
                                                                html.Button("Cancel", id="close-delete-modal",
                                                                            className="btn btn-secondary",
                                                                            style=button_style),
                                                            ]
                                                        ),
                                                    ],
                                                    id="delete-marker-modal",
                                                    is_open=False,
                                                ),
                                                html.Button("Add Mission", id="add-mission-marker-btn",
                                                            className="fas fa-bullseye me-2", style=button_primary_style),
                                                dbc.Modal(
                                                    [
                                                        dbc.ModalHeader(dbc.ModalTitle("Add Mission From Markers")),
                                                        dbc.ModalBody(
                                                            [
                                                                dcc.Dropdown(id="mission-marker-dropdown",
                                                                             placeholder="Choose Marker to Add Mission"),
                                                            ]
                                                        ),
                                                        dbc.ModalFooter(
                                                            [
                                                                html.Button("Append", id="append-mission-btn",
                                                                            className="btn btn-success me-2",
                                                                            style=button_style),
                                                                html.Button("Clear and Append", id="clear-and-append-btn",
                                                                            className="btn btn-danger me-2",
                                                                            style=button_style),
                                                                html.Button("Cancel", id="close-delete-mission-modal",
                                                                            className="btn btn-secondary",
                                                                            style=button_style),
                                                            ]
                                                        ),
                                                    ],
                                                    id="add-mission-marker-modal",
                                                    is_open=False,
                                                ),
                                            ],
                                            className="mb-3"
                                        ),

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
                                                                            dbc.Input(type="number", id="x-input",
                                                                                      placeholder="Enter X coordinate"),
                                                                            width=10,
                                                                        ),
                                                                    ],
                                                                    className="mb-3",
                                                                ),
                                                                dbc.Row(
                                                                    [
                                                                        dbc.Label("Y:", width=2),
                                                                        dbc.Col(
                                                                            dbc.Input(type="number", id="y-input",
                                                                                      placeholder="Enter Y coordinate"),
                                                                            width=10,
                                                                        ),
                                                                    ],
                                                                    className="mb-3",
                                                                ),
                                                                dbc.Row(
                                                                    [
                                                                        dbc.Label("Z:", width=2),
                                                                        dbc.Col(
                                                                            dbc.Input(type="number", id="z-input",
                                                                                      placeholder="Enter Z coordinate"),
                                                                            width=10,
                                                                        ),
                                                                    ],
                                                                    className="mb-3",
                                                                ),
                                                                dbc.Row(
                                                                    [
                                                                        dbc.Label("W:", width=2),
                                                                        dbc.Col(
                                                                            dbc.Input(type="number", id="w-input",
                                                                                      placeholder="Enter W coordinate"),
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
                                                        html.Button("Use Robot Position", id="use-robot-btn",
                                                                    className="btn btn-secondary me-2",
                                                                    style=button_style),
                                                        html.Button("Add Position", id="add-position-btn",
                                                                    className="btn btn-primary me-2",
                                                                    style=button_style),
                                                        html.Button("Cancel", id="cancel-btn", className="btn btn-danger",
                                                                    style=button_style),
                                                    ]
                                                ),
                                            ],
                                            id="position-modal",
                                            is_open=False,
                                        ),

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
                                                                            dbc.Input(type="number", id="docker-x",
                                                                                      placeholder="Enter X coordinate"),
                                                                            width=10,
                                                                        ),
                                                                    ],
                                                                    className="mb-3",
                                                                ),
                                                                dbc.Row(
                                                                    [
                                                                        dbc.Label("Y:", width=2),
                                                                        dbc.Col(
                                                                            dbc.Input(type="number", id="docker-y",
                                                                                      placeholder="Enter Y coordinate"),
                                                                            width=10,
                                                                        ),
                                                                    ],
                                                                    className="mb-3",
                                                                ),
                                                                dbc.Row(
                                                                    [
                                                                        dbc.Label("Z:", width=2),
                                                                        dbc.Col(
                                                                            dbc.Input(type="number", id="docker-z",
                                                                                      placeholder="Enter Z coordinate"),
                                                                            width=10,
                                                                        ),
                                                                    ],
                                                                    className="mb-3",
                                                                ),
                                                                dbc.Row(
                                                                    [
                                                                        dbc.Label("W:", width=2),
                                                                        dbc.Col(
                                                                            dbc.Input(type="number", id="docker-w",
                                                                                      placeholder="Enter W coordinate"),
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
                                                        html.Button("Use Robot Position", id="use-robot-docker-btn",
                                                                    className="btn btn-secondary me-2",
                                                                    style=button_style),
                                                        html.Button("Add Docker", id="add-docker-btn",
                                                                    className="btn btn-primary me-2",
                                                                    style=button_style),
                                                        html.Button("Cancel", id="cancel-btn", className="btn btn-danger",
                                                                    style=button_style),
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
                                                        "width": "100%",
                                                        "height": "600px",
                                                        "border": "5px solid #34495E",
                                                        "borderRadius": "10px",
                                                        "object-fit": "contain",
                                                        "position": "relative",
                                                        "z-index": "1",
                                                    },
                                                ),
                                                html.Img(
                                                    id="lidar-f-image",
                                                    src="/static/f_scan_image.png",
                                                    style={
                                                        "width": "100%",
                                                        "height": "600px",
                                                        "border": "5px solid #34495E",
                                                        "borderRadius": "10px",
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
                                                        "width": "100%",
                                                        "height": "600px",
                                                        "border": "5px solid #34495E",
                                                        "borderRadius": "10px",
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
                                                        "width": "100%",
                                                        "height": "600px",
                                                        "border": "5px solid #34495E",
                                                        "borderRadius": "10px",
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
                                                        "width": "100%",
                                                        "height": "600px",
                                                        "border": "5px solid #34495E",
                                                        "borderRadius": "10px",
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
                                                        "width": "100%",
                                                        "height": "600px",
                                                        "border": "5px solid #34495E",
                                                        "borderRadius": "10px",
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
                                                        "width": "100%",
                                                        "height": "600px",
                                                        "border": "5px solid #34495E",
                                                        "borderRadius": "10px",
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
                                                        "width": "100%",
                                                        "height": "600px",
                                                        "border": "5px solid #34495E",
                                                        "borderRadius": "10px",
                                                        "object-fit": "contain",
                                                        "position": "absolute",
                                                        "top": "0",
                                                        "left": "0",
                                                        "z-index": "5",
                                                    },
                                                ),
                                                html.Img(
                                                    id="costmap-image",
                                                    src="/static/cost_map_image.png",
                                                    style={
                                                        "width": "100%",
                                                        "height": "600px",
                                                        "border": "5px solid #34495E",
                                                        "borderRadius": "10px",
                                                        "object-fit": "contain",
                                                        "position": "absolute",
                                                        "top": "0",
                                                        "left": "0",
                                                        "z-index": "6",
                                                    },
                                                ),
                                                html.Img(
                                                    id="global-costmap-image",
                                                    src="/static/global_costmap_image.png",
                                                    style={
                                                        "width": "100%",
                                                        "height": "600px",
                                                        "border": "5px solid #34495E",
                                                        "borderRadius": "10px",
                                                        "object-fit": "contain",
                                                        "position": "absolute",
                                                        "top": "0",
                                                        "left": "0",
                                                        "z-index": "6",
                                                    },
                                                ),
                                            ],
                                            style={"position": "relative"}
                                        ),
                                        create_charging_status_component(),
                                    ],
                                    width=8,
                                ),

                                dbc.Col(
                                    [
                                        html.Div(
                                            id="pause-button-1",
                                            style={
                                                "width": "100%",
                                                "height": "100px",
                                                "backgroundColor": "#003049",
                                                "color": "orange",
                                                "textAlign": "center",
                                                "lineHeight": "100px",
                                                "fontSize": "4em",
                                                "borderRadius": "10px",
                                                "marginBottom": "10px",
                                                "marginTop": "10px"
                                            },
                                            children=["||"],
                                        ),
                                        create_plc_info_component(),
                                        create_plc_info_component(),
                                        create_manual_control_component(),
                                    ],
                                    width=4,
                                ),
                            ]
                        ),
                        html.Div(id="content-area"),
                        dcc.Interval(
                            id='interval-component',
                            interval=1 * 1000,
                            n_intervals=0
                        )
                    ],
                    style={
                        "padding": "20px",
                        "flex": "1",
                        "marginLeft": "50px",
                        "marginTop": "50px",
                    },
                )
            ]
        )
    