from dash import dcc, html
import dash_bootstrap_components as dbc
from utils.data import LANGUAGES
from dash_iconify import DashIconify

class StatusBar:
    def create_status_bar(self):
        return html.Div(
            [
                dbc.Row(
                    [
                        dbc.Col(
                            [
                                html.Span(id="mission-status", className="badge bg-info text-dark me-2", children="Listening to Mission"),
                                html.Button(
                                    html.I(className="fas fa-play"),  
                                    id="pause-button",
                                    className="btn btn-success btn-sm me-2"
                                ),
                                html.Span("ALL OK", className="badge bg-success me-2"),
                                html.Span(id="linear-speed-display", className="badge bg-info text-dark me-2", children="Linear Speed: 0.5"),
                                html.Span(id="angular-speed-display", className="badge bg-info text-dark me-2", children="Angular Speed: 0.3"),
                            ],
                            width="auto",
                        ),
                        dbc.Col(
                            dcc.Dropdown(
                                id="language-dropdown",
                                options=LANGUAGES,
                                value="en",  
                                clearable=False,
                            ),
                            width="auto",
                            className="me-2"
                        ),
                         dbc.Col(
                            [
                                html.Div("Linear Speed:", style={"marginRight": "5px", "color": "white"}), 
                                dbc.Input(id="linear-speed-input", type="number", placeholder="Linear Speed", value=0.5, step=0.1, style={"width": "100px", "marginRight": "10px"}),
                                html.Div("Angular Speed:", style={"marginLeft": "10px","marginRight": "5px", "color": "white"}),
                                dbc.Input(id="angular-speed-input", type="number", placeholder="Angular Speed", value=0.3, step=0.1, style={"width": "100px"}),
                            ],
                            width="auto",
                            className="me-2",
                            style={"display": "flex", "alignItems": "center"}
                        ),
                        dbc.Col(
                            dbc.Button("Open Teleoperation", id="open-joystick-btn", color="primary", size="sm"),
                            width="auto",
                            className="text-end",
                        ),
                        dbc.Col(
                        [
                            DashIconify(icon="mdi:battery-90", width=30, height=30, style={"color": "white", "marginRight": "5px"}),
                            html.Span(id="battery-percen", children="--%", style={"color": "white", "fontSize": "16px"})
                        ],
                        width="auto",
                        className="text-end"
                        ),

                    ],
                    align="center",
                    className="g-0",
                    style={"width": "100%"},
                )
            ],
            style={
                "background": "#34495E",
                "color": "white",
                "padding": "10px",
                "position": "fixed",
                "top": 0,
                "left": "250px",
                "width": "100%",
                "zIndex": 1000,
            },
        )