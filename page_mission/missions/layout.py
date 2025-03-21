from dash import html, dcc
import dash_bootstrap_components as dbc

def create_mission_row(mission):
    mission_id = mission.get("guid", "")
    return html.Tr([
        html.Td(mission_id),
        html.Td(mission.get("name", "")),
        html.Td([
            html.Button(
                html.I(className="fas fa-check"),
                id={"type": "addmission-btn", "index": mission_id},
                className="btn btn-success btn-sm",
                style={"marginLeft": "5px"},
            ),
            html.Button(
                html.I(className="fas fa-eye"),
                id = "viewmission-btn",
                className="btn btn-primary btn-sm",
                style={"marginLeft": "5px"},
            ),
            html.Button(
                html.I(className="fas fa-times"),
                id = "stopmission-btn",
                className="btn btn-danger btn-sm",
                style={"marginLeft": "5px"},
            ),
        ]),
    ])

def mission_queue_layout():
    return dbc.Container([
        # Hàng trên cùng: Missions & Dropdown
        dbc.Row([
            dbc.Col([
                html.H1("Missions"),
                html.P("Create and edit missions."),
            ], width=8),
            
        ], className="mb-4"),
        dbc.Row([
            dbc.Col([
                html.Label("Show missions:"),
                dcc.Dropdown(
                    id="mission-dropdown",
                    options=[
                        {"label": "Missions", "value": "missions"},
                    ],
                    value="missions",
                    clearable=False,
                    style={"width": "100%"}
                ),
                html.Button("Create / Edit groups", id="edit-groups", style={"marginTop": "10px"})
            ], width=4),
        ]),
        # Hàng dưới: Mission List và Mission Queue
        dbc.Row([
            # Cột trái: Mission List
            
            dbc.Col([
                html.H4("Mission list", className="text-primary"),
                html.Div(  # Thêm một div bọc bên ngoài bảng
                [
                    html.Table(
                        [html.Thead(html.Tr([html.Th("ID"), html.Th("Mission"), html.Th("Actions")]))] +
                        [html.Tbody(id="mission-list-container")],
                        className="table table-striped"
                    ),
                ],
                style={"maxHeight": "400px", "overflowY": "auto", "border": "1px solid #ccc"}  # Giới hạn chiều cao & thêm thanh cuộn
                )
            ], width=8),

            # Cột phải: Mission Queue
            dbc.Col([
                html.H4("Mission Queue", className="text-primary"),
                html.Div(  # Thêm một div bọc bên ngoài bảng
                [
                    html.Table(
                    [html.Thead(html.Tr([html.Th("ID"), html.Th("Mission"), html.Th("Actions")]))] +
                    [html.Tbody(id="mission-queue-container")],
                    className="table table-striped"
                    ),
                ],
                style={"maxHeight": "400px", "overflowY": "auto", "border": "1px solid #ccc"}  # Giới hạn chiều cao & thêm thanh cuộn
                ),
                
                dbc.Row([
                    dbc.Col(dbc.Button("Create", id="create-mission-btn", color="primary", className="me-1"), width="auto"),
                    dbc.Col(dbc.Button("Add", color="warning", className="me-1"), width="auto"),
                    dbc.Col(dbc.Button("Stop", color="danger", className="me-1"), width="auto"),
                ], style={"marginTop": "10px"}),
                html.Div(id="api-addmissions-response-message", style={"margin-top": "20px"}),
            ], width=4)
        ], className="mb-4"),

        dcc.Interval(id="mission-interval", interval=5000, n_intervals=0),

        dbc.Modal(
        [
            dbc.ModalHeader("Mission Configuration"),
            dbc.ModalBody([
                # Nhập tên & mô tả nhiệm vụ
                dbc.Row([
                    dbc.Col([
                        dbc.Label("Mission Name"),
                        dbc.Input(id="mission-name-input", placeholder="Enter mission name", type="text"),
                    ], width=6),
                    dbc.Col([
                        dbc.Label("Mission Description"),
                        dbc.Input(id="mission-desc-input", placeholder="Enter description", type="text"),
                    ], width=6),
                ], className="mb-3"),

                # Chọn nhóm nhiệm vụ
                dbc.Row([
                    dbc.Col([
                        dbc.Label("Select Mission Group"),
                        dcc.Dropdown(
                            id="mission-group-dropdown",
                            options=[],  # Cập nhật từ API
                            placeholder="Select a group",
                            style={"width": "100%"}
                        ),
                    ], width=6),
                ], className="mb-3"),

                html.Hr(),

                # Chọn Actions
                html.H4("Add Actions to Mission", className="text-primary"),
                dbc.Row([
                    dbc.Col([
                        dbc.Label("Action Type"),
                        dcc.Dropdown(
                            id="action-type-dropdown",
                            options=[
                                {"label": "Move", "value": "move"},
                                {"label": "Wait", "value": "wait"},
                                {"label": "Charge", "value": "charge"},
                                {"label": "Set Digital Output", "value": "set_digital_output"}
                            ],
                            placeholder="Select an action",
                            style={"width": "100%"}
                        ),
                    ], width=6),
                ], className="mb-3"),

                dbc.Row([
                    dbc.Col([
                        dbc.Label("Action Parameters (JSON format)"),
                        dbc.Input(id="action-params-input", placeholder='{"position": "some_guid", "velocity": 0.5}', type="text"),
                    ], width=12),
                ], className="mb-3"),

                # Nút thêm action
                dbc.Button("Add Action", id="add-action-btn", color="success", className="me-2"),
                html.Div(id="api-addaction-response-message", style={"margin-top": "10px"}),

                html.Hr(),

                # Danh sách Actions đã thêm
                html.H4("Configured Actions", className="text-primary"),
                html.Div(id="configured-actions-container", children=[]),
            ]),

            dbc.ModalFooter([
                dbc.Button("Finish & Send to API", id="finish-mission-btn", color="primary"),
                dbc.Button("Close", id="close-config-btn", className="ms-auto"),
                html.Div(id="api-finishmission-response-message", style={"margin-top": "10px"}),
            ]),
        ],
        id="mission-config-modal",
        is_open=False,
        size="lg",  # Kích thước lớn
        backdrop=True,
        centered=True
    )
    ],
    style={"background": "#f8f9fa", "borderRadius": "10px", "padding": "15px", "boxShadow": "2px 2px 10px rgba(0,0,0,0.1)"})
