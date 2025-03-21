import dash, rospy, tf, requests
from dash import dcc, html, Input, Output, State
import dash_bootstrap_components as dbc
from components import LoginPage, ChangePasswordPage, Sidebar, StatusBar, MapSection
from utils.data import authenticate, user_credentials, update_password
from components.draw_mode import create_draw_mode_layout
from page_draw_mode.function_draw_mode import *
from components.rviz_section import create_rviz_section
from make_marker_with_json import *
from page_map.map_api import MapAPI
from page_mission.missions.layout import mission_queue_layout

login_page = LoginPage()
change_password_page = ChangePasswordPage()
sidebar = Sidebar()
status_bar = StatusBar()
map_section = MapSection()
map_api = MapAPI()

ip = '192.168.0.172'
host = 'http://' + ip + '/api/v2.0.0/'
headers = {
    'Content-Type': 'application/json',
    'Authorization': 'Basic YWRtaW46OGM2OTc2ZTViNTQxMDQxNWJkZTkwOGJkNGRlZTE1ZGZiMTY3YTljODczZmM0YmI4YTgxZjZmMmFiNDQ4YTkxOA=='
}

@callback(
    Output("app-container", "children"),
    Input("login-button", "n_clicks"),
    State("username", "value"),
    State("password", "value"),
    prevent_initial_call=True
)
def login(n_clicks, username, password):
    if authenticate(username, password):
        return html.Div(
            [
                dcc.Location(id='url', refresh=False),
                status_bar.create_status_bar(),
                sidebar.create_sidebar(),
                html.Div(id="page-content", style={"marginLeft": "250px"}),
            ],
            style={"background": "#BDC3C7", "height": "100vh", "overflow": "hidden"},
        )
    else:
        return html.Div([login_page.layout, html.Div("Login Failed", style={"color": "red"})])
    
@callback(
    [
        Output("position-modal", "is_open"),
        Output("x-input", "value"),
        Output("y-input", "value"),
        Output("z-input", "value"),
        Output("w-input", "value"),
        Output("content-area", "children"),
    ],
    [
        Input("add-positions-btn", "n_clicks"),
        Input("use-robot-btn", "n_clicks"),
        Input("add-position-btn", "n_clicks"),
        Input("cancel-btn", "n_clicks"),
    ],
    [
        State("x-input", "value"),
        State("y-input", "value"),
        State("z-input", "value"),
        State("w-input", "value"),
        State("position-modal", "is_open"),
    ],
    prevent_initial_call=True
)
def manage_position_modal(add_pos_clicks, use_robot_clicks, add_clicks, cancel_clicks,
                         x_val, y_val, z_val, w_val, is_open):
    ctx = dash.callback_context
    if not ctx.triggered:
        return no_update, no_update, no_update, no_update, no_update, no_update
    button_id = ctx.triggered[0]["prop_id"].split(".")[0]
    if button_id == "add-positions-btn":
        return True, None, None, None, None, no_update
    elif button_id == "use-robot-btn":
        try:
            listener = tf.TransformListener()
            listener.waitForTransform("/map", "/base_link", rospy.Time(0), rospy.Duration(1.0))
            (trans, rot) = listener.lookupTransform("/map", "/base_link", rospy.Time(0))
            robot_x, robot_y, _ = trans
            _, _, yaw = tf.transformations.euler_from_quaternion(rot)
            robot_z, robot_w = math.sin(yaw), math.cos(yaw)
        except (tf.Exception, tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            return True, None, None, None, None, "Không thể lấy vị trí robot"
        return True, robot_x, robot_y, robot_z, robot_w, no_update
    elif button_id == "add-position-btn":
        if x_val is not None and y_val is not None:
            save_position_to_json(x_val, y_val, z_val, w_val) 
            generate_marker_image() 
            return False, None, None, None, None, f"Đã thêm điểm P ({x_val}, {y_val})"
        return True, x_val, y_val, z_val, w_val, "Vui lòng nhập X và Y"
    elif button_id == "cancel-btn":
        return False, None, None, None, None, no_update
    return no_update, no_update, no_update, no_update, no_update, no_update

@callback(
    Output("docker-modal", "is_open", allow_duplicate=True),
    Input("add-dockers-btn", "n_clicks"),
    State("docker-modal", "is_open"),
    prevent_initial_call=True
)
def toggle_docker_modal(n_clicks, is_open):
    return not is_open 

@callback(
    [
        Output("docker-modal", "is_open"),
        Output("docker-x", "value"),
        Output("docker-y", "value"),
        Output("docker-z", "value"),
        Output("docker-w", "value"),
        Output("content-area", "children", allow_duplicate=True),
    ],
    [
        Input("add-dockers-btn", "n_clicks"),
        Input("use-robot-docker-btn", "n_clicks"),
        Input("add-docker-btn", "n_clicks"),
        Input("cancel-btn", "n_clicks"),
    ],
    [
        State("docker-x", "value"),
        State("docker-y", "value"),
        State("docker-z", "value"),
        State("docker-w", "value"),
        State("docker-modal", "is_open"),
    ],
    prevent_initial_call=True
)
def manage_docker_modal(add_pos_clicks, use_robot_clicks, add_clicks, cancel_clicks,
                         x_val, y_val, z_val, w_val, is_open):
    ctx = dash.callback_context
    if not ctx.triggered:
        return no_update, no_update, no_update, no_update, no_update, no_update
    button_id = ctx.triggered[0]["prop_id"].split(".")[0]
    if button_id == "add-dockers-btn":
        return True, None, None, None, None, no_update
    elif button_id == "use-robot-docker-btn":
        try:
            listener = tf.TransformListener()
            listener.waitForTransform("/map", "/base_link", rospy.Time(0), rospy.Duration(1.0))
            (trans, rot) = listener.lookupTransform("/map", "/base_link", rospy.Time(0))
            robot_x, robot_y, _ = trans
            _, _, yaw = tf.transformations.euler_from_quaternion(rot)
            robot_z, robot_w = math.sin(yaw), math.cos(yaw)
        except (tf.Exception, tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            return True, None, None, None, None, "Không thể lấy vị trí robot"
        return True, robot_x, robot_y, robot_z, robot_w, no_update
    elif button_id == "add-docker-btn":
        if x_val is not None and y_val is not None:
            save_docker_to_json(x_val, y_val, z_val, w_val) 
            generate_docker_image() 
            return False, None, None, None, None, f"Đã thêm điểm Docker"
        return True, x_val, y_val, z_val, w_val, "Vui lòng nhập X và Y"
    elif button_id == "cancel-btn":
        return False, None, None, None, None, no_update
    return no_update, no_update, no_update, no_update, no_update, no_update

@callback(
    [Output("pause-button", "children"), Output("pause-button", "className")],
    Input("pause-button", "n_clicks"),
    State("pause-button", "children"),
    State("pause-button", "className"),
    prevent_initial_call=True
)
def toggle_pause(n_clicks, current_icon, current_class):
    if 'fa-play' in current_icon['props']['className']:  
        return html.I(className="fas fa-pause"), "btn btn-warning btn-sm me-2" 
    return html.I(className="fas fa-play"), "btn btn-success btn-sm me-2" 

@callback(
    Output('page-content', 'children'),
    Input('url', 'pathname')
)
def display_page(pathname):
    if pathname == '/draw-mode':
        return html.Div(
                [
                    status_bar.create_status_bar(),
                    create_draw_mode_layout(),
                    html.Div(id="joystick-popup-container"),
                    html.Div(id="joystick-output", style={"margin": "20px", "fontSize": "0px"}),
                    dcc.Interval(
                        id='interval-component',
                        interval=1*1000,
                        n_intervals=0
                    )
                ]
            )
    elif pathname == '/maps':
        return html.Div(
                [
                    status_bar.create_status_bar(),
                    map_section.create_map_section(),
                    html.Div(id="joystick-popup-container"),
                    html.Div(id="joystick-output", style={"margin": "20px", "fontSize": "0px"}),
                    dcc.Interval(
                        id='interval-component',
                        interval=1*1000,
                        n_intervals=0
                    )
                ]
            )
    elif pathname == '/change-password':
        return html.Div(
                [
                    status_bar.create_status_bar(),
                    change_password_page.layout,
                    html.Div(id="joystick-popup-container"),
                    html.Div(id="joystick-output", style={"margin": "20px", "fontSize": "0px"}),
                    dcc.Interval(
                        id='interval-component',
                        interval=1*1000,
                        n_intervals=0
                    )
                ]
            )
    elif pathname == '/rviz':
        return html.Div(
                [
                    status_bar.create_status_bar(),
                    create_rviz_section(),
                    html.Div(id="joystick-popup-container"),
                    html.Div(id="joystick-output", style={"margin": "20px", "fontSize": "0px"}),
                    dcc.Interval(
                        id='interval-component',
                        interval=1*1000,
                        n_intervals=0
                    )
                ]
            )
    elif pathname == '/missions':
        return mission_queue_layout()
    elif pathname == '/map-api':
        return map_api.create_map_api()
    else:
        return html.Div(
            [
                status_bar.create_status_bar(),
                map_section.create_map_section(),
                html.Div(id="joystick-popup-container"),
                html.Div(id="joystick-output", style={"margin": "20px", "fontSize": "0px"}),
                dcc.Interval(
                    id='interval-component',
                    interval=1*1000,
                    n_intervals=0
                )
            ]
        )
    
@callback(
    Output("password-status", "children"),
    Input("update-password-button", "n_clicks"),
    State("new-password", "value"),
    State("confirm-password", "value"),
    prevent_initial_call=True
)
def update_password_callback(n_clicks, new_password, confirm_password):
    if new_password == confirm_password:
        global user_credentials
        username = list(user_credentials.keys())[0]
        if update_password(username, new_password):
            return html.Div("Password updated successfully!", style={"color": "green"})
        else:
            return html.Div("Failed to update password.", style={"color": "red"})
    else:
        return html.Div("Passwords do not match.", style={"color": "red"})
    
@callback(
    Output('map_section', 'children'),
    Input('language-dropdown', 'value')
)
def change_language(language):
    translations = {
        'en': {'title': 'Main Floor', 'map': 'Edit and draw the map', 'ready': 'El mapa está listo para tu trabajo.'},
        'vi': {'title': 'Tầng Chính', 'map': 'Edit and draw the map', 'ready': 'El mapa está listo para tu trabajo.'},
        'es': {'title': 'Piso Principal', 'map': 'Edit and draw the map', 'ready': 'El mapa está listo para tu trabajo.'},
    }
    translation = translations.get(language, translations['en'])
    return html.Div(
        [
            html.H3(translation['title'], className="mb-3", style={"color": "#2C3E50"}),
            html.P(translation['map'], className="text-muted"),
            html.Img(src="/path/to/save/map_image.png", style={"width": "100%", "border": "2px solid #34495E"}),
            html.P(translation['ready'], className="text-info mt-2"),
            html.Div(id="content-area"),
        ],
        style={
            "padding": "20px",
            "flex": "1",
            "background": "#ECF0F1",
            "marginLeft": "250px",
            "marginTop": "50px",
        },
    )

layout = html.Div(
    [
        dcc.Location(id='url', refresh=False),
        html.Div(id="app-container", children=[login_page.layout]),
        html.Div(id="joystick-output", style={"margin": "20px", "fontSize": "0px"}),
        dcc.Interval(
            id='interval-component',
            interval=1000,
            n_intervals=0
        )
    ]
)

@callback(
    Output("sidebar-nav", "children"),
    Input('url', 'pathname'),
    prevent_initial_call=True
)
def update_active_link(pathname):
    nav_links = [
        {"href": "/", "id": "index-link", "label": "Home"},
        {"href": "/draw-mode", "id": "draw-mode-link", "label": "Draw Mode"},
        {"href": "/rviz", "id": "rviz-link", "label": "RViz"},
        {"href": "/missions", "id": "mission-link", "label": "Missions"},
        {"href": "/map-api", "id": "map-api-link", "label": "Maps"},
        {"href": "#", "id": "io-modules-link", "label": "I/O Modules"},
        {"href": "#", "id": "users-link", "label": "Users"},
        {"href": "#", "id": "user-groups-link", "label": "User Groups"},
        {"href": "#", "id": "paths-link", "label": "Paths"},
        {"href": "#", "id": "path-guides-link", "label": "Path Guides"},
        {"href": "#", "id": "marker-types-link", "label": "Marker Types"},
        {"href": "#", "id": "footprints-link", "label": "Footprints"},
        {"href": "/change-password", "id": "change-password-link", "label": "Change Password"},
    ]

    updated_links = []
    for link in nav_links:
        active = pathname == link["href"]
        updated_links.append(
            dbc.NavLink(
                link["label"],
                href=link["href"],
                id=link["id"],
                className="text-white",
                active=active
            )
        )
    return updated_links

@callback(
    Output("battery-percen", "children"),
    Input("interval-component", "n_intervals")
)
def update_battery_status(n):

    try:
        response = requests.get(host + '/status', headers=headers)  
        data = response.json()
        battery_level = data.get("battery_percentage", "--") 
        if isinstance(battery_level, float):
            battery_level = round(battery_level)
        return f"{battery_level}%"
    except Exception as e:
        print(f"⚠️ Lỗi khi gọi API: {e}")  # Log lỗi
        return "--%"