# app.py
import dash, time, rospy, random, os, tf, json
from dash import dcc, html, Input, Output, State
import dash_bootstrap_components as dbc
from components import LoginPage, ChangePasswordPage, Sidebar, StatusBar, MapSection, RVizSection
from utils.data import authenticate, user_credentials, update_password
from geometry_msgs.msg import Twist
from components.draw_mode import create_draw_mode_layout
from function_draw_mode.draw_line_mode_callbacks import *
from function_draw_mode.draw_arc_mode_callbacks import *
from function_teleop_control.teleop_control import TeleopControl
from dash import callback_context
from components.rviz_section import create_rviz_section
from dash.exceptions import PreventUpdate
import numpy as np
from work_with_json.process_with_json import *
from work_with_json.generate_image_from_json import *
from page_mission.missions.layout import mission_queue_layout
from page_mission.missions.callbacks import register_callbacks
import requests
from map_api_f.map_api import MapAPI
# from callbacks.image_components_callbacks import *
# from callbacks.navigation_callbacks import *
# from callbacks.mission_callbacks import *
# from callbacks.ui_callbacks import *

stop_requested = False 
ip = '192.168.0.172'
host = 'http://' + ip + '/api/v2.0.0/'
headers = {
    'Content-Type': 'application/json',
    'Authorization': 'Basic YWRtaW46OGM2OTc2ZTViNTQxMDQxNWJkZTkwOGJkNGRlZTE1ZGZiMTY3YTljODczZmM0YmI4YTgxZjZmMmFiNDQ4YTkxOA=='
}

app = dash.Dash(
    __name__,
    external_stylesheets=[
        dbc.themes.BOOTSTRAP,
        "https://cdnjs.cloudflare.com/ajax/libs/font-awesome/6.0.0/css/all.min.css",
    ],
    suppress_callback_exceptions=True,
    external_scripts=["assets/script.js"]
)

if not rospy.core.is_initialized():
    try:
        rospy.init_node('dash_app', anonymous=True)
        print("ROS node 'dash_app' initialized successfully.")
    except rospy.exceptions.ROSException as e:
        print(f"Error initializing ROS node: {e}")

if rospy.core.is_initialized():
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
else:
    pub = None
    print("ROS not properly initialized, publisher not created.")

linear_speed = 0.5
angular_speed = 0.3

if pub:
    joystick_control = TeleopControl(pub, linear_speed, angular_speed)
    joystick_control.stop()  

else:
    class DummyTeleopControl:
        def __init__(self):
            pass
        def create_joystick_popup(self):
            return html.Div("ROS not available, TeleopControl is disabled.")

    joystick_control = DummyTeleopControl()

login_page = LoginPage()
change_password_page = ChangePasswordPage()
sidebar = Sidebar()
status_bar = StatusBar()
map_section = MapSection()
map_api = MapAPI()
map_api.register_callbacks(app)

app.layout = html.Div(
    [
        dcc.Location(id='url', refresh=False),
        html.Div(id="app-container", children=[login_page.layout]),
        html.Div(id="joystick-output", style={"margin": "20px", "fontSize": "0px"}),
    ]
)

@app.callback(
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

@app.callback(
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

@app.callback(
    [Output("delete-marker-modal", "is_open"),
     Output("marker-dropdown", "options")],
    Input("delete-marker-btn", "n_clicks"),
    prevent_initial_call=True
)
def show_marker_list(n_clicks):
    markers = load_markers()
    if not markers:
        return True, [] 
    options = [{"label": f"Marker {m['id']}", "value": m["id"]} for m in markers]
    return True, options  

@app.callback(
    Output("delete-marker-modal", "is_open", allow_duplicate=True),
    Input("confirm-delete-btn", "n_clicks"),
    State("marker-dropdown", "value"),
    prevent_initial_call=True
)
def delete_marker(n_clicks, selected_marker):
    if selected_marker is None:
        return no_update  
    markers = load_markers()
    markers = [m for m in markers if m["id"] != selected_marker]
    with open(JSON_FILE_PATH, "w") as file:
        json.dump(markers, file, indent=4)
    generate_marker_image()  
    return False  

@app.callback(
    [Output("add-mission-marker-modal", "is_open", allow_duplicate=True),
     Output("mission-marker-dropdown", "options")],
     Input("add-mission-marker-btn", "n_clicks"),
     prevent_initial_call = True
)
def show_mission_marker_list(n_clicks):
    markers = load_markers()
    if not markers:
        return True, []
    options = [{"label": f"Marker {m['id']}", "value": m["id"]} for m in markers]
    return True, options 

@app.callback(
    Output("add-mission-marker-modal", "is_open", allow_duplicate=True),
    Input("append-mission-btn", "n_clicks"),
    State("mission-marker-dropdown", "value"),
    prevent_initial_call=True
)
def append_marker_to_mission(n_clicks, selected_marker):
    if selected_marker is None:
        return no_update  
    save_marker_to_json(selected_marker, clear=False)  
    return False 

@app.callback(
    Output("add-mission-marker-modal", "is_open", allow_duplicate=True),
    Input("clear-and-append-btn", "n_clicks"),
    State("mission-marker-dropdown", "value"),
    prevent_initial_call=True
)
def clear_and_append_marker(n_clicks, selected_marker):
    if selected_marker is None:
        return no_update  
    
    save_marker_to_json(selected_marker, clear=True)  
    return False 

@app.callback(
    Output("docker-modal", "is_open", allow_duplicate=True),
    Input("add-dockers-btn", "n_clicks"),
    State("docker-modal", "is_open"),
    prevent_initial_call=True
)
def toggle_docker_modal(n_clicks, is_open):
    return not is_open 

@app.callback(
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

@app.callback(
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

@app.callback(
    [Output("pause-button", "children", allow_duplicate=True),  
     Output("pause-button", "className", allow_duplicate=True)],  
    Input("pause-button", "n_clicks"),
    State("pause-button", "children"),
    prevent_initial_call=True
)
def toggle_pause(n_clicks, current_icon):
    global stop_requested  
    stop_requested = False 
    rviz_section = RVizSection()
    if 'fa-play' in current_icon['props']['className']:  
        mission_list = load_mission_data()
        if not mission_list:
            print("Không có nhiệm vụ nào trong marker_mission.json!")
            return html.I(className="fas fa-play"), "btn btn-warning btn-sm me-2"
        for mission in mission_list:
            if stop_requested:  
                return html.I(className="fas fa-play"), "btn btn-warning btn-sm me-2"
            rviz_section.publish_goal(mission["x"], mission["y"], np.arctan2(mission["z"], mission["w"]))
            start_time = time.time()
            while not check_goal_status():
                if time.time() - start_time > 30:
                    print("Timeout: Goal không hoàn thành!")
                    break
                if stop_requested:  
                    return html.I(className="fas fa-pause"), "btn btn-warning btn-sm me-2"
                time.sleep(1)

            print(f"Đã hoàn thành nhiệm vụ: {mission['id']}")
            remove_completed_mission(mission["id"])
        return html.I(className="fas fa-play"), "btn btn-success btn-sm me-2"
    else:
        stop_requested = True  
        return html.I(className="fas fa-play"), "btn btn-success btn-sm me-2"

@app.callback(
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

@app.callback(
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

@app.callback(
    Output("joystick-popup-container", "children"),
    Input("open-joystick-btn", "n_clicks"),
    prevent_initial_call=True,
)
def open_joystick(n_clicks):
    global joystick_control
    joystick_control.stop()  
    return joystick_control.create_joystick_popup()

@app.callback(
    Output("joystick-modal", "is_open"),
    Input("close-joystick-btn", "n_clicks"),
    State("joystick-modal", "is_open"),
    prevent_initial_call=True
)
def close_joystick(n_clicks, is_open):
    return not is_open

@app.callback(
    [Output("linear-speed-display", "children"),
     Output("angular-speed-display", "children")],
    [Input("linear-speed-input", "value"),
     Input("angular-speed-input", "value")],
    prevent_initial_call=True
)
def update_speed(linear_speed_value, angular_speed_value):
    global joystick_control

    linear_speed = linear_speed_value
    angular_speed = angular_speed_value

    joystick_control.linear_speed = linear_speed
    joystick_control.angular_speed = angular_speed

    speed_message = "Giá trị tốc độ đã được cập nhật"

    return (
        f"Linear Speed: {linear_speed_value}",
        f"Angular Speed: {angular_speed_value}",
    )

@app.callback(
    Output("joystick-output", "children"),
    [Input("forward-button", "n_clicks_timestamp"),
     Input("backward-button", "n_clicks_timestamp"),
     Input("left-button", "n_clicks_timestamp"),
     Input("right-button", "n_clicks_timestamp"),
     Input("forward-left-button", "n_clicks_timestamp"),
     Input("forward-right-button", "n_clicks_timestamp"),
     Input("back-left-button", "n_clicks_timestamp"),
     Input("back-right-button", "n_clicks_timestamp"),
     Input("stop-button", "n_clicks")],
    prevent_initial_call=True
)
def teleop_control(f, b, l, r, fl, fr, bl, br, s):
    ctx = callback_context
    joystick_control.stop()
    triggered_id = ctx.triggered[0]['prop_id'].split('.')[0] if ctx.triggered else None

    if triggered_id == "forward-button":
        joystick_control.move_forward()
        return "Moving Forward"
    elif triggered_id == "backward-button":
        joystick_control.move_backward()
        return "Moving Backward"
    elif triggered_id == "left-button":
        joystick_control.turn_left()
        return "Turning Left"
    elif triggered_id == "right-button":
        joystick_control.turn_right()
        return "Turning Right"
    elif triggered_id == "forward-left-button":
        joystick_control.move_forward_left()
        return "Moving Forward Left"
    elif triggered_id == "forward-right-button":
        joystick_control.move_forward_right()
        return "Moving Forward Right"
    elif triggered_id == "back-left-button":
        joystick_control.move_backward_left()
        return "Moving Backward Left"
    elif triggered_id == "back-right-button":
        joystick_control.move_backward_right()
        return "Moving Backward Right"
    elif triggered_id == "stop-button":
        joystick_control.stop()
        return "Stop"
    else:
        return "No movement"

@app.callback(
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

@app.callback(
    Output("lidar-image", "src"),
    Input("interval-component", "n_intervals")
)
def update_lidar_image(n):
    timestamp = int(time.time())
    return f"/static/lidar_image.png?{timestamp}"

@app.callback(
    [Output("lidar-f-image", "src"), Output("lidar-b-image", "src")],
    Input("interval-component", "n_intervals")
)
def update_lidar_images(n):
    timestamp = int(time.time())
    return (
        f"/static/f_scan_image.png?{timestamp}",
        f"/static/b_scan_image.png?{timestamp}"
    )

@app.callback(
    [Output("path-image", "src")],
    Input("interval-component", "n_intervals")
)
def update_path_image(n):
    random_value = random.randint(1, 100000)
    return (
        f"/static/path_image.png?random={random_value}",
    )

@app.callback(
    Output("lines-image", "src"),
    Input("interval-component", "n_intervals")
)
def update_lines_image(n):
    timestamp = int(time.time())
    return f"/static/line_image.png?{timestamp}"

@app.callback(
    Output("markers-image", "src"),
    Input("interval-component", "n_intervals")
)
def update_markers_image(n):
    timestamp = int(time.time())
    return f"/static/all_markers.png?{timestamp}"

@app.callback(
    Output("dockers-image", "src"),
    Input("interval-component", "n_intervals")
)
def update_markers_image(n):
    timestamp = int(time.time())
    return f"/static/dockers.png?{timestamp}"

@app.callback(
    Output("map-image", "src"),
    Input("interval-component", "n_intervals")
)
def update_map_image(n):
    timestamp = int(time.time())
    return f"/static/map_image.png?{timestamp}"

last_modified_time = 0

@app.callback(
    [Output("robot-image", "src")],
    Input("interval-component", "n_intervals")
)
def update_robot_image(n):
    global last_modified_time
    image_path = "static/robot_image.png"
    try:
        modified_time = os.path.getmtime(image_path)
    except FileNotFoundError:
        print(f"Error: Image file not found: {image_path}")
        return [dash.no_update]
    except Exception as e:
        print(f"Other error getting modified time: {e}")
        return [dash.no_update]
    if modified_time > last_modified_time:
        last_modified_time = modified_time
        print("dash- callback updating img with: " + image_path + "?time=" + str(modified_time))
        return [f"/static/robot_image.png?time={modified_time}"]
    else:
        raise PreventUpdate

app.layout = html.Div(
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

@app.callback(
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

@app.callback(
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
register_callbacks(app)

if __name__ == "__main__":
    app.run_server(debug=True, host='0.0.0.0', port=8000)