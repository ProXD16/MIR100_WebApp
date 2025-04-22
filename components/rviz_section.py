from dash import html, dcc, callback, Input, Output, State, no_update, Dash
import dash_bootstrap_components as dbc
import rospy, dash, io, base64, json, os, math, threading
from geometry_msgs.msg import PoseStamped, Twist, PoseWithCovarianceStamped
import numpy as np
from PIL import Image
import plotly.express as px
from utils.move_along_path_line import *
from mpc_controller import LineMPCController, Spline3_MPCController, Spline5_MPCController, Arc_MPCController
from components import button_primary_style, button_style

is_moving = True
line_controller = LineMPCController(dt=0.1, v_max=1.0, v_min=-1.0, omega_max=1.5, omega_min=-1.5)
spline3_controller = Spline3_MPCController(dt=0.01, v_max=1.0, v_min=-1.0, omega_max=1.5, omega_min=-1.5, lookahead_distance=0.1)
spline5_controller = Spline5_MPCController(dt=0.001, v_max=1.0, v_min=-1.0, omega_max=1.5, omega_min=-1.5, lookahead_distance=0.1)
arc_controller = Arc_MPCController(dt=0.1, v_max=1.0, v_min=-1.0, omega_max=1.5, omega_min=-1.5, lookahead_distance=0.2)
app = Dash(__name__, external_stylesheets=[dbc.themes.BOOTSTRAP])

class RVizSection:
    FIXED_ARROW_LENGTH = 30

    def __init__(self, goal_topic="/move_base_simple/goal", twist_topic="/cmd_vel", pose_topic="/amcl_pose"):
        self.goal_topic = goal_topic
        self.twist_topic = twist_topic
        self.pose_topic = pose_topic
        try:
            self.goal_pub = rospy.Publisher(self.goal_topic, PoseStamped, queue_size=10)
            self.twist_pub = rospy.Publisher(self.twist_topic, Twist, queue_size=10)
            self.current_pose = None
            self.pose_sub = rospy.Subscriber(self.pose_topic, PoseWithCovarianceStamped, self.pose_callback)
        except rospy.exceptions.ROSException as e:
            print(f"Error connecting to ROS: {e}")
            self.goal_pub = None
            self.twist_pub = None
        self.map_data = self.load_image_as_numpy("static/map_image.png")
        rospy.Subscriber("/amcl_pose", PoseWithCovarianceStamped, self.pose_callback)

    def pose_callback(self, msg):
        self.current_pose = msg.pose.pose

    def get_robot_position(self):
        if self.current_pose is None:
            return None, None, None
        
        pos = self.current_pose.position
        ori = self.current_pose.orientation

        _, _, yaw = tf.transformations.euler_from_quaternion([
            ori.x, ori.y, ori.z, ori.w
        ])
        return pos.x, pos.y, yaw

    def load_image_as_numpy(self, image_path):
        try:
            img = Image.open(image_path).convert('L')
            map_data = np.array(img)
            if len(map_data.shape) > 2:
                print("WARNING: Image has more than two dimensions, potential color issue")
            return map_data
        except FileNotFoundError:
            print(f"Error: Image file not found at {image_path}")
            return self.generate_fake_map()

    def generate_fake_map(self):
        size = 500
        return np.random.randint(0, 255, size=(size, size))

    def create_composite_image(self):
        map_path = "static/map_image.png"
        if not os.path.exists(map_path):
            print(f"[ERROR] Không tìm thấy ảnh bản đồ tại: {map_path}")
            width, height = 400, 400 
        else:
            map_img = Image.open(map_path)
            width, height = map_img.size  
        final_img = Image.new("RGBA", (width, height), (255, 255, 255, 0)) 
        return final_img

    def create_figure(self, drawing_enabled=False, start_x=None, start_y=None, end_x=None, end_y=None):
        composite_img = self.create_composite_image()
        if composite_img is None:
            fake_map = self.generate_fake_map()
            fig = px.imshow(fake_map, color_continuous_scale='gray')
        else:
            img_byte_arr = io.BytesIO()
            composite_img.save(img_byte_arr, format='PNG')
            img_byte_arr = img_byte_arr.getvalue()
            encoded_image = base64.b64encode(img_byte_arr).decode()
            img_data = f'data:image/png;base64,{encoded_image}'
            fig = px.imshow(Image.open(io.BytesIO(base64.b64decode(encoded_image))), binary_string=True)
            fig.update_layout(
                images=[dict(
                    source=img_data,
                    xref="x", yref="y",
                    x=0, y=composite_img.size[1],
                    sizex=composite_img.size[0], sizey=composite_img.size[1],
                    sizing="stretch",
                    opacity=1,
                    layer="below")],
                plot_bgcolor='rgba(255,255,255,0)',
                paper_bgcolor='rgba(255,255,255,0)'
            )

        dragmode = "drawline" if drawing_enabled else False
        fig.update_layout(
            dragmode=dragmode,
            xaxis=dict(showgrid=False, zeroline=False, visible=False),
            yaxis=dict(showgrid=False, zeroline=False, visible=False, scaleratio=1),
            margin=dict(l=0, r=0, b=0, t=0),
            height=600,
            width=800,
            newshape_line_color='red',
            coloraxis_showscale=False
        )
        fig.update_traces(
            hovertemplate=None,
            hoverinfo='skip'
        )

        if start_x is not None and start_y is not None and end_x is not None and end_y is not None:
            dx = end_x - start_x
            dy = end_y - start_y
            angle = math.atan2(dy, dx)
            fixed_end_x = start_x + self.FIXED_ARROW_LENGTH * math.cos(angle)
            fixed_end_y = start_y + self.FIXED_ARROW_LENGTH * math.sin(angle)
            fig.add_annotation(
                x=fixed_end_x, y=fixed_end_y, ax=start_x, ay=start_y,
                xref="x", yref="y", axref="x", ayref="y",
                arrowhead=2, arrowcolor='red', arrowwidth=2,
            )
        return fig

    def publish_goal(self, x, y, angle):
        if self.goal_pub:
            try:
                pose = PoseStamped()
                pose.header.stamp = rospy.Time.now()
                pose.header.frame_id = "map"
                pose.pose.position.x = float(x)
                pose.pose.position.y = float(y)
                pose.pose.orientation.z = math.sin(angle / 2)
                pose.pose.orientation.w = math.cos(angle / 2)
                self.goal_pub.publish(pose)
                return "Goal published successfully! (x: {}, y: {}, angle: {})".format(x, y, angle)
            except Exception as e:
                return f"Error publishing goal: {e}"
        return "Goal publisher not initialized."

    def publish_twist(self, linear_x=0.0, angular_z=0.0):
        if self.twist_pub:
            try:
                twist = Twist()
                twist.linear.x = float(linear_x)
                twist.angular.z = float(angular_z)
                self.twist_pub.publish(twist)
                return f"Twist published: linear_x={linear_x}, angular_z={angular_z}"
            except Exception as e:
                return f"Error publishing twist: {e}"
        return "Twist publisher not initialized."

    def stop_robot(self):
        return self.publish_twist(0.0, 0.0)

    def is_at_position(self, target_x, target_y, tolerance=0.2):
        if self.current_pose is None:
            return False
        current_x = self.current_pose.position.x
        current_y = self.current_pose.position.y
        distance = math.sqrt((current_x - target_x)**2 + (current_y - target_y)**2)
        return distance < tolerance

def create_rviz_section():
    active_button_style = {
        "background-color": "#2ecc71",
        "color": "white",
        "box-shadow": "0 2px 5px rgba(0, 0, 0, 0.3)",
    }
    emergency_stop_style = {
        **button_style,
        "background-color": "#DC3545",  # Red when active
        "color": "white",
    }
    emergency_stopped_style = {
        **button_style,
        "background-color": "#6C757D",  # Gray when stopped
        "color": "white",
    }
    layout = html.Div(
        style={'backgroundColor': '#FFFFFF', 'height': '100vh', 'fontFamily': 'Arial, sans-serif'},
        children=[
            html.Div(
                [
                    html.H3("RVIZ INTERFACE", className="mb-3", style={"color": "#2C3E50", "fontWeight": "bold"}),
                    dbc.Row(
                        [
                            dbc.Col(
                                html.Div(
                                    [
                                        html.Button(
                                            "send goal",
                                            id="send-goal-btn",
                                            className="fas fa-bullseye me-2",
                                            style=button_primary_style,
                                            n_clicks=0,
                                        ),
                                        html.Button(
                                            "2d nav goal",
                                            id="nav-goal-btn",
                                            style=button_primary_style,
                                            className="fas fa-crosshairs me-2",
                                            n_clicks=0,
                                        ),
                                        html.Button(
                                            "move line",
                                            id="move-line-btn",
                                            style=button_primary_style,
                                            className="fas fa-route me-2",
                                            n_clicks=0,
                                        ),
                                        html.Button(
                                            "move arc",
                                            id="move-arc-btn",
                                            style=button_primary_style,
                                            className="fas fa-circle-notch me-2",
                                            n_clicks=0,
                                        ),
                                        html.Button(
                                            "move path",
                                            id="move-path-btn",
                                            style=button_primary_style,
                                            className="fas fa-road me-2",
                                            n_clicks=0,
                                        ),
                                        html.Button(
                                            "move spline 3",
                                            id="move-spline3-btn",
                                            style=button_primary_style,
                                            className="fas fa-wave-square me-2",
                                            n_clicks=0,
                                        ),
                                        html.Button(
                                            "move spline 5",
                                            id="move-spline5-btn",
                                            style=button_primary_style,
                                            className="fas fa-bezier-curve me-2",
                                            n_clicks=0,
                                        ),
                                        html.Button(
                                            "emergency stop",
                                            id="emergency-stop-btn",
                                            style=emergency_stop_style,
                                            className="fas fa-stop me-2",
                                            n_clicks=0,
                                        ),
                                    ],
                                    className="mb-3"
                                ),
                                width=11,
                            ),
                        ]
                    ),
                    html.Div(
                        [
                            dcc.Graph(
                                id="map-graph",
                                figure=RVizSection().create_figure(),
                                style={
                                    "width": "800px",
                                    "height": "600px",
                                    "background": "rgba(0, 0, 0, 0)",
                                    "position": "absolute",
                                    "z-index": "8",
                                    "top": "0",
                                    "left": "0",
                                    "border": "1px solid #CED4DA",
                                    "borderRadius": "9px",
                                },
                                config={'scrollZoom': False, 'displayModeBar': False}
                            ),
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
                                    "top": "0",
                                    "left": "0",
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
                                    "z-index": "2",
                                    "top": "0",
                                    "left": "0",
                                },
                            ),
                            html.Img(
                                id="lidar-b-image",
                                src="/static/lidar_b_image.png",
                                style={
                                    "width": "800px",
                                    "height": "600px",
                                    "border": "2px solid #34495E",
                                    "object-fit": "contain",
                                    "position": "absolute",
                                    "z-index": "3",
                                    "top": "0",
                                    "left": "0",
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
                                    "z-index": "4",
                                    "top": "0",
                                    "left": "0",
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
                                    "z-index": "5",
                                    "top": "0",
                                    "left": "0",
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
                                    "z-index": "6",
                                    "top": "0",
                                    "left": "0",
                                },
                            ),
                            html.Img(
                                id="costmap-image",
                                src="/static/cost_map_image.png",
                                style={
                                    "width": "800px",
                                    "height": "600px",
                                    "border": "2px solid #34495E",
                                    "object-fit": "contain",
                                    "position": "absolute",
                                    "z-index": "7",
                                    "top": "0",
                                    "left": "0",
                                },
                            ),
                            html.Img(
                                id="global-costmap-image",
                                src="/static/global_costmap_image.png",
                                style={
                                    "width": "800px",
                                    "height": "600px",
                                    "border": "2px solid #34495E",
                                    "object-fit": "contain",
                                    "position": "absolute",
                                    "z-index": "8",
                                    "top": "0",
                                    "left": "0",
                                },
                            ),
                        ],
                        style={"position": "relative", "width": "800px", "height": "600px"},
                    ),
                    html.P("Draw line and after release, it sends the goal", className="text-info mt-2", style={"fontStyle": "italic"}),
                    html.Div(id="goal-status"),
                    dcc.Interval(
                        id='interval-component',
                        interval=1 * 1000,
                        n_intervals=0
                    ),
                    dcc.Interval(
                        id='move-line-interval',
                        interval=1 * 1000,
                        disabled=True
                    ),
                    dcc.Interval(
                        id='move-spline-interval',
                        interval=1 * 1000,
                        disabled=True
                    ),
                    dcc.Store(id="drag-start-coords", data=None),
                    dcc.Store(id="drawing-enabled", data=False),
                    dcc.Store(id="latest-goal", data=None),
                    dcc.Store(id="move-line-data", data=None),
                    dcc.Store(id="path-line-data", data=[]),
                    dcc.Store(id="spline3-coordinates", data=[]),
                    dcc.Store(id="spline5-coordinates", data=[]),
                    dcc.Store(id="current-line-index", data=0),
                    dcc.Store(id="button-style-store", data={"nav_goal_button": button_style, "active_nav_goal_button": active_button_style}),
                    dcc.Store(id="emergency-stop-state", data={"is_stopped": False, "style": emergency_stop_style}),
                    dbc.Modal(
                        [
                            dbc.ModalHeader(dbc.ModalTitle("Enter Goal Coordinates")),
                            dbc.ModalBody(
                                dbc.Form(
                                    [
                                        dbc.Row(
                                            [
                                                dbc.Col(dbc.Label("X:", html_for="goal-x")),
                                                dbc.Col(dbc.Input(type="number", id="goal-x", placeholder="X Coordinate")),
                                            ],
                                            className="mb-3",
                                        ),
                                        dbc.Row(
                                            [
                                                dbc.Col(dbc.Label("Y:", html_for="goal-y")),
                                                dbc.Col(dbc.Input(type="number", id="goal-y", placeholder="Y Coordinate")),
                                            ],
                                            className="mb-3",
                                        ),
                                        dbc.Row(
                                            [
                                                dbc.Col(dbc.Label("Z:", html_for="goal-z")),
                                                dbc.Col(dbc.Input(type="number", id="goal-z", placeholder="Z Coordinate", value=0)),
                                            ],
                                            className="mb-3",
                                        ),
                                        dbc.Row(
                                            [
                                                dbc.Col(dbc.Label("W:", html_for="goal-w")),
                                                dbc.Col(dbc.Input(type="number", id="goal-w", placeholder="W Orientation", value=1)),
                                            ],
                                            className="mb-3",
                                        ),
                                    ]
                                )
                            ),
                            dbc.ModalFooter(
                                [
                                    dbc.Button("Close", id="close-goal-modal", className="ms-auto", style=button_style),
                                    dbc.Button("Send", id="send-goal-modal-btn", color="primary", className="ms-2", style=button_style),
                                ]
                            ),
                        ],
                        id="goal-modal",
                        is_open=False,
                    ),
                    dbc.Modal(
                        [
                            dbc.ModalHeader(dbc.ModalTitle("Set Movement Speed")),
                            dbc.ModalBody(
                                dbc.Form(
                                    [
                                        dbc.Row(
                                            [
                                                dbc.Col(dbc.Label("Linear Speed (m/s):", html_for="linear-speed")),
                                                dbc.Col(dbc.Input(type="number", id="linear-speed", placeholder="Enter speed", value=0.2, step=0.1)),
                                            ],
                                            className="mb-3",
                                        ),
                                        dbc.Row(
                                            [
                                                dbc.Col(dbc.Label("Angular Speed (rad/s):", html_for="angular-speed")),
                                                dbc.Col(dbc.Input(type="number", id="angular-speed", placeholder="Enter angular speed", value=0.5, step=0.1)),
                                            ],
                                            className="mb-3",
                                        ),
                                    ]
                                )
                            ),
                            dbc.ModalFooter(
                                [
                                    dbc.Button("Cancel", id="cancel-line-modal", className="ms-auto", style=button_style),
                                    dbc.Button("Start", id="start-line-modal-btn", color="primary", className="ms-2", style=button_style),
                                ]
                            ),
                        ],
                        id="line-modal",
                        is_open=False,
                    ),
                    dbc.Modal(
                        [
                            dbc.ModalHeader(dbc.ModalTitle("Move Along Spline")),
                            dbc.ModalBody("Do you want to start moving along the spline?"),
                            dbc.ModalFooter(
                                [
                                    dbc.Button("Cancel", id="cancel-spline-modal", className="ms-auto", style=button_style),
                                    dbc.Button("Start", id="start-spline-modal-btn", color="primary", className="ms-2", style=button_style),
                                ]
                            ),
                        ],
                        id="spline-modal",
                        is_open=False,
                    ),
                    dbc.Modal(
                        [
                            dbc.ModalHeader(dbc.ModalTitle("Move Along Spline 5")),
                            dbc.ModalBody("Do you want to start moving along the spline 5?"),
                            dbc.ModalFooter(
                                [
                                    dbc.Button("Cancel", id="cancel-spline5-modal", className="ms-auto", style=button_style),
                                    dbc.Button("Start", id="start-spline5-modal-btn", color="primary", className="ms-2", style=button_style),
                                ]
                            ),
                        ],
                        id="spline5-modal",
                        is_open=False,
                    ),
                    dbc.Modal(
                        [
                            dbc.ModalHeader(dbc.ModalTitle("Move Along Full Path")),
                            dbc.ModalBody("Do you want to start moving along the full path?"),
                            dbc.ModalFooter([
                                dbc.Button("Cancel", id="cancel-path-modal", className="ms-auto"),
                                dbc.Button("Start", id="start-path-modal-btn", color="primary", className="ms-2"),
                            ]),
                        ],
                        id="path-modal",
                        is_open=False,
                    ),
                    dbc.Modal(
                        [
                            dbc.ModalHeader(dbc.ModalTitle("Move Along Arc")),
                            dbc.ModalBody("Do you want to start moving along the arc?"),
                            dbc.ModalFooter([
                                dbc.Button("Cancel", id="cancel-arc-modal", className="ms-auto"),
                                dbc.Button("Start", id="start-arc-modal-btn", color="primary", className="ms-2"),
                            ]),
                        ],
                        id="arc-modal",
                        is_open=False,
                    ),
                ],
                style={
                    "padding": "30px",
                    "flex": "1",
                    "marginLeft": "auto",
                    "marginRight": "auto",
                    "marginTop": "50px",
                    "maxWidth": "1200px",
                    "boxShadow": "0 4px 8px rgba(0, 0, 0, 0.1)",
                    "borderRadius": "12px",
                    "overflowY": "auto",
                },
            )
        ]
    )
    return layout

def load_map_info():
    map_info_path = "static/map_image.json"
    if os.path.exists(map_info_path):
        with open(map_info_path, "r") as f:
            return json.load(f)
    else:
        print("WARNING: map_info.json not found. Using default values or disabling auto-goal sending.")
        return None

# Callbacks
@callback(
    Output("goal-modal", "is_open"),
    [Input("send-goal-btn", "n_clicks"), Input("close-goal-modal", "n_clicks"), Input("send-goal-modal-btn", "n_clicks")],
    [State("goal-modal", "is_open")],
    prevent_initial_call=True,
)
def toggle_modal(n1, n2, n3, is_open):
    ctx = dash.callback_context
    if not ctx.triggered:
        return False
    button_id = ctx.triggered[0]['prop_id'].split('.')[0]
    if button_id == "send-goal-btn":
        return True
    elif button_id == "close-goal-modal" or button_id == "send-goal-modal-btn":
        return False
    return is_open

@callback(
    Output("goal-status", "children"),
    [Input("send-goal-modal-btn", "n_clicks")],
    [
        State("goal-x", "value"),
        State("goal-y", "value"),
        State("goal-z", "value"),
        State("goal-w", "value"),
    ],
    prevent_initial_call=True,
)
def send_goal_coordinates(n_clicks, x, y, z, w):
    if n_clicks:
        rviz_section = RVizSection()
        angle = math.atan2(z, w)
        status = rviz_section.publish_goal(x, y, angle)
        return status
    return no_update

@callback(
    Output("map-graph", "figure"),
    [Input("map-graph", "clickData"), Input("map-graph", "relayoutData"), Input("interval-component", "n_intervals")],
    [State("map-graph", "figure"), State("drag-start-coords", "data"), State("drawing-enabled", "data")],
    prevent_initial_call=True,
)
def update_map(clickData, relayoutData, n_intervals, existing_map, drag_start_coords, drawing_enabled):
    ctx = dash.callback_context
    rviz_section = RVizSection()
    if not ctx.triggered:
        return existing_map
    trigger_id = ctx.triggered[0]['prop_id'].split('.')[0]

    if trigger_id == "map-graph" and clickData and drawing_enabled:
        start_x = clickData['points'][0]['x']
        start_y = clickData['points'][0]['y']
        return rviz_section.create_figure(drawing_enabled, start_x=start_x, start_y=start_y, end_x=start_x, end_y=start_y)

    elif trigger_id == "map-graph" and relayoutData and drag_start_coords and drawing_enabled:
        start_x = drag_start_coords["start_x"]
        start_y = drag_start_coords["start_y"]
        end_x = relayoutData.get("xaxis.range[0]")
        end_y = relayoutData.get("yaxis.range[1]")
        return rviz_section.create_figure(drawing_enabled, start_x=start_x, start_y=start_y, end_x=end_x, end_y=end_y)

    elif trigger_id == "interval-component":
        return rviz_section.create_figure(drawing_enabled)

    return existing_map

@callback(
    Output("drag-start-coords", "data"),
    [Input("map-graph", "clickData")],
    [State("drawing-enabled", "data")],
    prevent_initial_call=True
)
def store_drag_start_coords(clickData, drawing_enabled):
    if clickData and drawing_enabled:
        return {"start_x": clickData['points'][0]['x'], "start_y": clickData['points'][0]['y']}
    return no_update

@callback(
    [Output("drawing-enabled", "data"), Output("nav-goal-btn", "style")],
    [Input("nav-goal-btn", "n_clicks")],
    [State("drawing-enabled", "data"), State("button-style-store", "data")],
    prevent_initial_call=True
)
def toggle_drawing_mode(n_clicks, drawing_enabled, button_style_store):
    if n_clicks is None:
        return no_update, no_update

    new_drawing_enabled = not drawing_enabled
    nav_goal_style = button_style_store["active_nav_goal_button"] if new_drawing_enabled else button_style_store["nav_goal_button"]

    return new_drawing_enabled, nav_goal_style

@callback(
    [Output("goal-status", "children", allow_duplicate=True),
     Output("drawing-enabled", "data", allow_duplicate=True),
     Output("nav-goal-btn", "style", allow_duplicate=True)],
    Input("map-graph", "relayoutData"),
    [State("drag-start-coords", "data"), State("drawing-enabled", "data"), State("button-style-store", "data")],
    prevent_initial_call=True
)
def auto_send_goal(relayoutData, drag_start_coords, drawing_enabled, button_style_store):
    if not (relayoutData and drawing_enabled):
        print("Không thỏa mãn điều kiện gửi goal")
        return no_update, no_update, no_update

    map_info = load_map_info()

    if not map_info:
        print("Không thể tải thông tin bản đồ từ map_info.json. Không tự động gửi goal.")
        return "Không thể gửi goal tự động do thiếu thông tin bản đồ.", no_update, no_update

    if "shapes" not in relayoutData or not relayoutData["shapes"]:
        print("relayoutData không có shapes hợp lệ!")
        return no_update, no_update, no_update

    shape = relayoutData["shapes"][0]
    x_pixel_start, y_pixel_start = shape["x0"], shape["y0"]
    x_pixel_end, y_pixel_end = shape["x1"], shape["y1"]

    start_x = map_info["origin_x"] + (x_pixel_start * map_info["resolution"])
    start_y = map_info["origin_y"] + ((map_info["height"] - y_pixel_start) * map_info["resolution"])
    end_x = map_info["origin_x"] + (x_pixel_end * map_info["resolution"])
    end_y = map_info["origin_y"] + ((map_info["height"] - y_pixel_end) * map_info["resolution"])

    dx = end_x - start_x
    dy = end_y - start_y
    angle = math.atan2(dy, dx)

    rviz_section = RVizSection()
    status = rviz_section.publish_goal(start_x, start_y, angle)

    new_drawing_enabled = False
    nav_goal_style = button_style_store["nav_goal_button"]

    return status, new_drawing_enabled, nav_goal_style

@callback(
    Output("line-modal", "is_open"),
    [Input("move-line-btn", "n_clicks"), Input("cancel-line-modal", "n_clicks"), Input("start-line-modal-btn", "n_clicks")],
    [State("line-modal", "is_open")],
    prevent_initial_call=True,
)
def toggle_line_modal(n1, n2, n3, is_open):
    ctx = dash.callback_context
    if not ctx.triggered:
        return False
    button_id = ctx.triggered[0]['prop_id'].split('.')[0]
    if button_id == "move-line-btn":
        return True
    elif button_id in ["cancel-line-modal", "start-line-modal-btn"]:
        return False
    return is_open

def move_along_line_thread():
    global is_moving
    try:
        file_path = "database_json/line_drawn.json"
        if not os.path.exists(file_path):
            print(f"[MPC] File not found: {file_path}")
            return

        with open(file_path, 'r') as f:
            path_line_data = json.load(f)

        if not path_line_data:
            print("[MPC] Không tìm thấy dữ liệu đường đi")
            return

        rviz_section = RVizSection()

        for item in path_line_data:
            if not is_moving:
                print("[MPC] Dừng khẩn cấp được kích hoạt")
                send_velocity(0.0, 0.0)
                return

            item_type = item.get("type", "line")

            if item_type == "line":
                points = zip(item['x'], item['y'])
            elif item_type == "polyline":
                points = item['points']
            else:
                print(f"[MPC] Unsupported type: {item_type}")
                continue

            for x_goal, y_goal in points:
                while is_moving:
                    pose = rviz_section.get_robot_position()
                    if pose is None or pose[0] is None:
                        continue
                    x, y, theta = pose

                    v, omega = line_controller.mpc_control(x, y, theta, x_goal, y_goal)
                    send_velocity(v, omega)

                    distance = np.sqrt((x - x_goal)**2 + (y - y_goal)**2)
                    if distance < 0.1:
                        break
                    rospy.sleep(0.1)

        send_velocity(0.0, 0.0)
        print("[MPC] Hoàn thành di chuyển theo đường.")

    except Exception as e:
        print(f"[MPC] Lỗi khi di chuyển: {e}")

@callback(
    [Output("goal-status", "children", allow_duplicate=True),
     Output("move-line-data", "data", allow_duplicate=True),
     Output("path-line-data", "data", allow_duplicate=True),
     Output("current-line-index", "data", allow_duplicate=True),
     Output("move-line-interval", "disabled", allow_duplicate=True)],
    Input("start-line-modal-btn", "n_clicks"),
    [State("linear-speed", "value"),
     State("angular-speed", "value")],
    prevent_initial_call=True,
)
def initiate_move_along_path(n_clicks, linear_speed, angular_speed):
    global is_moving
    is_moving = True
    thread = threading.Thread(target=move_along_line_thread)
    thread.start()
    return "", no_update, no_update, no_update, False

def move_along_spline3_thread(spline_points):
    global is_moving
    if len(spline_points) < 4:
        print("[Spline3] Không đủ điểm spline3")
        return

    try:
        x_start, y_start = spline_points[0]

        while is_moving:
            rviz_section = RVizSection()
            x, y, theta = rviz_section.get_robot_position()
            if x is None:
                continue

            v, omega = line_controller.mpc_control(x, y, theta, x_start, y_start)
            send_velocity(v, omega)

            dist = np.hypot(x - x_start, y - y_start)
            if dist < 0.1:
                break
            rospy.sleep(0.1)

        if not is_moving:
            print("[Spline3] Dừng khẩn cấp được kích hoạt")
            send_velocity(0.0, 0.0)
            return

        print("[Spline3] Đã đến điểm đầu spline, bắt đầu MPC spline...")

        x_vals, y_vals = zip(*spline_points)
        spline3_controller.waypoints_x = list(x_vals)
        spline3_controller.waypoints_y = list(y_vals)

        if not spline3_controller.calculate_spline():
            print("[Spline3] Không thể tính spline")
            return

        spline3_controller.reached_goal = False
        while is_moving and not spline3_controller.reached_goal:
            rviz_section = RVizSection()
            x, y, theta = rviz_section.get_robot_position()
            if x is None:
                continue
            v, omega = spline3_controller.mpc_control(x, y, theta)
            send_velocity(v, omega)
            rospy.sleep(0.1)

        send_velocity(0.0, 0.0)
        print("[Spline3] Hoàn thành bám spline3.")

    except Exception as e:
        print(f"[Spline3] Lỗi: {e}")

@callback(
    Output("goal-status", "children", allow_duplicate=True),
    Input("start-spline-modal-btn", "n_clicks"),
    prevent_initial_call=True,
)
def follow_spline3_from_json(n_clicks):
    global is_moving
    try:
        file_path = "database_json/line_drawn.json"
        if not os.path.exists(file_path):
            return f"❌ Không tìm thấy file spline3: {file_path}"

        with open(file_path, "r") as f:
            spline_data = json.load(f)

        if not spline_data or len(spline_data) == 0:
            return "⚠️ Không có dữ liệu spline3 trong file."

        for spline in spline_data:
            if spline["type"] != "spline3" or len(spline["points"]) < 4:
                continue
            if not is_moving:
                return "❌ Dừng khẩn cấp được kích hoạt"
            thread = threading.Thread(target=move_along_spline3_thread, args=(spline["points"],))
            thread.start()
            return "🚀 Đang điều khiển spline3 bằng MPC..."

        return "⚠️ Không có spline3 hợp lệ trong file."

    except Exception as e:
        return f"❌ Lỗi đọc spline3: {e}"

def move_along_spline5_thread():
    global is_moving
    try:
        file_path = "database_json/line_drawn.json"
        if not os.path.exists(file_path):
            print(f"[Spline5] Không tìm thấy file: {file_path}")
            return

        with open(file_path, "r") as f:
            spline_data = json.load(f)

        if not spline_data:
            print("[Spline5] File rỗng hoặc không có spline5")
            return

        for spline in spline_data:
            if spline["type"] != "spline5" or len(spline["points"]) < 6:
                continue
            if not is_moving:
                print("[Spline5] Dừng khẩn cấp được kích hoạt")
                send_velocity(0.0, 0.0)
                return

            x_start, y_start = spline["points"][0]
            while is_moving:
                rviz_section = RVizSection()
                x, y, theta = rviz_section.get_robot_position()
                if x is None:
                    continue
                v, omega = line_controller.mpc_control(x, y, theta, x_start, y_start)
                send_velocity(v, omega)
                if np.hypot(x - x_start, y - y_start) < 0.1:
                    break
                rospy.sleep(0.1)

            print("[Spline5] Đã đến điểm đầu spline5, bắt đầu MPC spline5...")

            x_vals, y_vals = zip(*spline["points"])
            spline5_controller.waypoints_x = list(x_vals)
            spline5_controller.waypoints_y = list(y_vals)

            if not spline5_controller.calculate_spline():
                print("[Spline5] Không thể tính spline")
                continue

            spline5_controller.reached_goal = False
            while is_moving and not spline5_controller.reached_goal:
                rviz_section = RVizSection()
                x, y, theta = rviz_section.get_robot_position()
                if x is None:
                    continue
                v, omega = spline5_controller.mpc_control(x, y, theta)
                send_velocity(v, omega)
                rospy.sleep(0.1)

            send_velocity(0.0, 0.0)
            print("[Spline5] Đã hoàn thành một spline5")

    except Exception as e:
        print(f"[Spline5] Lỗi MPC: {e}")

@callback(
    Output("goal-status", "children", allow_duplicate=True),
    Input("start-spline5-modal-btn", "n_clicks"),
    prevent_initial_call=True,
)
def follow_spline5_from_modal(n_clicks):
    global is_moving
    is_moving = True
    thread = threading.Thread(target=move_along_spline5_thread)
    thread.start()
    return "🚀 Đang điều khiển spline5 bằng MPC..."

@callback(
    Output("spline-modal", "is_open"),
    [Input("move-spline3-btn", "n_clicks"),
     Input("cancel-spline-modal", "n_clicks"),
     Input("start-spline-modal-btn", "n_clicks")],
    [State("spline-modal", "is_open")],
    prevent_initial_call=True
)
def toggle_spline_modal(n1, n2, n3, is_open):
    ctx = dash.callback_context
    if not ctx.triggered:
        return False
    trigger = ctx.triggered[0]['prop_id'].split('.')[0]
    if trigger == "move-spline3-btn":
        return True
    elif trigger in ["cancel-spline-modal", "start-spline-modal-btn"]:
        return False
    return is_open

@callback(
    Output("spline5-modal", "is_open"),
    [Input("move-spline5-btn", "n_clicks"),
     Input("cancel-spline5-modal", "n_clicks"),
     Input("start-spline5-modal-btn", "n_clicks")],
    [State("spline5-modal", "is_open")],
    prevent_initial_call=True
)
def toggle_spline5_modal(n1, n2, n3, is_open):
    ctx = dash.callback_context
    if not ctx.triggered:
        return False
    trigger = ctx.triggered[0]['prop_id'].split('.')[0]
    if trigger == "move-spline5-btn":
        return True
    elif trigger in ["cancel-spline5-modal", "start-spline5-modal-btn"]:
        return False
    return is_open

def move_along_path_thread():
    global is_moving
    try:
        file_path = "database_json/line_drawn.json"
        if not os.path.exists(file_path):
            print(f"[Path] Không tìm thấy file: {file_path}")
            return

        with open(file_path, 'r') as f:
            path_data = json.load(f)

        if not path_data:
            print("[Path] File trống hoặc không có dữ liệu")
            return

        rviz_section = RVizSection()

        for item in path_data:
            if not is_moving:
                print("[Path] Dừng khẩn cấp được kích hoạt")
                send_velocity(0.0, 0.0)
                return

            if "type" not in item:
                continue

            item_type = item["type"]

            if item_type == "line":
                x_vals = item["x"]
                y_vals = item["y"]
            elif item_type == "spline3":
                points = item["points"]
                x_vals, y_vals = zip(*points)
            elif item_type == "spline5":
                points = item["points"]
                x_vals, y_vals = zip(*points)
            elif item_type == "polyline":
                points = item["points"]
                x_vals, y_vals = zip(*points)
            elif item_type == "arc":
                start_x = item["start_x"]
                start_y = item["start_y"]
            else:
                continue

            if item_type != "arc":
                x_start, y_start = x_vals[0], y_vals[0]
            else:
                x_start, y_start = start_x, start_y

            while is_moving:
                pose = rviz_section.get_robot_position()
                if pose is None or pose[0] is None:
                    continue
                x, y, theta = pose
                v, omega = line_controller.mpc_control(x, y, theta, x_start, y_start)
                send_velocity(v, omega)
                if np.hypot(x - x_start, y - y_start) < 0.1:
                    break
                rospy.sleep(0.1)

            if item_type == "line":
                for i in range(len(x_vals)):
                    if not is_moving:
                        print("[Path] Dừng khẩn cấp được kích hoạt")
                        send_velocity(0.0, 0.0)
                        return
                    x_goal, y_goal = x_vals[i], y_vals[i]
                    while is_moving:
                        pose = rviz_section.get_robot_position()
                        if pose is None or pose[0] is None:
                            continue
                        x, y, theta = pose
                        v, omega = line_controller.mpc_control(x, y, theta, x_goal, y_goal)
                        send_velocity(v, omega)
                        if np.hypot(x - x_goal, y - y_goal) < 0.1:
                            break
                        rospy.sleep(0.1)

            elif item_type == "spline3":
                spline3_controller.waypoints_x = list(x_vals)
                spline3_controller.waypoints_y = list(y_vals)
                if not spline3_controller.calculate_spline():
                    print("[Path] Lỗi tính spline3")
                    continue
                spline3_controller.reached_goal = False
                while is_moving and not spline3_controller.reached_goal:
                    pose = rviz_section.get_robot_position()
                    if pose is None or pose[0] is None:
                        continue
                    x, y, theta = pose
                    v, omega = spline3_controller.mpc_control(x, y, theta)
                    send_velocity(v, omega)
                    rospy.sleep(0.1)

            elif item_type == "spline5":
                spline5_controller.waypoints_x = list(x_vals)
                spline5_controller.waypoints_y = list(y_vals)
                if not spline5_controller.calculate_spline():
                    print("[Path] Lỗi tính spline5")
                    continue
                spline5_controller.reached_goal = False
                while is_moving and not spline5_controller.reached_goal:
                    pose = rviz_section.get_robot_position()
                    if pose is None or pose[0] is None:
                        continue
                    x, y, theta = pose
                    v, omega = spline5_controller.mpc_control(x, y, theta)
                    send_velocity(v, omega)
                    rospy.sleep(0.1)

            elif item_type == "polyline":
                for i in range(len(x_vals)):
                    if not is_moving:
                        print("[Path] Dừng khẩn cấp được kích hoạt")
                        send_velocity(0.0, 0.0)
                        return
                    x_goal, y_goal = x_vals[i], y_vals[i]
                    while is_moving:
                        pose = rviz_section.get_robot_position()
                        if pose is None or pose[0] is None:
                            continue
                        x, y, theta = pose
                        v, omega = line_controller.mpc_control(x, y, theta, x_goal, y_goal)
                        send_velocity(v, omega)
                        if np.hypot(x - x_goal, y - y_goal) < 0.1:
                            break
                        rospy.sleep(0.1)

            elif item_type == "arc":
                center_x = item["center_x"]
                center_y = item["center_y"]
                radius = item["radius"]
                start_angle = item["start_angle"]
                end_angle = item["end_angle"]
                arc_controller.waypoints_x = [start_x, start_x + 0.1, center_x]
                arc_controller.waypoints_y = [start_y, start_y + 0.1, center_y]
                arc_controller.circle_center = (center_x, center_y)
                arc_controller.circle_radius = radius
                arc_controller.arc_start_angle = start_angle
                arc_controller.arc_end_angle = end_angle
                arc_controller.start_time = rospy.Time.now().to_sec()
                arc_controller.reached_goal = False
                while is_moving and not arc_controller.reached_goal:
                    pose = rviz_section.get_robot_position()
                    if pose is None or pose[0] is None:
                        continue
                    x, y, theta = pose
                    v, omega = arc_controller.mpc_control(x, y, theta)
                    send_velocity(v, omega)
                    rospy.sleep(0.1)

        send_velocity(0.0, 0.0)
        print("[Path] Đã hoàn thành di chuyển toàn bộ đường path")

    except Exception as e:
        print(f"[Path] Lỗi tổng hợp khi di chuyển path: {e}")

@callback(
    Output("goal-status", "children", allow_duplicate=True),
    Input("start-path-modal-btn", "n_clicks"),
    prevent_initial_call=True,
)
def follow_full_path(n_clicks):
    global is_moving
    is_moving = True
    thread = threading.Thread(target=move_along_path_thread)
    thread.start()
    return "🚀 Đang thực hiện di chuyển toàn bộ path bằng MPC..."

@callback(
    Output("path-modal", "is_open"),
    [Input("move-path-btn", "n_clicks"),
     Input("cancel-path-modal", "n_clicks"),
     Input("start-path-modal-btn", "n_clicks")],
    [State("path-modal", "is_open")],
    prevent_initial_call=True
)
def toggle_path_modal(n1, n2, n3, is_open):
    ctx = dash.callback_context
    if not ctx.triggered:
        return False
    trigger = ctx.triggered[0]['prop_id'].split('.')[0]
    if trigger == "move-path-btn":
        return True
    elif trigger in ["cancel-path-modal", "start-path-modal-btn"]:
        return False
    return is_open

def move_along_arc_thread():
    global is_moving
    try:
        file_path = "database_json/line_drawn.json"
        if not os.path.exists(file_path):
            print(f"[Arc] ❌ Không tìm thấy file: {file_path}")
            return

        with open(file_path, "r") as f:
            arc_data = json.load(f)

        if not arc_data:
            print("[Arc] ⚠️ File rỗng hoặc không có dữ liệu cung tròn")
            return

        for arc in arc_data:
            if arc["type"] != "arc":
                continue
            if not is_moving:
                print("[Arc] Dừng khẩn cấp được kích hoạt")
                send_velocity(0.0, 0.0)
                return

            start_x = arc["start_x"]
            start_y = arc["start_y"]
            center_x = arc["center_x"]
            center_y = arc["center_y"]
            radius = arc["radius"]
            start_angle = arc["start_angle"]
            end_angle = arc["end_angle"]
            rviz = RVizSection()
            arc_controller.waypoints_x = [start_x, start_x + 0.1, center_x]
            arc_controller.waypoints_y = [start_y, start_y + 0.1, center_y]
            arc_controller.circle_center = (center_x, center_y)
            arc_controller.circle_radius = radius
            arc_controller.arc_start_angle = start_angle
            arc_controller.arc_end_angle = end_angle
            arc_controller.start_time = rospy.Time.now().to_sec()
            while is_moving:
                x, y, theta = rviz.get_robot_position()
                if x is None:
                    continue
                v, omega = line_controller.mpc_control(x, y, theta, start_x, start_y)
                send_velocity(v, omega)
                if np.hypot(x - start_x, y - start_y) < 0.1:
                    break
                rospy.sleep(0.1)
            print("[Arc] ✅ Đã đến điểm bắt đầu, bắt đầu bám cung tròn...")
            arc_controller.reached_goal = False
            while is_moving and not arc_controller.reached_goal:
                x, y, theta = rviz.get_robot_position()
                if x is None:
                    continue
                v, omega = arc_controller.mpc_control(x, y, theta)
                send_velocity(v, omega)
                rospy.sleep(0.1)
            send_velocity(0.0, 0.0)
            print("[Arc] 🎯 Đã hoàn thành một cung tròn")

    except Exception as e:
        print(f"[Arc] ❌ Lỗi MPC cung tròn: {e}")

@callback(
    Output("goal-status", "children", allow_duplicate=True),
    Input("start-arc-modal-btn", "n_clicks"),
    prevent_initial_call=True,
)
def follow_arc_path(n_clicks):
    global is_moving
    is_moving = True
    thread = threading.Thread(target=move_along_arc_thread)
    thread.start()
    return "🚀 Đang thực hiện di chuyển arc bằng MPC..."

@callback(
    Output("arc-modal", "is_open"),
    [Input("move-arc-btn", "n_clicks"),
     Input("cancel-arc-modal", "n_clicks"),
     Input("start-arc-modal-btn", "n_clicks")],
    [State("arc-modal", "is_open")],
    prevent_initial_call=True
)
def toggle_arc_modal(n1, n2, n3, is_open):
    ctx = dash.callback_context
    if not ctx.triggered:
        return False
    trigger = ctx.triggered[0]['prop_id'].split('.')[0]
    if trigger == "move-arc-btn":
        return True
    elif trigger in ["cancel-arc-modal", "start-arc-modal-btn"]:
        return False
    return is_open

@callback(
    [Output("goal-status", "children", allow_duplicate=True),
     Output("emergency-stop-btn", "style"),
     Output("emergency-stop-state", "data")],
    Input("emergency-stop-btn", "n_clicks"),
    State("emergency-stop-state", "data"),
    prevent_initial_call=True
)
def emergency_stop(n_clicks, stop_state):
    if n_clicks is None:
        return no_update, no_update, no_update

    global is_moving
    rviz_section = RVizSection()
    
    is_stopped = stop_state["is_stopped"]
    emergency_stop_style = {
        **button_style,
        "background-color": "#DC3545",
        "color": "white",
    }
    emergency_stopped_style = {
        **button_style,
        "background-color": "#6C757D",
        "color": "white",
    }

    if not is_stopped:
        # Stop the robot
        is_moving = False
        status = rviz_section.stop_robot()
        new_state = {"is_stopped": True, "style": emergency_stopped_style}
        return f"Emergency Stop Activated: {status}", emergency_stopped_style, new_state
    else:
        # Resume movement
        is_moving = True
        new_state = {"is_stopped": False, "style": emergency_stop_style}
        return "", emergency_stop_style, new_state

def create_rviz_section_layout():
    return create_rviz_section()