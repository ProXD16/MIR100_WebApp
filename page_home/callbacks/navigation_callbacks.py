import time
from dash import html, Input, Output, State
from components import RVizSection
from page_draw_mode.function_draw_mode import *
from dash import callback_context
import numpy as np
from make_marker_with_json.process_with_json import *
from make_marker_with_json.generate_image_from_json import *
from components import button_default_manual_style, button_active_manual_style
from geometry_msgs.msg import Twist

try:
    rospy.init_node('mir_joystick_interface', anonymous=True)
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
except Exception as e:
    print(f"ROS initialization failed: {e}")
    pub = None

@callback(
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
    
@callback(
    Output("joystick-popup-container", "children"),
    Input("open-joystick-btn", "n_clicks"),
    prevent_initial_call=True,
)
def open_joystick(n_clicks):
    global joystick_control
    joystick_control.stop()  
    return joystick_control.create_joystick_popup()

@callback(
    Output("joystick-modal", "is_open"),
    Input("close-joystick-btn", "n_clicks"),
    State("joystick-modal", "is_open"),
    prevent_initial_call=True
)
def close_joystick(n_clicks, is_open):
    return not is_open

@callback(
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

@callback(
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
    
@callback(
    [Output("joystick-container", "style"),
     Output("manual-control", "style"),
     Output("interval-joystick", "disabled")],
    Input("manual-control", "n_clicks"),
    State("joystick-container", "style"),
    prevent_initial_call=True
)
def toggle_joystick(n_clicks, current_style):
    """
    Callback to show/hide the joystick container and enable/disable the joystick interval
    when the MANUAL CONTROL button is clicked.

    Args:
        n_clicks: Number of times the button has been clicked.
        current_style: The current style of the joystick container.

    Returns:
        Tuple of joystick container style, manual control button style, and interval disabled state.
    """
    if n_clicks and current_style:
        if current_style.get("display") == "none":
            return {"display": "block"}, button_active_manual_style, False
        else:
            # When hiding joystick, stop robot
            if pub is not None:
                twist = Twist()
                pub.publish(twist)
            return {"display": "none"}, button_default_manual_style, True
    return {"display": "none"}, button_default_manual_style, True

@callback(
    Output("joystick-data", "data"),
    Input("joystick", "angle"),
    Input("joystick", "force"),
    prevent_initial_call=True
)
def update_joystick_data(angle, force):
    """
    Callback to store joystick angle and force data.

    Args:
        angle: Joystick angle in degrees.
        force: Joystick force (0 to 1).

    Returns:
        Dictionary with angle and force.
    """
    return {"angle": angle or 0, "force": force or 0}

@callback(
    Output("joystick-output", "children", allow_duplicate=True),
    Input("interval-joystick", "n_intervals"),
    State("joystick-data", "data"),
    State("speed-scale", "value"),
    State("emergency-stop", "on"),
    prevent_initial_call=True
)
def send_twist(n, data, speed_scale, emergency_stop):
    """
    Callback to publish ROS Twist messages based on joystick input and update display.

    Args:
        n: Interval trigger count.
        data: Stored joystick data (angle, force).
        speed_scale: Speed scale from slider (0.1 to 1.0).
        emergency_stop: Boolean switch state.

    Returns:
        String for joystick-output display.
    """
    if pub is None:
        return "⚠️ ROS not initialized!"

    if emergency_stop:
        twist = Twist()
        pub.publish(twist)
        return "🛑 Emergency Stop Activated!"

    angle = data["angle"]
    force = data["force"]

    if force == 0:
        twist = Twist()
        pub.publish(twist)
        return "⏹ Robot Stopped"

    angle = angle % 360
    linear, angular = 0.0, 0.0

    linear = math.sin(math.radians(angle)) * force * speed_scale
    angular = math.cos(math.radians(angle)) * force * speed_scale * 2.0

    linear = max(min(linear, 1.0), -1.0)
    angular = max(min(angular, 2.0), -2.0)

    twist = Twist()
    twist.linear.x = linear
    twist.angular.z = angular
    pub.publish(twist)

    return f"🚀 Moving: Linear = {linear:.2f} m/s, Angular = {angular:.2f} rad/s"