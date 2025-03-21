import time
from dash import html, Input, Output, State
from components import RVizSection
from page_draw_mode.function_draw_mode import *
from dash import callback_context
import numpy as np
from make_marker_with_json.process_with_json import *
from make_marker_with_json.generate_image_from_json import *

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