import json
from dash import Input, Output, State
from page_draw_mode.function_draw_mode import *
from make_marker_with_json import *

@callback(
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

@callback(
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

@callback(
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

@callback(
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

@callback(
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