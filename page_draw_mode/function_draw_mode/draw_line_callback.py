from dash import Output, Input, State, callback, callback_context, no_update
import plotly.graph_objects as go

@callback(
    Output("map-image-draw-mode", "figure"),
    Input("map-image-draw-mode", "clickData"),
    State("draw-line-mode", "data"),
    State("map-image-draw-mode", "figure"),
    prevent_initial_call=True,
)
def draw_line_manual(clickData, is_active, figure):
    if not is_active:
        return no_update

    ctx = callback_context
    if not ctx.triggered:
        return no_update
    x = clickData["points"][0]["x"]
    y = clickData["points"][0]["y"]
    figure["data"].append(
        go.Scatter(
            x=[x],
            y=[y],
            mode="markers",
            marker=dict(color="red", size=10),
            showlegend=False,
        )
    )
    return figure