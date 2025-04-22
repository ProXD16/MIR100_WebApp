import dash
from dash import dcc, html
from dash.dependencies import Input, Output, State
import plotly.express as px
import pandas as pd
import numpy as np

# Dữ liệu mẫu ban đầu
df = pd.DataFrame({
    "x": [1, 2, 3, 4, 5],
    "y": [10, 15, 13, 17, 20],
    "label": ["A", "B", "C", "D", "E"]
})

# Tạo lưới vô hình để bắt click bất kỳ
x_range = np.linspace(0, 6, 50)  # Phủ toàn bộ trục x
y_range = np.linspace(9, 21, 50)  # Phủ toàn bộ trục y
invisible_df = pd.DataFrame([(x, y) for x in x_range for y in y_range], columns=["x", "y"])

# Tạo figure ban đầu
fig = px.scatter(df, x="x", y="y", text="label", title="Biểu đồ tương tác - Chấm điểm bất kỳ")
fig.update_traces(textposition="top center")
# Thêm lớp scatter vô hình
fig.add_scatter(
    x=invisible_df["x"],
    y=invisible_df["y"],
    mode="markers",
    marker=dict(size=1, opacity=0),  # Vô hình
    showlegend=False,
    hoverinfo="none"
)
fig.update_layout(
    clickmode="event",
    xaxis=dict(range=[0, 6]),
    yaxis=dict(range=[9, 21])
)

# Khởi tạo ứng dụng Dash
app = dash.Dash(__name__)

# Layout của ứng dụng
app.layout = html.Div([
    dcc.Graph(id="graph", figure=fig, style={"height": "600px"}),
    html.Div(id="output", children="Click trên biểu đồ để chấm điểm!"),
    html.Button("Xóa tất cả điểm", id="reset-button", n_clicks=0),
    dcc.Store(id="marked-points", data=[]),  # Lưu danh sách điểm được chấm
])

# Callback để cập nhật biểu đồ và thông tin
@app.callback(
    Output("graph", "figure"),
    Output("output", "children"),
    Output("marked-points", "data"),
    Input("graph", "clickData"),
    Input("reset-button", "n_clicks"),
    State("marked-points", "data"),
    prevent_initial_call=True
)
def update_figure(clickData, n_clicks, marked_points):
    # Khởi tạo figure từ dữ liệu gốc
    updated_fig = px.scatter(df, x="x", y="y", text="label", title="Biểu đồ tương tác - Chấm điểm bất kỳ")
    updated_fig.update_traces(textposition="top center")
    # Thêm lớp scatter vô hình
    updated_fig.add_scatter(
        x=invisible_df["x"],
        y=invisible_df["y"],
        mode="markers",
        marker=dict(size=1, opacity=0),
        showlegend=False,
        hoverinfo="none"
    )
    updated_fig.update_layout(
        clickmode="event",
        xaxis=dict(range=[0, 6]),
        yaxis=dict(range=[9, 21])
    )

    # Xử lý sự kiện
    ctx = dash.callback_context
    if ctx.triggered_id == "reset-button":
        # Xóa tất cả điểm
        marked_points = []
        output_text = "Đã xóa tất cả điểm! Click để chấm lại."
    elif ctx.triggered_id == "graph" and clickData:
        # Lấy tọa độ từ clickData
        point = clickData["points"][0]
        x = point["x"]
        y = point["y"]

        # Thêm điểm vào danh sách
        marked_points.append({"x": x, "y": y})

        # Hiển thị tất cả điểm đã chấm
        if marked_points:
            marked_df = pd.DataFrame(marked_points)
            # Thêm marker đỏ
            updated_fig.add_scatter(
                x=marked_df["x"],
                y=marked_df["y"],
                mode="markers",
                marker=dict(size=10, color="red"),
                name="Điểm chấm",
                showlegend=True
            )
            # Thêm annotation cho mỗi điểm với tọa độ
            for i, row in marked_df.iterrows():
                updated_fig.add_annotation(
                    x=row["x"],
                    y=row["y"],
                    text=f"({row['x']:.2f}, {row['y']:.2f})",
                    showarrow=False,
                    font=dict(size=10, color="black"),
                    xanchor="center",
                    yanchor="bottom",
                    yshift=5
                )
        # Cập nhật thông tin output
        output_text = f"Đã chấm điểm tại: ({x:.2f}, {y:.2f}) | Tổng điểm: {len(marked_points)}"
    else: 
        output_text = "Click trên biểu đồ để chấm điểm!"

    return updated_fig, output_text, marked_points

# Chạy ứng dụng
if __name__ == "__main__":
    app.run_server(debug=True)