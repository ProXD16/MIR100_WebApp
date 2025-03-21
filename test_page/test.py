# # import rospy
# # import tf2_ros
# # from nav_msgs.msg import Path, OccupancyGrid
# # from sensor_msgs.msg import LaserScan
# # from geometry_msgs.msg import PoseWithCovarianceStamped
# # from PIL import Image, ImageDraw
# # import math
# # import json
# # import numpy as np
# # import tf2_geometry_msgs 
# # from tf.transformations import euler_from_quaternion

# # # Constants
# # OUTPUT_IMAGE_PATH = "static/combined_image.png"
# # MAP_IMAGE_PATH = "static/map_image.png"
# # ROBOT_IMAGE_PATH = "static/robot_image.png"
# # PATH_IMAGE_PATH = "static/path_image.png"
# # F_SCAN_IMAGE_PATH = "static/f_scan_image.png"
# # B_SCAN_IMAGE_PATH = "static/b_scan_image.png"
# # LINE_IMAGE_PATH = "static/line_image.png"  
# # IMAGE_WIDTH = None
# # IMAGE_HEIGHT = None
# # MAP_ORIGIN_X = None
# # MAP_ORIGIN_Y = None
# # MAP_RESOLUTION = None
# # LIDAR_RANGE = 50
# # POINT_SIZE = 1
# # JSON_FILE_PATH = "save_lines/line_drawn.json"


# # # Function to convert world coordinates to image coordinates
# # def world_to_image(x, y):
# #     global IMAGE_WIDTH, IMAGE_HEIGHT, MAP_ORIGIN_X, MAP_ORIGIN_Y, MAP_RESOLUTION
# #     if MAP_ORIGIN_X is None or MAP_ORIGIN_Y is None or MAP_RESOLUTION is None or IMAGE_WIDTH is None or IMAGE_HEIGHT is None:
# #         rospy.logwarn("Map information not yet received. Cannot transform coordinates.")
# #         return None, None

# #     px = int((x - MAP_ORIGIN_X) / MAP_RESOLUTION)
# #     py = int(IMAGE_HEIGHT - (y - MAP_ORIGIN_Y) / MAP_RESOLUTION)
# #     return px, py


# # def process_lidar_data(msg, tf_buffer, frame_id):
# #     points = []
    
# #     for i in range(len(msg.ranges)):
# #         r = msg.ranges[i]
# #         angle = i * msg.angle_increment + msg.angle_min

# #         if msg.range_min < r < msg.range_max and r < LIDAR_RANGE:
# #             x = r * math.cos(angle)
# #             y = r * math.sin(angle)

# #             # Tạo point trong hệ tọa độ của Lidar
# #             point_stamped = geometry_msgs.msg.PointStamped()
# #             point_stamped.header.frame_id = frame_id
# #             point_stamped.header.stamp = rospy.Time(0)  # Lấy thời gian mới nhất
# #             point_stamped.point.x = x
# #             point_stamped.point.y = y
# #             point_stamped.point.z = 0  # Lidar thường không có chiều cao

# #             try:
# #                 if tf_buffer.can_transform("map", frame_id, rospy.Time(0)):
# #                     transform = tf_buffer.lookup_transform("map", frame_id, rospy.Time(0))

# #                     # Sử dụng đúng hàm từ tf2_geometry_msgs
# #                     transformed_point = tf2_geometry_msgs.do_transform_point(point_stamped, transform)
# #                     points.append((transformed_point.point.x, transformed_point.point.y))

# #                 else:
# #                     rospy.logwarn(f"Không thể transform {frame_id} sang map. Bỏ qua điểm này.")

# #             except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
# #                 rospy.logwarn(f"Lỗi TF: {e}")

# #     return points



# # def create_lidar_image(points):
# #     img = Image.new('RGBA', (IMAGE_WIDTH, IMAGE_HEIGHT), (0, 0, 0, 0))
# #     draw = ImageDraw.Draw(img)

# #     if not points:
# #         return img
# #     for point_x, point_y in points:
# #         px, py = world_to_image(point_x, point_y)

# #         if px is not None and py is not None:
# #             draw.ellipse((px - POINT_SIZE, py - POINT_SIZE, px + POINT_SIZE, py + POINT_SIZE), fill=(255, 0, 0))

# #     return img


# # def path_callback(msg):
# #     global IMAGE_WIDTH, IMAGE_HEIGHT
# #     try:
# #         path_points = [(pose.pose.position.x, pose.pose.position.y) for pose in msg.poses]
# #         img = Image.new("RGBA", (IMAGE_WIDTH, IMAGE_HEIGHT), (0, 0, 0, 0))
# #         draw = ImageDraw.Draw(img)

# #         for world_x, world_y in path_points:
# #             px, py = world_to_image(world_x, world_y)
# #             if px is not None and py is not None:
# #                 draw.ellipse((px - 0.5, py - 0.5, px + 0.5, py + 0.5), fill=(0, 255, 255))

# #         img.save(PATH_IMAGE_PATH)
# #         rospy.loginfo(f"Path image saved to {PATH_IMAGE_PATH}")

# #         combine_images()  # Gọi sau khi cập nhật ảnh đường đi

# #     except Exception as e:
# #         rospy.logerr(f"Error processing path data: {e}")


# # def pose_callback(msg, tf_buffer):
# #     global IMAGE_WIDTH, IMAGE_HEIGHT

# #     if IMAGE_WIDTH is None or IMAGE_HEIGHT is None:
# #         rospy.logwarn("Image dimensions not yet initialized. Skipping pose callback.")
# #         return

# #     try:
# #         # Lấy tọa độ robot từ `amcl_pose`
# #         x = msg.pose.pose.position.x
# #         y = msg.pose.pose.position.y
# #         quaternion = (
# #             msg.pose.pose.orientation.x,
# #             msg.pose.pose.orientation.y,
# #             msg.pose.pose.orientation.z,
# #             msg.pose.pose.orientation.w
# #         )
# #         _, _, yaw = euler_from_quaternion(quaternion)

# #         img = Image.new("RGBA", (IMAGE_WIDTH, IMAGE_HEIGHT), (0, 0, 0, 0))
# #         draw = ImageDraw.Draw(img)

# #         px, py = world_to_image(x, y)
# #         if px is None or py is None:
# #             rospy.logwarn(f"Could not convert robot pose ({x}, {y}) to image coordinates.  Map origin/resolution may be incorrect. px={px}, py={py}")
# #             return  # Bỏ qua nếu lỗi tọa độ

# #         rect_length = 0.8
# #         rect_width = 0.6
# #         tri_side = 0.3

# #         # Chuyển đổi hình chữ nhật thành tọa độ ảnh
# #         corners_world = [
# #             (x + rect_length / 2 * math.cos(yaw) - rect_width / 2 * math.sin(yaw),
# #              y + rect_length / 2 * math.sin(yaw) + rect_width / 2 * math.cos(yaw)),
# #             (x + rect_length / 2 * math.cos(yaw) + rect_width / 2 * math.sin(yaw),
# #              y + rect_length / 2 * math.sin(yaw) - rect_width / 2 * math.cos(yaw)),
# #             (x - rect_length / 2 * math.cos(yaw) + rect_width / 2 * math.sin(yaw),
# #              y - rect_length / 2 * math.sin(yaw) - rect_width / 2 * math.cos(yaw)),
# #             (x - rect_length / 2 * math.cos(yaw) - rect_width / 2 * math.sin(yaw),
# #              y - rect_length / 2 * math.sin(yaw) + rect_width / 2 * math.cos(yaw))
# #         ]
# #         corners_image = []
# #         all_corners_valid = True  # Flag to track if all corners are valid
# #         for corner in corners_world:
# #             px_c, py_c = world_to_image(corner[0], corner[1])
# #             if px_c is None or py_c is None:
# #                 rospy.logwarn(f"Could not convert rectangle corner ({corner[0]}, {corner[1]}) to image coordinates.")
# #                 all_corners_valid = False
# #                 break # Exit the loop if one corner is invalid
# #             corners_image.append((px_c, py_c))

# #         # Vẽ robot trên bản đồ
# #         if all_corners_valid:
# #             draw.polygon(corners_image, fill=(128, 128, 128, 102))
# #         else:
# #             rospy.logwarn("Not all rectangle corners could be converted; skipping rectangle drawing.")


# #         # Vẽ tam giác hướng robot
# #         triangle_points_world = [
# #             (x + tri_side * math.cos(yaw), y + tri_side * math.sin(yaw)),
# #             (x - tri_side / 2 * math.cos(yaw) + (tri_side * math.sqrt(3) / 2) * math.sin(yaw),
# #              y - tri_side / 2 * math.sin(yaw) - (tri_side * math.sqrt(3) / 2) * math.cos(yaw)),
# #             (x - tri_side / 2 * math.cos(yaw) - (tri_side * math.sqrt(3) / 2) * math.sin(yaw),
# #              y - tri_side / 2 * math.sin(yaw) + (tri_side * math.sqrt(3) / 2) * math.cos(yaw))
# #         ]

# #         triangle_points_image = []
# #         all_triangle_points_valid = True  # Flag for triangle point validity
# #         for point in triangle_points_world:
# #             px_t, py_t = world_to_image(point[0], point[1])
# #             if px_t is None or py_t is None:
# #                 rospy.logwarn(f"Could not convert triangle point ({point[0]}, {point[1]}) to image coordinates.")
# #                 all_triangle_points_valid = False
# #                 break
# #             triangle_points_image.append((px_t, py_t))

# #         if all_triangle_points_valid:
# #             draw.polygon(triangle_points_image, fill=(0, 0, 255, 178))
# #         else:
# #             rospy.logwarn("Not all triangle points could be converted; skipping triangle drawing.")



# #         # Lưu ảnh robot
# #         img.save(ROBOT_IMAGE_PATH)
# #         rospy.loginfo(f"Robot location image updated: {ROBOT_IMAGE_PATH}")
# #         combine_images()

# #     except Exception as e:
# #         rospy.logerr(f"Error updating robot pose image: {e}")


# # def map_info_callback(map_data):
# #     global MAP_ORIGIN_X, MAP_ORIGIN_Y, MAP_RESOLUTION, IMAGE_WIDTH, IMAGE_HEIGHT

# #     MAP_ORIGIN_X = map_data.info.origin.position.x
# #     MAP_ORIGIN_Y = map_data.info.origin.position.y
# #     MAP_RESOLUTION = map_data.info.resolution
# #     IMAGE_WIDTH = map_data.info.width
# #     IMAGE_HEIGHT = map_data.info.height
# #     rospy.loginfo("Received map info")

# #     combine_images()  # Gọi sau khi nhận thông tin bản đồ mới


# # def lidar_callback(msg, tf_buffer, topic_name):
# #     try:
# #         frame_id = "back_laser_link" if topic_name == "/b_scan" else "front_laser_link"
# #         points = process_lidar_data(msg, tf_buffer, frame_id)

# #         if points:
# #             img_output = create_lidar_image(points)
# #             output_path = f"static/{topic_name.split('/')[-1]}_image.png"
# #             img_output.save(output_path)
# #             rospy.loginfo(f"Lidar image for {topic_name} saved to {output_path}")

# #             combine_images()  # Gọi sau khi cập nhật ảnh Lidar

# #         else:
# #             rospy.logwarn(f"No valid Lidar points for {topic_name}, skipping image save.")

# #     except Exception as e:
# #         rospy.logerr(f"Error processing {topic_name} data: {e}")



# # def load_lines_from_json():
# #     """Loads line data from the specified JSON file."""
# #     try:
# #         with open(JSON_FILE_PATH, 'r') as f:
# #             data = json.load(f)
# #             rospy.loginfo("Loaded line data from JSON.")
# #             return data
# #     except FileNotFoundError:
# #         rospy.logwarn(f"JSON file not found: {JSON_FILE_PATH}")
# #         return None
# #     except json.JSONDecodeError:
# #         rospy.logerr(f"Error decoding JSON from {JSON_FILE_PATH}")
# #         return None
# #     except Exception as e:
# #         rospy.logerr(f"Error loading line data: {e}")
# #         return None


# # def create_line_image(lines_data):
# #     """Creates an image with the lines drawn on it."""
# #     global IMAGE_WIDTH, IMAGE_HEIGHT

# #     img = Image.new("RGBA", (IMAGE_WIDTH, IMAGE_HEIGHT), (0, 0, 0, 0))
# #     draw = ImageDraw.Draw(img)

# #     if lines_data is None:
# #         return img

# #     try:
# #         for item in lines_data:
# #             if isinstance(item, dict):
# #                 if "type" in item:
# #                     if item["type"] == "line":
# #                         x_coords = item["x"]
# #                         y_coords = item["y"]

# #                         # Convert world coordinates to image coordinates
# #                         image_coords = []
# #                         for i in range(len(x_coords)):
# #                             px, py = world_to_image(x_coords[i], y_coords[i])
# #                             if px is not None and py is not None:
# #                                 image_coords.append((px, py))
# #                             else:
# #                                 rospy.logwarn("Invalid world coordinates, skipping point.")

# #                         # Draw the line if there are enough valid image coordinates
# #                         if len(image_coords) >= 2:
# #                             draw.line(image_coords, fill="purple", width=2)
# #                         else:
# #                             rospy.logwarn("Not enough valid points to draw a line.")


# #                     elif item["type"] == "arc":
# #                         n_points = 50
# #                         start_angle = item["start_angle"]
# #                         end_angle = item["end_angle"]
# #                         center_x = item["center_x"]
# #                         center_y = item["center_y"]
# #                         radius = item["radius"]

# #                         angles = np.linspace(start_angle, end_angle, n_points)
# #                         x = center_x + radius * np.cos(angles)
# #                         y = center_y + radius * np.sin(angles)

# #                         # Convert world coordinates to image coordinates
# #                         image_coords = []
# #                         for i in range(n_points):
# #                             px, py = world_to_image(x[i], y[i])
# #                             if px is not None and py is not None:
# #                                 image_coords.append((px, py))
# #                             else:
# #                                 rospy.logwarn("Invalid world coordinates for arc, skipping point.")

# #                         # Draw the arc if there are enough valid image coordinates
# #                         if len(image_coords) >= 2:
# #                             draw.line(image_coords, fill="purple", width=2)
# #                         else:
# #                             rospy.logwarn("Not enough valid points to draw the arc.")
# #                     else:
# #                         rospy.logwarn(f"Unknown type: {item['type']} in item: {item}")
# #                 else:
# #                     rospy.logwarn(f"'type' key missing in item: {item}")
# #             else:
# #                 rospy.logwarn(f"Expected a dictionary, but got: {type(item)}, value: {item}")

# #         img.save(LINE_IMAGE_PATH)
# #         rospy.loginfo(f"Line image saved to {LINE_IMAGE_PATH}")

# #     except Exception as e:
# #         rospy.logerr(f"Error drawing lines: {e}")

# #     return img


# # def combine_images():
# #     """Combines the map, robot, path, lidar and line images into one."""
# #     try:
# #         # Load the images
# #         map_img = Image.open(MAP_IMAGE_PATH).convert("RGBA")
# #         robot_img = Image.open(ROBOT_IMAGE_PATH).convert("RGBA")
# #         path_img = Image.open(PATH_IMAGE_PATH).convert("RGBA")
# #         f_scan_img = Image.open(F_SCAN_IMAGE_PATH).convert("RGBA")
# #         b_scan_img = Image.open(B_SCAN_IMAGE_PATH).convert("RGBA")

# #         # Load and create the line image
# #         lines_data = load_lines_from_json()
# #         line_img = create_line_image(lines_data)

# #         # Paste the images onto the map image
# #         map_img.paste(path_img, (0, 0), path_img)
# #         map_img.paste(robot_img, (0, 0), robot_img)
# #         map_img.paste(f_scan_img, (0, 0), f_scan_img)
# #         map_img.paste(b_scan_img, (0, 0), b_scan_img)
# #         map_img.paste(line_img, (0, 0), line_img)  # Paste the line image

# #         # Save the combined image
# #         map_img.save(OUTPUT_IMAGE_PATH)
# #         rospy.loginfo(f"Combined image saved to {OUTPUT_IMAGE_PATH}")

# #     except FileNotFoundError as e:
# #         rospy.logerr(f"One or more image files not found: {e}")
# #     except Exception as e:
# #         rospy.logerr(f"Error combining images: {e}")


# # def main():
# #     """Main function."""
# #     rospy.init_node('image_combiner_node', anonymous=True)
# #     tf_buffer = tf2_ros.Buffer(cache_time=rospy.Duration(5.0))  # Tăng cache TF
# #     tf_listener = tf2_ros.TransformListener(tf_buffer)
# #     # Subscribers
# #     rospy.Subscriber("/map", OccupancyGrid, map_info_callback)
# #     rospy.Subscriber("/move_base_node/SBPLLatticePlanner/plan", Path, path_callback)
# #     rospy.Subscriber("/f_scan", LaserScan, lambda msg: lidar_callback(msg, tf_buffer, "/f_scan"))
# #     rospy.Subscriber("/b_scan", LaserScan, lambda msg: lidar_callback(msg, tf_buffer, "/b_scan"))
# #     rospy.Subscriber("/amcl_pose", PoseWithCovarianceStamped, pose_callback, tf_buffer)

# #     rate = rospy.Rate(1)

# #     while not rospy.is_shutdown():
# #         combine_images()
# #         rate.sleep()


# # if __name__ == '__main__':
# #     try:
# #         import geometry_msgs.msg

# #         main()
# #     except rospy.ROSInterruptException:
# #         pass

# from dash import html, dcc, callback, Input, Output, State, no_update
# import dash_bootstrap_components as dbc
# import rospy
# from geometry_msgs.msg import PoseStamped, Twist, PoseWithCovarianceStamped
# import numpy as np
# import math
# from PIL import Image
# import plotly.express as px
# import dash
# import io
# import base64
# import json
# import os
# from utils.move_along_path_line import *

# is_moving = True

# class RVizSection:
#     FIXED_ARROW_LENGTH = 30

#     def __init__(self, goal_topic="/move_base_simple/goal", twist_topic="/cmd_vel", pose_topic="/amcl_pose"):
#         self.goal_topic = goal_topic
#         self.twist_topic = twist_topic
#         self.pose_topic = pose_topic
#         try:
#             self.goal_pub = rospy.Publisher(self.goal_topic, PoseStamped, queue_size=10)
#             self.twist_pub = rospy.Publisher(self.twist_topic, Twist, queue_size=10)
#             self.current_pose = None
#             self.pose_sub = rospy.Subscriber(self.pose_topic, PoseWithCovarianceStamped, self.pose_callback)
#         except rospy.exceptions.ROSException as e:
#             print(f"Error connecting to ROS: {e}")
#             self.goal_pub = None
#             self.twist_pub = None
#         self.map_data = self.load_image_as_numpy("static/map_image.png")

#     def pose_callback(self, msg):
#         self.current_pose = msg.pose.pose

#     def load_image_as_numpy(self, image_path):
#         try:
#             img = Image.open(image_path).convert('L')
#             map_data = np.array(img)
#             if len(map_data.shape) > 2:
#                 print("WARNING: Image has more than two dimensions, potential color issue")
#             return map_data
#         except FileNotFoundError:
#             print(f"Error: Image file not found at {image_path}")
#             return self.generate_fake_map()

#     def generate_fake_map(self):
#         size = 500
#         return np.random.randint(0, 255, size=(size, size))

#     def create_composite_image(self):
#         width, height = 800, 600  # Kích thước ảnh, có thể thay đổi theo nhu cầu
#         final_img = Image.new("RGBA", (width, height), (255, 255, 255, 0))  # Ảnh nền trắng, trong suốt
#         return final_img


#     def create_figure(self, drawing_enabled=False, start_x=None, start_y=None, end_x=None, end_y=None):
#         composite_img = self.create_composite_image()
#         if composite_img is None:
#             fake_map = self.generate_fake_map()
#             fig = px.imshow(fake_map, color_continuous_scale='gray')
#         else:
#             img_byte_arr = io.BytesIO()
#             composite_img.save(img_byte_arr, format='PNG')
#             img_byte_arr = img_byte_arr.getvalue()
#             encoded_image = base64.b64encode(img_byte_arr).decode()
#             img_data = f'data:image/png;base64,{encoded_image}'
#             fig = px.imshow(Image.open(io.BytesIO(base64.b64decode(encoded_image))), binary_string=True)
#             fig.update_layout(
#                 images=[dict(
#                     source=img_data,
#                     xref="x", yref="y",
#                     x=0, y=composite_img.size[1],
#                     sizex=composite_img.size[0], sizey=composite_img.size[1],
#                     sizing="stretch",
#                     opacity=1,
#                     layer="below")],
#                 plot_bgcolor='rgba(255,255,255,0)',
#                 paper_bgcolor='rgba(255,255,255,0)'
#             )

#         dragmode = "drawline" if drawing_enabled else False
#         fig.update_layout(
#             dragmode=dragmode,
#             xaxis=dict(showgrid=False, zeroline=False, visible=False),
#             yaxis=dict(showgrid=False, zeroline=False, visible=False, scaleratio=1),
#             margin=dict(l=0, r=0, b=0, t=0),
#             height=600,
#             width=800,
#             newshape_line_color='red',
#             coloraxis_showscale=False
#         )
#         fig.update_traces(
#             hovertemplate=None,
#             hoverinfo='skip'
#         )

#         if start_x is not None and start_y is not None and end_x is not None and end_y is not None:
#             dx = end_x - start_x
#             dy = end_y - start_y
#             angle = math.atan2(dy, dx)
#             fixed_end_x = start_x + self.FIXED_ARROW_LENGTH * math.cos(angle)
#             fixed_end_y = start_y + self.FIXED_ARROW_LENGTH * math.sin(angle)
#             fig.add_annotation(
#                 x=fixed_end_x, y=fixed_end_y, ax=start_x, ay=start_y,
#                 xref="x", yref="y", axref="x", ayref="y",
#                 arrowhead=2, arrowcolor='red', arrowwidth=2,
#             )
#         return fig

#     def publish_goal(self, x, y, angle):
#         if self.goal_pub:
#             try:
#                 pose = PoseStamped()
#                 pose.header.stamp = rospy.Time.now()
#                 pose.header.frame_id = "map"
#                 pose.pose.position.x = float(x)
#                 pose.pose.position.y = float(y)
#                 pose.pose.orientation.z = math.sin(angle / 2)
#                 pose.pose.orientation.w = math.cos(angle / 2)
#                 self.goal_pub.publish(pose)
#                 return "Goal published successfully! (x: {}, y: {}, angle: {})".format(x, y, angle)
#             except Exception as e:
#                 return f"Error publishing goal: {e}"
#         return "Goal publisher not initialized."

#     def publish_twist(self, linear_x=0.0, angular_z=0.0):
#         if self.twist_pub:
#             try:
#                 twist = Twist()
#                 twist.linear.x = float(linear_x)
#                 twist.angular.z = float(angular_z)
#                 self.twist_pub.publish(twist)
#                 return f"Twist published: linear_x={linear_x}, angular_z={angular_z}"
#             except Exception as e:
#                 return f"Error publishing twist: {e}"
#         return "Twist publisher not initialized."

#     def stop_robot(self):
#         return self.publish_twist(0.0, 0.0)

#     def is_at_position(self, target_x, target_y, tolerance=0.2):
#         if self.current_pose is None:
#             return False
#         current_x = self.current_pose.position.x
#         current_y = self.current_pose.position.y
#         distance = math.sqrt((current_x - target_x)**2 + (current_y - target_y)**2)
#         return distance < tolerance

# def create_rviz_section():
#     layout = html.Div(
#         [
#             html.H3("RViz Interface", className="mb-3", style={"color": "#2C3E50"}),
#             dbc.Row(
#                 [
#                     dbc.Col(
#                         html.Div(
#                             [
#                                 html.Button("Send Goal", id="send-goal-btn", className="btn btn-primary me-2"),
#                                 html.Button("2D Nav Goal", id="nav-goal-btn", className="btn btn-secondary me-2"),
#                                 html.Button("Move Along Line", id="move-line-btn", className="btn btn-primary me-2"),
#                                 html.Button("Move Along Arc", id="move-arc-btn", className="btn btn-primary me-2"),
#                                 html.Button("Move Along Path", id="move-path-btn", className="btn btn-primary me-2"),
#                                 html.Button("Emergency Stop", id="emergency-stop-btn", className="btn btn-danger"),
#                             ],
#                             className="mb-3"
#                         ),
#                         width=12,
#                     ),
#                 ]
#             ),
#             html.Div(
#                 [
#                     dcc.Graph(
#                         id="map-graph",
#                         figure=RVizSection().create_figure(),
#                         style={
#                             "width": "800px",
#                             "height": "600px",
#                             "background": "rgba(0, 0, 0, 0)",
#                             "position": "absolute",
#                             "z-index": "7",
#                             "top": "0",
#                             "left": "0",
#                         },
#                         config={'scrollZoom': False, 'displayModeBar': False}
#                     ),
#                     html.Img(
#                         id="map-image",
#                         src="/static/map_image.png",
#                         style={
#                             "width": "800px",
#                             "height": "600px",
#                             "border": "2px solid #34495E",
#                             "object-fit": "contain",
#                             "position": "absolute",
#                             "z-index": "1",
#                             "top": "0",
#                             "left": "0",
#                         },
#                     ),
#                     html.Img(
#                         id="lidar-f-image",
#                         src="/static/f_scan_image.png",
#                         style={
#                             "width": "800px",
#                             "height": "600px",
#                             "border": "2px solid #34495E",
#                             "object-fit": "contain",
#                             "position": "absolute",
#                             "z-index": "2",
#                             "top": "0",
#                             "left": "0",
#                         },
#                     ),
#                     html.Img(
#                         id="lidar-b-image",
#                         src="/static/lidar_b_image.png",
#                         style={
#                             "width": "800px",
#                             "height": "600px",
#                             "border": "2px solid #34495E",
#                             "object-fit": "contain",
#                             "position": "absolute",
#                             "z-index": "3",
#                             "top": "0",
#                             "left": "0",
#                         },
#                     ),
#                     html.Img(
#                         id="path-image",
#                         src="/static/path_image.png",
#                         style={
#                             "width": "800px",
#                             "height": "600px",
#                             "border": "2px solid #34495E",
#                             "object-fit": "contain",
#                             "position": "absolute",
#                             "z-index": "4",
#                             "top": "0",
#                             "left": "0",
#                         },
#                     ),
#                     html.Img(
#                         id="robot-image",
#                         src="/static/robot_image.png",
#                         style={
#                             "width": "800px",
#                             "height": "600px",
#                             "border": "2px solid #34495E",
#                             "object-fit": "contain",
#                             "position": "absolute",
#                             "z-index": "5",
#                             "top": "0",
#                             "left": "0",
#                         },
#                     ),
#                     html.Img(
#                         id="lines-image",
#                         src="/static/line_image.png",
#                         style={
#                             "width": "800px",
#                             "height": "600px",
#                             "border": "2px solid #34495E",
#                             "object-fit": "contain",
#                             "position": "absolute",
#                             "z-index": "6",
#                             "top": "0",
#                             "left": "0",
#                         },
#                     ),
#                 ],
#                 style={"position": "relative", "width": "800px", "height": "600px"},
#             ),
#             html.P("Draw line and after release, it sends the goal", className="text-info mt-2"),
#             html.Div(id="goal-status"),
#             dcc.Interval(
#                 id='interval-component',
#                 interval=1 * 1000,
#                 n_intervals=0
#             ),
#             dcc.Interval(
#                 id='move-line-interval',
#                 interval=1 * 1000,
#                 n_intervals=0,
#                 disabled=True
#             ),
#             dcc.Store(id="drag-start-coords", data=None),
#             dcc.Store(id="drawing-enabled", data=False),
#             dcc.Store(id="latest-goal", data=None),
#             dcc.Store(id="move-line-data", data=None),
#             dcc.Store(id="path-line-data", data=[]), 
#             dcc.Store(id="current-line-index", data=0),  
#             dbc.Modal(
#                 [
#                     dbc.ModalHeader(dbc.ModalTitle("Enter Goal Coordinates")),
#                     dbc.ModalBody(
#                         dbc.Form(
#                             [
#                                 dbc.Row(
#                                     [
#                                         dbc.Col(dbc.Label("X:", html_for="goal-x")),
#                                         dbc.Col(dbc.Input(type="number", id="goal-x", placeholder="X Coordinate")),
#                                     ],
#                                     className="mb-3",
#                                 ),
#                                 dbc.Row(
#                                     [
#                                         dbc.Col(dbc.Label("Y:", html_for="goal-y")),
#                                         dbc.Col(dbc.Input(type="number", id="goal-y", placeholder="Y Coordinate")),
#                                     ],
#                                     className="mb-3",
#                                 ),
#                                 dbc.Row(
#                                     [
#                                         dbc.Col(dbc.Label("Z:", html_for="goal-z")),
#                                         dbc.Col(dbc.Input(type="number", id="goal-z", placeholder="Z Coordinate", value=0)),
#                                     ],
#                                     className="mb-3",
#                                 ),
#                                 dbc.Row(
#                                     [
#                                         dbc.Col(dbc.Label("W:", html_for="goal-w")),
#                                         dbc.Col(dbc.Input(type="number", id="goal-w", placeholder="W Orientation", value=1)),
#                                     ],
#                                     className="mb-3",
#                                 ),
#                             ]
#                         )
#                     ),
#                     dbc.ModalFooter(
#                         [
#                             dbc.Button("Close", id="close-goal-modal", className="ms-auto"),
#                             dbc.Button("Send", id="send-goal-modal-btn", color="primary", className="ms-2"),
#                         ]
#                     ),
#                 ],
#                 id="goal-modal",
#                 is_open=False,
#             ),
#             dbc.Modal(
#                 [
#                     dbc.ModalHeader(dbc.ModalTitle("Set Movement Speed")),
#                     dbc.ModalBody(
#                         dbc.Form(
#                             [
#                                 dbc.Row(
#                                     [
#                                         dbc.Col(dbc.Label("Linear Speed (m/s):", html_for="linear-speed")),
#                                         dbc.Col(dbc.Input(type="number", id="linear-speed", placeholder="Enter speed", value=0.2, step=0.1)),
#                                     ],
#                                     className="mb-3",
#                                 ),
#                                 dbc.Row(
#                                     [
#                                         dbc.Col(dbc.Label("Angular Speed (rad/s):", html_for="angular-speed")),
#                                         dbc.Col(dbc.Input(type="number", id="angular-speed", placeholder="Enter angular speed", value=0.5, step=0.1)),
#                                     ],
#                                     className="mb-3",
#                                 ),
#                             ]
#                         )
#                     ),
#                     dbc.ModalFooter(
#                         [
#                             dbc.Button("Cancel", id="cancel-line-modal", className="ms-auto"),
#                             dbc.Button("Start", id="start-line-modal-btn", color="primary", className="ms-2"),
#                         ]
#                     ),
#                 ],
#                 id="line-modal",
#                 is_open=False,
#             ),
#         ],
#         style={
#             "padding": "20px",
#             "flex": "1",
#             "background": "#ECF0F1",
#             "marginLeft": "250px",
#             "marginTop": "50px",
#         },
#     )
#     return layout

# def load_map_info():
#     map_info_path = "static/map_image.json"
#     if os.path.exists(map_info_path):
#         with open(map_info_path, "r") as f:
#             return json.load(f)
#     else:
#         print("WARNING: map_info.json not found. Using default values or disabling auto-goal sending.")
#         return None

# # Callbacks
# @callback(
#     Output("goal-modal", "is_open"),
#     [Input("send-goal-btn", "n_clicks"),
#      Input("close-goal-modal", "n_clicks"),
#      Input("send-goal-modal-btn", "n_clicks")],
#     [State("goal-modal", "is_open")],
#     prevent_initial_call=True,
# )
# def toggle_modal(n1, n2, n3, is_open):
#     ctx = dash.callback_context
#     if not ctx.triggered:
#         return False
#     button_id = ctx.triggered[0]['prop_id'].split('.')[0]
#     if button_id == "send-goal-btn":
#         return True
#     elif button_id == "close-goal-modal" or button_id == "send-goal-modal-btn":
#         return False
#     return is_open

# @callback(
#     Output("goal-status", "children"),
#     [Input("send-goal-modal-btn", "n_clicks")],
#     [
#         State("goal-x", "value"),
#         State("goal-y", "value"),
#         State("goal-z", "value"),
#         State("goal-w", "value"),
#     ],
#     prevent_initial_call=True,
# )
# def send_goal_coordinates(n_clicks, x, y, z, w):
#     if n_clicks:
#         rviz_section = RVizSection()
#         angle = math.atan2(float(y), float(x))
#         status = rviz_section.publish_goal(x, y, angle)
#         return status
#     return no_update

# @callback(
#     Output("map-graph", "figure"),
#     [Input("map-graph", "clickData"), Input("map-graph", "relayoutData"), Input("interval-component", "n_intervals")],
#     [State("map-graph", "figure"), State("drag-start-coords", "data"), State("drawing-enabled", "data")],
#     prevent_initial_call=True,
# )
# def update_map(clickData, relayoutData, n_intervals, existing_map, drag_start_coords, drawing_enabled):
#     ctx = dash.callback_context
#     rviz_section = RVizSection()
#     if not ctx.triggered:
#         return existing_map
#     trigger_id = ctx.triggered[0]['prop_id'].split('.')[0]

#     if trigger_id == "map-graph" and clickData and drawing_enabled:
#         start_x = clickData['points'][0]['x']
#         start_y = clickData['points'][0]['y']
#         return rviz_section.create_figure(drawing_enabled, start_x=start_x, start_y=start_y, end_x=start_x, end_y=start_y)

#     elif trigger_id == "map-graph" and relayoutData and drag_start_coords and drawing_enabled:
#         start_x = drag_start_coords["start_x"]
#         start_y = drag_start_coords["start_y"]
#         end_x = relayoutData.get("xaxis.range[0]")
#         end_y = relayoutData.get("yaxis.range[1]")
#         return rviz_section.create_figure(drawing_enabled, start_x=start_x, start_y=start_y, end_x=end_x, end_y=end_y)

#     elif trigger_id == "interval-component":
#         return rviz_section.create_figure(drawing_enabled)

#     return existing_map

# @callback(
#     Output("drag-start-coords", "data"),
#     [Input("map-graph", "clickData")],
#     [State("drawing-enabled", "data")],
#     prevent_initial_call=True
# )
# def store_drag_start_coords(clickData, drawing_enabled):
#     if clickData and drawing_enabled:
#         return {"start_x": clickData['points'][0]['x'], "start_y": clickData['points'][0]['y']}
#     return no_update

# @callback(
#     [Output("drawing-enabled", "data"), Output("nav-goal-btn", "className")],
#     [Input("nav-goal-btn", "n_clicks")],
#     [State("drawing-enabled", "data")],
#     prevent_initial_call=True
# )
# def toggle_drawing_mode(n_clicks, drawing_enabled):
#     if n_clicks:
#         drawing_enabled = not drawing_enabled
#         button_class = "btn btn-success" if drawing_enabled else "btn btn-secondary"
#         return drawing_enabled, button_class
#     return drawing_enabled, "btn btn-secondary"

# @callback(
#     [Output("goal-status", "children", allow_duplicate=True),
#      Output("drawing-enabled", "data", allow_duplicate=True),
#      Output("nav-goal-btn", "className", allow_duplicate=True)],
#     Input("map-graph", "relayoutData"),
#     [State("drag-start-coords", "data"), State("drawing-enabled", "data")],
#     prevent_initial_call=True
# )
# def auto_send_goal(relayoutData, drag_start_coords, drawing_enabled):
#     if not (relayoutData and drawing_enabled):
#         print("Không thỏa mãn điều kiện gửi goal")
#         return no_update, no_update, no_update
#     map_info = load_map_info()
#     if not map_info:
#         print("Không thể tải thông tin bản đồ từ map_info.json. Không tự động gửi goal.")
#         return "Không thể gửi goal tự động do thiếu thông tin bản đồ.", no_update, no_update
#     if "shapes" not in relayoutData or not relayoutData["shapes"]:
#         print("relayoutData không có shapes hợp lệ!")
#         return no_update, no_update, no_update

#     shape = relayoutData["shapes"][0]
#     x_pixel_start, y_pixel_start = shape["x0"], shape["y0"]
#     x_pixel_end, y_pixel_end = shape["x1"], shape["y1"]

#     start_x = map_info["origin_x"] + (x_pixel_start * map_info["resolution"])
#     start_y = map_info["origin_y"] + ((map_info["height"] - y_pixel_start) * map_info["resolution"])
#     end_x = map_info["origin_x"] + (x_pixel_end * map_info["resolution"])
#     end_y = map_info["origin_y"] + ((map_info["height"] - y_pixel_end) * map_info["resolution"])

#     dx = end_x - start_x
#     dy = end_y - start_y
#     angle = math.atan2(dy, dx)

#     rviz_section = RVizSection()
#     status = rviz_section.publish_goal(start_x, start_y, angle)

#     drawing_enabled = False
#     button_class = "btn btn-secondary"
#     return status, drawing_enabled, button_class

# @callback(
#     Output("line-modal", "is_open"),
#     [Input("move-line-btn", "n_clicks"), Input("cancel-line-modal", "n_clicks"), Input("start-line-modal-btn", "n_clicks")],
#     [State("line-modal", "is_open")],
#     prevent_initial_call=True,
# )
# def toggle_line_modal(n1, n2, n3, is_open):
#     ctx = dash.callback_context
#     if not ctx.triggered:
#         return False
#     button_id = ctx.triggered[0]['prop_id'].split('.')[0]
#     if button_id == "move-line-btn":
#         return True
#     elif button_id in ["cancel-line-modal", "start-line-modal-btn"]:
#         return False
#     return is_open

# @callback(
#     [Output("goal-status", "children", allow_duplicate=True),
#      Output("move-line-data", "data", allow_duplicate=True),
#      Output("path-line-data", "data", allow_duplicate=True),
#      Output("current-line-index", "data", allow_duplicate=True),
#      Output("move-line-interval", "disabled", allow_duplicate=True)],
#     Input("start-line-modal-btn", "n_clicks"),
#     [State("linear-speed", "value"),
#      State("angular-speed", "value")],
#     prevent_initial_call=True,
# )
# def initiate_move_along_path(n_clicks, linear_speed, angular_speed):
#     global is_moving
#     if not n_clicks:
#         return no_update, no_update, no_update, no_update, no_update

#     is_moving = True  # Cho phép di chuyển lại sau Emergency Stop
#     print("DEBUG: Bắt đầu Move Along Line")

#     file_path = "/home/duc/Downloads/App_MIR100/save_lines/line_drawn.json"
#     try:
#         with open(file_path, 'r') as f:
#             path_line_data = json.load(f)

#         if not path_line_data:
#             return "Không tìm thấy dữ liệu đường đi", no_update, no_update, no_update, True

#         for line in path_line_data:
#             start_x, start_y = line['x'][0], line['y'][0]
#             end_x, end_y = line['x'][1], line['y'][1]
#             rotate_to_target(start_x, start_y, angular_speed)
#             move_to_target(start_x, start_y, linear_speed)
#             rotate_to_target(end_x, end_y, angular_speed)
#             move_to_target(end_x, end_y, linear_speed)

#         is_moving = False
#         return "Hoàn thành di chuyển theo đường", no_update, no_update, no_update, True

#     except Exception as e:
#         return f"Lỗi khi đọc dữ liệu hoặc khởi động di chuyển: {e}", no_update, no_update, no_update, True


# # @callback(
# #     [Output("goal-status", "children", allow_duplicate=True),
# #      Output("current-line-index", "data", allow_duplicate=True),
# #      Output("move-line-interval", "disabled", allow_duplicate=True)],
# #     Input("move-line-interval", "n_intervals"),
# #     [State("move-line-data", "data"),
# #      State("path-line-data", "data"),
# #      State("current-line-index", "data")],
# #     prevent_initial_call=True,
# # )
# # def move_along_path(n_intervals, move_data, path_line_data, current_line_index):
# #     if not move_data or not path_line_data:
# #         return no_update, no_update, no_update

# #     rviz_section = RVizSection()
# #     linear_speed = move_data["linear_speed"]
# #     angular_speed = move_data["angular_speed"]

# #     if current_line_index >= len(path_line_data):
# #         rviz_section.stop_robot()
# #         return "Reached the end of the path.", no_update, True

# #     current_line = path_line_data[current_line_index]
# #     line_id, start_x, start_y, end_x, end_y = current_line

# #     # Check if robot is at the start point of the current line
# #     if not rviz_section.is_at_position(start_x, start_y, tolerance=0.1):
# #         rospy.loginfo("Rotate to the start point")
# #         rotate_along_start_point(start_x, start_y, angular_speed)
# #         return f"Moving to start point {start_x}, {start_y}...", no_update, False

# #     # Now that the robot is at the start point, follow the straight line to the end
# #     rospy.loginfo("Follow straight line")
# #     follow_straight_line(end_x, end_y, linear_speed)
# #     next_line_index = current_line_index + 1

# #     # If there are more lines, move to the next line
# #     if next_line_index < len(path_line_data):
# #         next_line = path_line_data[next_line_index]
# #         next_start_x, next_start_y, next_end_x, next_end_y = next_line[1], next_line[2], next_line[3], next_line[4]
# #         dx = next_end_x - next_start_x
# #         dy = next_end_y - next_start_y
# #         angle = math.atan2(dy, dx)
# #         status = rviz_section.publish_goal(next_start_x, next_start_y, angle)
# #         return f"Reached line {current_line_index + 1}, {status}\nMoving to start point {next_start_x}, {next_start_y}...", next_line_index, False
# #     else:
# #         rviz_section.stop_robot()
# #         return "Reached the end of the path.", no_update, True


# # @callback(
# #     Output("goal-status", "children"),
# #     Input("emergency-stop-btn", "n_clicks")
# # )
# # def emergency_stop(n_clicks):
# #     if not n_clicks:
# #         return no_update

# #     rviz_section = RVizSection()
# #     status = rviz_section.stop_robot()
# #     return f"Emergency Stop: {status}"


# # @callback(
# #     Output("map-image", "src"),
# #     Input("interval-component", "n_intervals")
# # )
# # def update_map_image(n):
# #     timestamp = int(time.time())
# #     return f"/static/map_image.png?{timestamp}"

# # @callback(
# #     [Output("lidar-f-image", "src"), Output("lidar-b-image", "src")],
# #     Input("interval-component", "n_intervals")
# # )
# # def update_lidar_images(n):
# #     timestamp = int(time.time())
# #     return (
# #         f"/static/f_scan_image.png?{timestamp}",
# #         f"/static/b_scan_image.png?{timestamp}"
# #     )

# # @callback(
# #     Output("path-image", "src"),
# #     Input("interval-component", "n_intervals")
# # )
# # def update_path_image(n):
# #     timestamp = int(time.time())
# #     return f"/static/path_image.png?{timestamp}"

# # @callback(
# #     Output("robot-image", "src"),
# #     Input("interval-component", "n_intervals")
# # )
# # def update_robot_image(n):
# #     timestamp = int(time.time())
# #     return f"/static/robot_image.png?{timestamp}"

# # @callback(
# #     Output("lines-image", "src"),
# #     Input("interval-component", "n_intervals")
# # )
# # def update_lines_image(n):
# #     timestamp = int(time.time())
# #     return f"/static/line_image.png?{timestamp}"

from PIL import Image, ImageDraw

# Kích thước ảnh
WIDTH, HEIGHT = 200, 200

# Tạo ảnh mới
img = Image.new("RGBA", (WIDTH, HEIGHT), (255, 255, 255, 0))
draw = ImageDraw.Draw(img)

# Vẽ hai phần nền
draw.rectangle([(0, 0), (WIDTH//2, HEIGHT)], fill=(100, 100, 100))  # Màu xám bên trái
draw.rectangle([(WIDTH//2, 0), (WIDTH, HEIGHT)], fill=(30, 30, 30))  # Màu đen bên phải

# Vẽ hai tam giác đen (mũi tên bên trái)
triangle1 = [(40, 60), (60, 50), (40, 40)]
triangle2 = [(40, 140), (60, 130), (40, 120)]
draw.polygon(triangle1, fill=(50, 50, 50))
draw.polygon(triangle2, fill=(50, 50, 50))

# Vẽ tia sét màu vàng
lightning = [(120, 50), (140, 80), (130, 80), (150, 120), (130, 120), (145, 150)]
draw.polygon(lightning, fill="yellow", outline="black")

# Vẽ đường dây đỏ
# red_wire = [
#     (150, 10), (150, 30), (140, 40), (150, 60), (140, 70),
#     (150, 90), (140, 100), (150, 120), (140, 130), (150, 150)
# ]
# draw.line(red_wire, fill="red", width=5)

# Lưu ảnh
img.save("static/dockers.png")
img.show()  # Hiển thị ảnh nếu chạy trên máy tính

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
from callbacks.image_components_callbacks import *
from callbacks.navigation_callbacks import *
from callbacks.mission_callbacks import *
from callbacks.ui_callbacks import *

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

# @app.callback(
#     Output("app-container", "children"),
#     Input("login-button", "n_clicks"),
#     State("username", "value"),
#     State("password", "value"),
#     prevent_initial_call=True
# )
# def login(n_clicks, username, password):
#     if authenticate(username, password):
#         return html.Div(
#             [
#                 dcc.Location(id='url', refresh=False),
#                 status_bar.create_status_bar(),
#                 sidebar.create_sidebar(),
#                 html.Div(id="page-content", style={"marginLeft": "250px"}),
#             ],
#             style={"background": "#BDC3C7", "height": "100vh", "overflow": "hidden"},
#         )
#     else:
#         return html.Div([login_page.layout, html.Div("Login Failed", style={"color": "red"})])

# @app.callback(
#     [
#         Output("position-modal", "is_open"),
#         Output("x-input", "value"),
#         Output("y-input", "value"),
#         Output("z-input", "value"),
#         Output("w-input", "value"),
#         Output("content-area", "children"),
#     ],
#     [
#         Input("add-positions-btn", "n_clicks"),
#         Input("use-robot-btn", "n_clicks"),
#         Input("add-position-btn", "n_clicks"),
#         Input("cancel-btn", "n_clicks"),
#     ],
#     [
#         State("x-input", "value"),
#         State("y-input", "value"),
#         State("z-input", "value"),
#         State("w-input", "value"),
#         State("position-modal", "is_open"),
#     ],
#     prevent_initial_call=True
# )
# def manage_position_modal(add_pos_clicks, use_robot_clicks, add_clicks, cancel_clicks,
#                          x_val, y_val, z_val, w_val, is_open):
#     ctx = dash.callback_context
#     if not ctx.triggered:
#         return no_update, no_update, no_update, no_update, no_update, no_update
#     button_id = ctx.triggered[0]["prop_id"].split(".")[0]
#     if button_id == "add-positions-btn":
#         return True, None, None, None, None, no_update
#     elif button_id == "use-robot-btn":
#         try:
#             listener = tf.TransformListener()
#             listener.waitForTransform("/map", "/base_link", rospy.Time(0), rospy.Duration(1.0))
#             (trans, rot) = listener.lookupTransform("/map", "/base_link", rospy.Time(0))
#             robot_x, robot_y, _ = trans
#             _, _, yaw = tf.transformations.euler_from_quaternion(rot)
#             robot_z, robot_w = math.sin(yaw), math.cos(yaw)
#         except (tf.Exception, tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
#             return True, None, None, None, None, "Không thể lấy vị trí robot"
#         return True, robot_x, robot_y, robot_z, robot_w, no_update
#     elif button_id == "add-position-btn":
#         if x_val is not None and y_val is not None:
#             save_position_to_json(x_val, y_val, z_val, w_val) 
#             generate_marker_image() 
#             return False, None, None, None, None, f"Đã thêm điểm P ({x_val}, {y_val})"
#         return True, x_val, y_val, z_val, w_val, "Vui lòng nhập X và Y"
#     elif button_id == "cancel-btn":
#         return False, None, None, None, None, no_update
#     return no_update, no_update, no_update, no_update, no_update, no_update

# @app.callback(
#     [Output("delete-marker-modal", "is_open"),
#      Output("marker-dropdown", "options")],
#     Input("delete-marker-btn", "n_clicks"),
#     prevent_initial_call=True
# )
# def show_marker_list(n_clicks):
#     markers = load_markers()
#     if not markers:
#         return True, [] 
#     options = [{"label": f"Marker {m['id']}", "value": m["id"]} for m in markers]
#     return True, options  

# @app.callback(
#     Output("delete-marker-modal", "is_open", allow_duplicate=True),
#     Input("confirm-delete-btn", "n_clicks"),
#     State("marker-dropdown", "value"),
#     prevent_initial_call=True
# )
# def delete_marker(n_clicks, selected_marker):
#     if selected_marker is None:
#         return no_update  
#     markers = load_markers()
#     markers = [m for m in markers if m["id"] != selected_marker]
#     with open(JSON_FILE_PATH, "w") as file:
#         json.dump(markers, file, indent=4)
#     generate_marker_image()  
#     return False  

# @app.callback(
#     [Output("add-mission-marker-modal", "is_open", allow_duplicate=True),
#      Output("mission-marker-dropdown", "options")],
#      Input("add-mission-marker-btn", "n_clicks"),
#      prevent_initial_call = True
# )
# def show_mission_marker_list(n_clicks):
#     markers = load_markers()
#     if not markers:
#         return True, []
#     options = [{"label": f"Marker {m['id']}", "value": m["id"]} for m in markers]
#     return True, options 

# @app.callback(
#     Output("add-mission-marker-modal", "is_open", allow_duplicate=True),
#     Input("append-mission-btn", "n_clicks"),
#     State("mission-marker-dropdown", "value"),
#     prevent_initial_call=True
# )
# def append_marker_to_mission(n_clicks, selected_marker):
#     if selected_marker is None:
#         return no_update  
#     save_marker_to_json(selected_marker, clear=False)  
#     return False 

# @app.callback(
#     Output("add-mission-marker-modal", "is_open", allow_duplicate=True),
#     Input("clear-and-append-btn", "n_clicks"),
#     State("mission-marker-dropdown", "value"),
#     prevent_initial_call=True
# )
# def clear_and_append_marker(n_clicks, selected_marker):
#     if selected_marker is None:
#         return no_update  
    
#     save_marker_to_json(selected_marker, clear=True)  
#     return False 

# @app.callback(
#     Output("docker-modal", "is_open", allow_duplicate=True),
#     Input("add-dockers-btn", "n_clicks"),
#     State("docker-modal", "is_open"),
#     prevent_initial_call=True
# )
# def toggle_docker_modal(n_clicks, is_open):
#     return not is_open 

# @app.callback(
#     [
#         Output("docker-modal", "is_open"),
#         Output("docker-x", "value"),
#         Output("docker-y", "value"),
#         Output("docker-z", "value"),
#         Output("docker-w", "value"),
#         Output("content-area", "children", allow_duplicate=True),
#     ],
#     [
#         Input("add-dockers-btn", "n_clicks"),
#         Input("use-robot-docker-btn", "n_clicks"),
#         Input("add-docker-btn", "n_clicks"),
#         Input("cancel-btn", "n_clicks"),
#     ],
#     [
#         State("docker-x", "value"),
#         State("docker-y", "value"),
#         State("docker-z", "value"),
#         State("docker-w", "value"),
#         State("docker-modal", "is_open"),
#     ],
#     prevent_initial_call=True
# )
# def manage_docker_modal(add_pos_clicks, use_robot_clicks, add_clicks, cancel_clicks,
#                          x_val, y_val, z_val, w_val, is_open):
#     ctx = dash.callback_context
#     if not ctx.triggered:
#         return no_update, no_update, no_update, no_update, no_update, no_update
#     button_id = ctx.triggered[0]["prop_id"].split(".")[0]
#     if button_id == "add-dockers-btn":
#         return True, None, None, None, None, no_update
#     elif button_id == "use-robot-docker-btn":
#         try:
#             listener = tf.TransformListener()
#             listener.waitForTransform("/map", "/base_link", rospy.Time(0), rospy.Duration(1.0))
#             (trans, rot) = listener.lookupTransform("/map", "/base_link", rospy.Time(0))
#             robot_x, robot_y, _ = trans
#             _, _, yaw = tf.transformations.euler_from_quaternion(rot)
#             robot_z, robot_w = math.sin(yaw), math.cos(yaw)
#         except (tf.Exception, tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
#             return True, None, None, None, None, "Không thể lấy vị trí robot"
#         return True, robot_x, robot_y, robot_z, robot_w, no_update
#     elif button_id == "add-docker-btn":
#         if x_val is not None and y_val is not None:
#             save_docker_to_json(x_val, y_val, z_val, w_val) 
#             generate_docker_image() 
#             return False, None, None, None, None, f"Đã thêm điểm Docker"
#         return True, x_val, y_val, z_val, w_val, "Vui lòng nhập X và Y"
#     elif button_id == "cancel-btn":
#         return False, None, None, None, None, no_update
#     return no_update, no_update, no_update, no_update, no_update, no_update

# @app.callback(
#     [Output("pause-button", "children"), Output("pause-button", "className")],
#     Input("pause-button", "n_clicks"),
#     State("pause-button", "children"),
#     State("pause-button", "className"),
#     prevent_initial_call=True
# )
# def toggle_pause(n_clicks, current_icon, current_class):
#     if 'fa-play' in current_icon['props']['className']:  
#         return html.I(className="fas fa-pause"), "btn btn-warning btn-sm me-2" 
#     return html.I(className="fas fa-play"), "btn btn-success btn-sm me-2" 

# @app.callback(
#     [Output("pause-button", "children", allow_duplicate=True),  
#      Output("pause-button", "className", allow_duplicate=True)],  
#     Input("pause-button", "n_clicks"),
#     State("pause-button", "children"),
#     prevent_initial_call=True
# )
# def toggle_pause(n_clicks, current_icon):
#     global stop_requested  
#     stop_requested = False 
#     rviz_section = RVizSection()
#     if 'fa-play' in current_icon['props']['className']:  
#         mission_list = load_mission_data()
#         if not mission_list:
#             print("Không có nhiệm vụ nào trong marker_mission.json!")
#             return html.I(className="fas fa-play"), "btn btn-warning btn-sm me-2"
#         for mission in mission_list:
#             if stop_requested:  
#                 return html.I(className="fas fa-play"), "btn btn-warning btn-sm me-2"
#             rviz_section.publish_goal(mission["x"], mission["y"], np.arctan2(mission["z"], mission["w"]))
#             start_time = time.time()
#             while not check_goal_status():
#                 if time.time() - start_time > 30:
#                     print("Timeout: Goal không hoàn thành!")
#                     break
#                 if stop_requested:  
#                     return html.I(className="fas fa-pause"), "btn btn-warning btn-sm me-2"
#                 time.sleep(1)

#             print(f"Đã hoàn thành nhiệm vụ: {mission['id']}")
#             remove_completed_mission(mission["id"])
#         return html.I(className="fas fa-play"), "btn btn-success btn-sm me-2"
#     else:
#         stop_requested = True  
#         return html.I(className="fas fa-play"), "btn btn-success btn-sm me-2"

# @app.callback(
#     Output('page-content', 'children'),
#     Input('url', 'pathname')
# )
# def display_page(pathname):
#     if pathname == '/draw-mode':
#         return html.Div(
#                 [
#                     status_bar.create_status_bar(),
#                     create_draw_mode_layout(),
#                     html.Div(id="joystick-popup-container"),
#                     html.Div(id="joystick-output", style={"margin": "20px", "fontSize": "0px"}),
#                     dcc.Interval(
#                         id='interval-component',
#                         interval=1*1000,
#                         n_intervals=0
#                     )
#                 ]
#             )
#     elif pathname == '/maps':
#         return html.Div(
#                 [
#                     status_bar.create_status_bar(),
#                     map_section.create_map_section(),
#                     html.Div(id="joystick-popup-container"),
#                     html.Div(id="joystick-output", style={"margin": "20px", "fontSize": "0px"}),
#                     dcc.Interval(
#                         id='interval-component',
#                         interval=1*1000,
#                         n_intervals=0
#                     )
#                 ]
#             )
#     elif pathname == '/change-password':
#         return html.Div(
#                 [
#                     status_bar.create_status_bar(),
#                     change_password_page.layout,
#                     html.Div(id="joystick-popup-container"),
#                     html.Div(id="joystick-output", style={"margin": "20px", "fontSize": "0px"}),
#                     dcc.Interval(
#                         id='interval-component',
#                         interval=1*1000,
#                         n_intervals=0
#                     )
#                 ]
#             )
#     elif pathname == '/rviz':
#         return html.Div(
#                 [
#                     status_bar.create_status_bar(),
#                     create_rviz_section(),
#                     html.Div(id="joystick-popup-container"),
#                     html.Div(id="joystick-output", style={"margin": "20px", "fontSize": "0px"}),
#                     dcc.Interval(
#                         id='interval-component',
#                         interval=1*1000,
#                         n_intervals=0
#                     )
#                 ]
#             )
#     elif pathname == '/missions':
#         return mission_queue_layout()
#     elif pathname == '/map-api':
#         return map_api.create_map_api()
#     else:
#         return html.Div(
#             [
#                 status_bar.create_status_bar(),
#                 map_section.create_map_section(),
#                 html.Div(id="joystick-popup-container"),
#                 html.Div(id="joystick-output", style={"margin": "20px", "fontSize": "0px"}),
#                 dcc.Interval(
#                     id='interval-component',
#                     interval=1*1000,
#                     n_intervals=0
#                 )
#             ]
#         )

# @app.callback(
#     Output("password-status", "children"),
#     Input("update-password-button", "n_clicks"),
#     State("new-password", "value"),
#     State("confirm-password", "value"),
#     prevent_initial_call=True
# )
# def update_password_callback(n_clicks, new_password, confirm_password):
#     if new_password == confirm_password:
#         global user_credentials
#         username = list(user_credentials.keys())[0]
#         if update_password(username, new_password):
#             return html.Div("Password updated successfully!", style={"color": "green"})
#         else:
#             return html.Div("Failed to update password.", style={"color": "red"})
#     else:
#         return html.Div("Passwords do not match.", style={"color": "red"})

# @app.callback(
#     Output("joystick-popup-container", "children"),
#     Input("open-joystick-btn", "n_clicks"),
#     prevent_initial_call=True,
# )
# def open_joystick(n_clicks):
#     global joystick_control
#     joystick_control.stop()  
#     return joystick_control.create_joystick_popup()

# @app.callback(
#     Output("joystick-modal", "is_open"),
#     Input("close-joystick-btn", "n_clicks"),
#     State("joystick-modal", "is_open"),
#     prevent_initial_call=True
# )
# def close_joystick(n_clicks, is_open):
#     return not is_open

# @app.callback(
#     [Output("linear-speed-display", "children"),
#      Output("angular-speed-display", "children")],
#     [Input("linear-speed-input", "value"),
#      Input("angular-speed-input", "value")],
#     prevent_initial_call=True
# )
# def update_speed(linear_speed_value, angular_speed_value):
#     global joystick_control

#     linear_speed = linear_speed_value
#     angular_speed = angular_speed_value

#     joystick_control.linear_speed = linear_speed
#     joystick_control.angular_speed = angular_speed

#     speed_message = "Giá trị tốc độ đã được cập nhật"

#     return (
#         f"Linear Speed: {linear_speed_value}",
#         f"Angular Speed: {angular_speed_value}",
#     )

# @app.callback(
#     Output("joystick-output", "children"),
#     [Input("forward-button", "n_clicks_timestamp"),
#      Input("backward-button", "n_clicks_timestamp"),
#      Input("left-button", "n_clicks_timestamp"),
#      Input("right-button", "n_clicks_timestamp"),
#      Input("forward-left-button", "n_clicks_timestamp"),
#      Input("forward-right-button", "n_clicks_timestamp"),
#      Input("back-left-button", "n_clicks_timestamp"),
#      Input("back-right-button", "n_clicks_timestamp"),
#      Input("stop-button", "n_clicks")],
#     prevent_initial_call=True
# )
# def teleop_control(f, b, l, r, fl, fr, bl, br, s):
#     ctx = callback_context
#     joystick_control.stop()
#     triggered_id = ctx.triggered[0]['prop_id'].split('.')[0] if ctx.triggered else None

#     if triggered_id == "forward-button":
#         joystick_control.move_forward()
#         return "Moving Forward"
#     elif triggered_id == "backward-button":
#         joystick_control.move_backward()
#         return "Moving Backward"
#     elif triggered_id == "left-button":
#         joystick_control.turn_left()
#         return "Turning Left"
#     elif triggered_id == "right-button":
#         joystick_control.turn_right()
#         return "Turning Right"
#     elif triggered_id == "forward-left-button":
#         joystick_control.move_forward_left()
#         return "Moving Forward Left"
#     elif triggered_id == "forward-right-button":
#         joystick_control.move_forward_right()
#         return "Moving Forward Right"
#     elif triggered_id == "back-left-button":
#         joystick_control.move_backward_left()
#         return "Moving Backward Left"
#     elif triggered_id == "back-right-button":
#         joystick_control.move_backward_right()
#         return "Moving Backward Right"
#     elif triggered_id == "stop-button":
#         joystick_control.stop()
#         return "Stop"
#     else:
#         return "No movement"

# @app.callback(
#     Output('map_section', 'children'),
#     Input('language-dropdown', 'value')
# )
# def change_language(language):
#     translations = {
#         'en': {'title': 'Main Floor', 'map': 'Edit and draw the map', 'ready': 'El mapa está listo para tu trabajo.'},
#         'vi': {'title': 'Tầng Chính', 'map': 'Edit and draw the map', 'ready': 'El mapa está listo para tu trabajo.'},
#         'es': {'title': 'Piso Principal', 'map': 'Edit and draw the map', 'ready': 'El mapa está listo para tu trabajo.'},
#     }
#     translation = translations.get(language, translations['en'])
#     return html.Div(
#         [
#             html.H3(translation['title'], className="mb-3", style={"color": "#2C3E50"}),
#             html.P(translation['map'], className="text-muted"),
#             html.Img(src="/path/to/save/map_image.png", style={"width": "100%", "border": "2px solid #34495E"}),
#             html.P(translation['ready'], className="text-info mt-2"),
#             html.Div(id="content-area"),
#         ],
#         style={
#             "padding": "20px",
#             "flex": "1",
#             "background": "#ECF0F1",
#             "marginLeft": "250px",
#             "marginTop": "50px",
#         },
#     )

# @app.callback(
#     Output("lidar-image", "src"),
#     Input("interval-component", "n_intervals")
# )
# def update_lidar_image(n):
#     timestamp = int(time.time())
#     return f"/static/lidar_image.png?{timestamp}"

# @app.callback(
#     [Output("lidar-f-image", "src"), Output("lidar-b-image", "src")],
#     Input("interval-component", "n_intervals")
# )
# def update_lidar_images(n):
#     timestamp = int(time.time())
#     return (
#         f"/static/f_scan_image.png?{timestamp}",
#         f"/static/b_scan_image.png?{timestamp}"
#     )

# @app.callback(
#     [Output("path-image", "src")],
#     Input("interval-component", "n_intervals")
# )
# def update_path_image(n):
#     random_value = random.randint(1, 100000)
#     return (
#         f"/static/path_image.png?random={random_value}",
#     )

# @app.callback(
#     Output("lines-image", "src"),
#     Input("interval-component", "n_intervals")
# )
# def update_lines_image(n):
#     timestamp = int(time.time())
#     return f"/static/line_image.png?{timestamp}"

# @app.callback(
#     Output("markers-image", "src"),
#     Input("interval-component", "n_intervals")
# )
# def update_markers_image(n):
#     timestamp = int(time.time())
#     return f"/static/all_markers.png?{timestamp}"

# @app.callback(
#     Output("dockers-image", "src"),
#     Input("interval-component", "n_intervals")
# )
# def update_markers_image(n):
#     timestamp = int(time.time())
#     return f"/static/dockers.png?{timestamp}"

# @app.callback(
#     Output("map-image", "src"),
#     Input("interval-component", "n_intervals")
# )
# def update_map_image(n):
#     timestamp = int(time.time())
#     return f"/static/map_image.png?{timestamp}"

# last_modified_time = 0

# @app.callback(
#     [Output("robot-image", "src")],
#     Input("interval-component", "n_intervals")
# )
# def update_robot_image(n):
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
        #print("dash- callback updating img with: " + image_path + "?time=" + str(modified_time))
        return [f"/static/robot_image.png?time={modified_time}"]
    else:
        raise PreventUpdate

# app.layout = html.Div(
#     [
#         dcc.Location(id='url', refresh=False),
#         html.Div(id="app-container", children=[login_page.layout]),
#         html.Div(id="joystick-output", style={"margin": "20px", "fontSize": "0px"}),
#         dcc.Interval(
#             id='interval-component',
#             interval=1000,
#             n_intervals=0
#         )
#     ]
# )

# @app.callback(
#     Output("sidebar-nav", "children"),
#     Input('url', 'pathname'),
#     prevent_initial_call=True
# )
# def update_active_link(pathname):
#     nav_links = [
#         {"href": "/", "id": "index-link", "label": "Home"},
#         {"href": "/draw-mode", "id": "draw-mode-link", "label": "Draw Mode"},
#         {"href": "/rviz", "id": "rviz-link", "label": "RViz"},
#         {"href": "/missions", "id": "mission-link", "label": "Missions"},
#         {"href": "/map-api", "id": "map-api-link", "label": "Maps"},
#         {"href": "#", "id": "io-modules-link", "label": "I/O Modules"},
#         {"href": "#", "id": "users-link", "label": "Users"},
#         {"href": "#", "id": "user-groups-link", "label": "User Groups"},
#         {"href": "#", "id": "paths-link", "label": "Paths"},
#         {"href": "#", "id": "path-guides-link", "label": "Path Guides"},
#         {"href": "#", "id": "marker-types-link", "label": "Marker Types"},
#         {"href": "#", "id": "footprints-link", "label": "Footprints"},
#         {"href": "/change-password", "id": "change-password-link", "label": "Change Password"},
#     ]

#     updated_links = []
#     for link in nav_links:
#         active = pathname == link["href"]
#         updated_links.append(
#             dbc.NavLink(
#                 link["label"],
#                 href=link["href"],
#                 id=link["id"],
#                 className="text-white",
#                 active=active
#             )
#         )
#     return updated_links

# @app.callback(
#     Output("battery-percen", "children"),
#     Input("interval-component", "n_intervals")
# )
# def update_battery_status(n):

#     try:
#         response = requests.get(host + '/status', headers=headers)  
#         data = response.json()
#         battery_level = data.get("battery_percentage", "--") 
#         if isinstance(battery_level, float):
#             battery_level = round(battery_level)
#         return f"{battery_level}%"
#     except Exception as e:
#         print(f"⚠️ Lỗi khi gọi API: {e}")  # Log lỗi
#         return "--%"
# register_callbacks(app)

if __name__ == "__main__":
    app.run_server(debug=True, host='0.0.0.0', port=8000)