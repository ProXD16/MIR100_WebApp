import rospy
import tf2_ros
from nav_msgs.msg import Path, OccupancyGrid
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PoseWithCovarianceStamped
from PIL import Image, ImageDraw
import math
import json
import numpy as np
import tf2_geometry_msgs
from tf.transformations import euler_from_quaternion
import geometry_msgs.msg


OUTPUT_IMAGE_PATH = "static/combined_image.png"
MAP_IMAGE_PATH = "static/map_image.png"
ROBOT_IMAGE_PATH = "static/robot_image.png"
PATH_IMAGE_PATH = "static/path_image.png"
F_SCAN_IMAGE_PATH = "static/f_scan_image.png"
B_SCAN_IMAGE_PATH = "static/b_scan_image.png"
LINE_IMAGE_PATH = "static/line_image.png"
IMAGE_WIDTH = None
IMAGE_HEIGHT = None
MAP_ORIGIN_X = None
MAP_ORIGIN_Y = None
MAP_RESOLUTION = None
LIDAR_RANGE = 50
POINT_SIZE = 1
JSON_FILE_PATH = "save_lines/line_drawn.json"
data_flags = {
    "map": False,
    "path": False,
    "f_scan": False,
    "b_scan": False,
    "pose": False,
    "lines": False
}


# Function to convert world coordinates to image coordinates
def world_to_image(x, y):
    global IMAGE_WIDTH, IMAGE_HEIGHT, MAP_ORIGIN_X, MAP_ORIGIN_Y, MAP_RESOLUTION
    if MAP_ORIGIN_X is None or MAP_ORIGIN_Y is None or MAP_RESOLUTION is None or IMAGE_WIDTH is None or IMAGE_HEIGHT is None:
        rospy.logwarn("Map information not yet received. Cannot transform coordinates.")
        return None, None

    px = int((x - MAP_ORIGIN_X) / MAP_RESOLUTION)
    py = int(IMAGE_HEIGHT - (y - MAP_ORIGIN_Y) / MAP_RESOLUTION)
    return px, py


def process_lidar_data(msg, tf_buffer, frame_id):
    points = []

    for i in range(len(msg.ranges)):
        r = msg.ranges[i]
        angle = i * msg.angle_increment + msg.angle_min

        if msg.range_min < r < msg.range_max and r < LIDAR_RANGE:
            x = r * math.cos(angle)
            y = r * math.sin(angle)

            # Tạo point trong hệ tọa độ của Lidar
            point_stamped = geometry_msgs.msg.PointStamped()
            point_stamped.header.frame_id = frame_id
            point_stamped.header.stamp = rospy.Time(0)  
            point_stamped.point.x = x
            point_stamped.point.y = y
            point_stamped.point.z = 0  

            try:
                if tf_buffer.can_transform("map", frame_id, rospy.Time(0)):
                    transform = tf_buffer.lookup_transform("map", frame_id, rospy.Time(0))
                    transformed_point = tf2_geometry_msgs.do_transform_point(point_stamped, transform)
                    points.append((transformed_point.point.x, transformed_point.point.y))

                else:
                    rospy.logwarn(f"Không thể transform {frame_id} sang map. Bỏ qua điểm này.")

            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
                rospy.logwarn(f"Lỗi TF: {e}")

    return points


def create_lidar_image(points):
    img = Image.new('RGBA', (IMAGE_WIDTH, IMAGE_HEIGHT), (0, 0, 0, 0))
    draw = ImageDraw.Draw(img)

    if not points:
        return img
    for point_x, point_y in points:
        px, py = world_to_image(point_x, point_y)

        if px is not None and py is not None:
            draw.ellipse((px - POINT_SIZE, py - POINT_SIZE, px + POINT_SIZE, py + POINT_SIZE), fill=(255, 0, 0))

    return img


def path_callback(msg):
    global IMAGE_WIDTH, IMAGE_HEIGHT
    try:
        path_points = [(pose.pose.position.x, pose.pose.position.y) for pose in msg.poses]
        img = Image.new("RGBA", (IMAGE_WIDTH, IMAGE_HEIGHT), (0, 0, 0, 0))
        draw = ImageDraw.Draw(img)

        for world_x, world_y in path_points:
            px, py = world_to_image(world_x, world_y)
            if px is not None and py is not None:
                draw.ellipse((px - 0.5, py - 0.5, px + 0.5, py + 0.5), fill=(0, 255, 255))  

        img.save(PATH_IMAGE_PATH)
        rospy.loginfo(f"Path image saved to {PATH_IMAGE_PATH}")
        data_flags["path"] = True

    except Exception as e:
        rospy.logerr(f"Error processing path data: {e}")


def pose_callback(msg, tf_buffer):
    global IMAGE_WIDTH, IMAGE_HEIGHT
    if IMAGE_WIDTH is None or IMAGE_HEIGHT is None:
        rospy.logwarn("Image dimensions not yet initialized. Skipping pose callback.")
        return

    try:
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        quaternion = (
            msg.pose.pose.orientation.x,
            msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z,
            msg.pose.pose.orientation.w
        )
        _, _, yaw = euler_from_quaternion(quaternion)

        img = Image.new("RGBA", (IMAGE_WIDTH, IMAGE_HEIGHT), (0, 0, 0, 0))
        draw = ImageDraw.Draw(img)

        px, py = world_to_image(x, y)
        if px is None or py is None:
            rospy.logwarn(f"Could not convert robot pose ({x}, {y}) to image coordinates.  Map origin/resolution may be incorrect. px={px}, py={py}")
            return 
        rect_length = 0.8
        rect_width = 0.6
        tri_side = 0.3
        corners_world = [
            (x + rect_length / 2 * math.cos(yaw) - rect_width / 2 * math.sin(yaw),
             y + rect_length / 2 * math.sin(yaw) + rect_width / 2 * math.cos(yaw)),
            (x + rect_length / 2 * math.cos(yaw) + rect_width / 2 * math.sin(yaw),
             y + rect_length / 2 * math.sin(yaw) - rect_width / 2 * math.cos(yaw)),
            (x - rect_length / 2 * math.cos(yaw) + rect_width / 2 * math.sin(yaw),
             y - rect_length / 2 * math.sin(yaw) - rect_width / 2 * math.cos(yaw)),
            (x - rect_length / 2 * math.cos(yaw) - rect_width / 2 * math.sin(yaw),
             y - rect_length / 2 * math.sin(yaw) + rect_width / 2 * math.cos(yaw))
        ]
        corners_image = []
        all_corners_valid = True  #
        for corner in corners_world:
            px_c, py_c = world_to_image(corner[0], corner[1])
            if px_c is None or py_c is None:
                rospy.logwarn(f"Could not convert rectangle corner ({corner[0]}, {corner[1]}) to image coordinates.")
                all_corners_valid = False
                break 
            corners_image.append((px_c, py_c))
        if all_corners_valid:
            draw.polygon(corners_image, fill=(128, 128, 128, 102))
        else:
            rospy.logwarn("Not all rectangle corners could be converted; skipping rectangle drawing.")
        triangle_points_world = [
            (x + tri_side * math.cos(yaw), y + tri_side * math.sin(yaw)),
            (x - tri_side / 2 * math.cos(yaw) + (tri_side * math.sqrt(3) / 2) * math.sin(yaw),
             y - tri_side / 2 * math.sin(yaw) - (tri_side * math.sqrt(3) / 2) * math.cos(yaw)),
            (x - tri_side / 2 * math.cos(yaw) - (tri_side * math.sqrt(3) / 2) * math.sin(yaw),
             y - tri_side / 2 * math.sin(yaw) + (tri_side * math.sqrt(3) / 2) * math.cos(yaw))
        ]

        triangle_points_image = []
        all_triangle_points_valid = True  
        for point in triangle_points_world:
            px_t, py_t = world_to_image(point[0], point[1])
            if px_t is None or py_t is None:
                rospy.logwarn(f"Could not convert triangle point ({point[0]}, {point[1]}) to image coordinates.")
                all_triangle_points_valid = False
                break
            triangle_points_image.append((px_t, py_t))

        if all_triangle_points_valid:
            draw.polygon(triangle_points_image, fill=(0, 0, 255, 178))
        else:
            rospy.logwarn("Not all triangle points could be converted; skipping triangle drawing.")
        img.save(ROBOT_IMAGE_PATH)
        rospy.loginfo(f"Robot location image updated: {ROBOT_IMAGE_PATH}")
        # new_data_received = True
        data_flags["pose"] = True

    except Exception as e:
        rospy.logerr(f"Error updating robot pose image: {e}")


def map_info_callback(map_data):
    """Callback function to handle map information."""
    global MAP_ORIGIN_X, MAP_ORIGIN_Y, MAP_RESOLUTION, IMAGE_WIDTH, IMAGE_HEIGHT

    MAP_ORIGIN_X = map_data.info.origin.position.x
    MAP_ORIGIN_Y = map_data.info.origin.position.y
    MAP_RESOLUTION = map_data.info.resolution
    IMAGE_WIDTH = map_data.info.width
    IMAGE_HEIGHT = map_data.info.height
    rospy.loginfo("Received map info")
    data_flags["map"] = True
    load_and_create_lines()


def lidar_callback(msg, tf_buffer, topic_name):
    try:
        frame_id = "back_laser_link" if topic_name == "/b_scan" else "front_laser_link"
        points = process_lidar_data(msg, tf_buffer, frame_id)

        if points:
            img_output = create_lidar_image(points)
            output_path = f"static/{topic_name.split('/')[-1]}_image.png"
            img_output.save(output_path)
            rospy.loginfo(f"Lidar image for {topic_name} saved to {output_path}")
            if topic_name == "/f_scan":
                data_flags["f_scan"] = True
            else:
                data_flags["b_scan"] = True
        else:
            rospy.logwarn(f"No valid Lidar points for {topic_name}, skipping image save.")

    except Exception as e:
        rospy.logerr(f"Error processing {topic_name} data: {e}")


def load_lines_from_json():
    """Loads line data from the specified JSON file."""
    try:
        with open(JSON_FILE_PATH, 'r') as f:
            data = json.load(f)
            rospy.loginfo("Loaded line data from JSON.")
            return data
    except FileNotFoundError:
        rospy.logwarn(f"JSON file not found: {JSON_FILE_PATH}")
        return None
    except json.JSONDecodeError:
        rospy.logerr(f"Error decoding JSON from {JSON_FILE_PATH}")
        return None
    except Exception as e:
        rospy.logerr(f"Error loading line data: {e}")
        return None


def create_line_image(lines_data):
    """Creates an image with the lines drawn on it."""
    global IMAGE_WIDTH, IMAGE_HEIGHT

    img = Image.new("RGBA", (IMAGE_WIDTH, IMAGE_HEIGHT), (0, 0, 0, 0))
    draw = ImageDraw.Draw(img)

    if lines_data is None:
        return img

    try:
        for item in lines_data:
            if isinstance(item, dict):
                if "type" in item:
                    if item["type"] == "line":
                        x_coords = item["x"]
                        y_coords = item["y"]
                        image_coords = []
                        for i in range(len(x_coords)):
                            px, py = world_to_image(x_coords[i], y_coords[i])
                            if px is not None and py is not None:
                                image_coords.append((px, py))
                            else:
                                rospy.logwarn("Invalid world coordinates, skipping point.")
                        if len(image_coords) >= 2:
                            draw.line(image_coords, fill="purple", width=2)
                        else:
                            rospy.logwarn("Not enough valid points to draw a line.")


                    elif item["type"] == "arc":
                        n_points = 50
                        start_angle = item["start_angle"]
                        end_angle = item["end_angle"]
                        center_x = item["center_x"]
                        center_y = item["center_y"]
                        radius = item["radius"]

                        angles = np.linspace(start_angle, end_angle, n_points)
                        x = center_x + radius * np.cos(angles)
                        y = center_y + radius * np.sin(angles)
                        image_coords = []
                        for i in range(n_points):
                            px, py = world_to_image(x[i], y[i])
                            if px is not None and py is not None:
                                image_coords.append((px, py))
                            else:
                                rospy.logwarn("Invalid world coordinates for arc, skipping point.")
                        if len(image_coords) >= 2:
                            draw.line(image_coords, fill="purple", width=2)
                        else:
                            rospy.logwarn("Not enough valid points to draw the arc.")
                    else:
                        rospy.logwarn(f"Unknown type: {item['type']} in item: {item}")
                else:
                    rospy.logwarn(f"'type' key missing in item: {item}")
            else:
                rospy.logwarn(f"Expected a dictionary, but got: {type(item)}, value: {item}")

        img.save(LINE_IMAGE_PATH)
        rospy.loginfo(f"Line image saved to {LINE_IMAGE_PATH}")
        data_flags["lines"] = True 

    except Exception as e:
        rospy.logerr(f"Error drawing lines: {e}")

    return img

def load_and_create_lines():
    lines_data = load_lines_from_json()
    create_line_image(lines_data)

def combine_images():
    try:
        if not all(data_flags.values()):
            rospy.logwarn("Not all required data has been received. Skipping image combination.")
            return
        map_img = Image.open(MAP_IMAGE_PATH).convert("RGBA")
        robot_img = Image.open(ROBOT_IMAGE_PATH).convert("RGBA")
        path_img = Image.open(PATH_IMAGE_PATH).convert("RGBA")
        f_scan_img = Image.open(F_SCAN_IMAGE_PATH).convert("RGBA")
        b_scan_img = Image.open(B_SCAN_IMAGE_PATH).convert("RGBA")
        line_img = Image.open(LINE_IMAGE_PATH).convert("RGBA")
        map_img.paste(path_img, (0, 0), path_img)
        map_img.paste(robot_img, (0, 0), robot_img)
        map_img.paste(f_scan_img, (0, 0), f_scan_img)
        map_img.paste(b_scan_img, (0, 0), b_scan_img)
        map_img.paste(line_img, (0, 0), line_img) 
        map_img.save(OUTPUT_IMAGE_PATH)
        rospy.loginfo(f"Combined image saved to {OUTPUT_IMAGE_PATH}")
        reset_data_flags()

    except FileNotFoundError as e:
        rospy.logerr(f"One or more image files not found: {e}")
    except Exception as e:
        rospy.logerr(f"Error combining images: {e}")

def reset_data_flags():
    """Resets all data flags to False."""
    for key in data_flags:
        data_flags[key] = False


def main():
    """Main function."""
    rospy.init_node('image_combiner_node', anonymous=True)
    tf_buffer = tf2_ros.Buffer(cache_time=rospy.Duration(5.0))
    tf_listener = tf2_ros.TransformListener(tf_buffer)
    rospy.Subscriber("/map", OccupancyGrid, map_info_callback)
    rospy.Subscriber("/move_base_node/SBPLLatticePlanner/plan", Path, path_callback)
    rospy.Subscriber("/f_scan", LaserScan, lambda msg: lidar_callback(msg, tf_buffer, "/f_scan"))
    rospy.Subscriber("/b_scan", LaserScan, lambda msg: lidar_callback(msg, tf_buffer, "/b_scan"))
    rospy.Subscriber("/amcl_pose", PoseWithCovarianceStamped, pose_callback, tf_buffer)

    rate = rospy.Rate(0.1)

    while not rospy.is_shutdown():

        if all(data_flags.values()):
            rospy.loginfo("Combining images due to new data...")
            combine_images()
            rospy.sleep(0.5)  
        rate.sleep()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass