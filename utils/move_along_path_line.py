import numpy as np
import rospy
import tf
from geometry_msgs.msg import Twist

is_moving = True

def send_velocity(linear_speed, angular_speed):
    cmd_vel_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    twist = Twist()
    twist.linear.x = linear_speed
    twist.angular.z = angular_speed
    cmd_vel_publisher.publish(twist)

def get_robot_position():
    tf_listener = tf.TransformListener()
    try:
        tf_listener.waitForTransform('/map', '/base_link', rospy.Time(0), rospy.Duration(10))
        (trans, rot) = tf_listener.lookupTransform('/map', '/base_link', rospy.Time(0))
        robot_x = trans[0]
        robot_y = trans[1]
        robot_yaw = tf.transformations.euler_from_quaternion(rot)[2]
        return robot_x, robot_y, robot_yaw
    except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
        rospy.logerr("TF error when getting robot position. Check if TF is publishing /map to /base_link.")
        return None, None, None

def rotate_to_target(target_x, target_y, angular_speed):
    global is_moving
    robot_x, robot_y, robot_yaw = get_robot_position()
    if robot_x is None:
        return
    
    tolerance = np.radians(0.01)  

    while is_moving:
        target_angle = np.arctan2(target_y - robot_y, target_x - robot_x)
        angle_error = target_angle - robot_yaw
        angle_error = (angle_error + np.pi) % (2 * np.pi) - np.pi
        
        if abs(angle_error) <= tolerance:
            break

        angular_vel = (abs(angle_error) / (2*np.pi)) * angular_speed if angle_error > 0 else -(abs(angle_error) / (2*np.pi)) * angular_speed
        send_velocity(0, angular_vel)
        rospy.sleep(0.01)
        robot_x, robot_y, robot_yaw = get_robot_position()
    
    send_velocity(0, 0)


def calculate_distance(x_start, y_start, x_end, y_end):
    return np.sqrt((x_start-x_end)**2 + (y_start-y_end)**2)

def move_to_target(target_x, target_y, linear_speed):
    global is_moving
    robot_x, robot_y, _ = get_robot_position()
    if robot_x is None:
        return

    distance_to_target = calculate_distance(robot_x, robot_y, target_x, target_y)
    distance_traveled = 0.0 

    while calculate_distance(robot_x, robot_y, target_x, target_y) > 0.2 and distance_traveled < distance_to_target and is_moving:
        send_velocity(linear_speed, 0)
        rospy.sleep(0.1)
        robot_x_new, robot_y_new, _ = get_robot_position()

        incremental_distance = calculate_distance(robot_x, robot_y, robot_x_new, robot_y_new)
        distance_traveled += incremental_distance 
        robot_x, robot_y = robot_x_new, robot_y_new 
        print(str(robot_x) + " " + str(robot_y))

    send_velocity(0, 0)

def stop_emergency(): 
    global is_moving
    send_velocity(0, 0)
    is_moving = False
    rospy.loginfo("Emergency stop activated!")

def emergency_stop_callback(msg):
    global is_moving
    if msg.data:
        stop_emergency()
