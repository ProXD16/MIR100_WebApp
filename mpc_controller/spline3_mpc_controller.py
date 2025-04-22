import rospy
from geometry_msgs.msg import Pose, Twist
import numpy as np
import tf.transformations
from scipy.interpolate import CubicSpline
from scipy.signal import butter, filtfilt

class Spline3_MPCController:
    def __init__(self, dt, v_max, v_min, omega_max, omega_min, lookahead_distance=0.1, filter_order=4, cutoff_frequency=4.0):
        self.dt = dt
        self.v_max = v_max
        self.v_min = v_min
        self.omega_max = omega_max
        self.omega_min = omega_min
        self.current_pose = None
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.trajectory_x = []
        self.trajectory_y = []
        self.spline = None
        self.spline_derivative = None 
        self.waypoints_x = []
        self.waypoints_y = []
        self.distance_threshold = 0.2
        self.lookahead_distance = lookahead_distance
        self.reached_goal = False
        self.last_v = 0.0  
        self.last_omega = 0.0  
        self.velocity_data = []
        self.angular_velocity_data = []
        self.acceleration_data = []
        self.angular_acceleration_data = []
        self.time_data = []
        self.start_time = None
        self.filter_order = filter_order
        self.cutoff_frequency = cutoff_frequency  
        self.b, self.a = butter(self.filter_order, self.cutoff_frequency / (1 / (2 * self.dt)), btype='low', analog=False)

    def lowpass_filter(self, data):
        padlen = 3 * self.filter_order
        if len(data) <= padlen: 
            return data
        y = filtfilt(self.b, self.a, data, padlen=padlen)  
        return y

    def pose_callback(self, msg):
        quat = msg.orientation
        if quat.w == 0.0 and quat.x == 0.0 and quat.y == 0.0 and quat.z == 0.0:
            rospy.logwarn("Invalid quaternion received, skipping pose update.")
            return

        try:
            euler = tf.transformations.euler_from_quaternion([quat.x, quat.y, quat.z, quat.w])
            roll, pitch, yaw = euler[0], euler[1], euler[2]
        except Exception as e:
            rospy.logerr(f"Error converting quaternion to Euler: {e}")
            return

        self.x = msg.position.x
        self.y = msg.position.y
        self.theta = yaw
        self.current_pose = np.array([self.x, self.y, self.theta])
        self.trajectory_x.append(self.x)
        self.trajectory_y.append(self.y)

        if len(self.waypoints_x) == 0:
            self.waypoints_x.append(self.x)
            self.waypoints_y.append(self.y)
        else:
            self.waypoints_x[0] = self.x
            self.waypoints_y[0] = self.y

    def calculate_spline(self):
        if len(self.waypoints_x) < 2:
            rospy.logwarn("Need at least two waypoints to create a spline.")
            return False

        self.waypoints_x = np.array(self.waypoints_x)
        self.waypoints_y = np.array(self.waypoints_y)

        sorted_indices = np.argsort(self.waypoints_x)
        x_sorted = self.waypoints_x[sorted_indices]
        y_sorted = self.waypoints_y[sorted_indices]

        try:
            self.spline = CubicSpline(x_sorted, y_sorted)
            self.spline_derivative = self.spline.derivative()  # Tính đạo hàm bậc 1
            return True
        except Exception as e:
            rospy.logerr(f"Error creating spline: {e}")
            return False

    def find_closest_point_on_spline(self, x, y):
        # Tìm điểm gần nhất trên spline
        x_range = np.linspace(self.waypoints_x[0], self.waypoints_x[-1], num=1000)
        y_range = self.spline(x_range)
        distances = np.sqrt((x_range - x)**2 + (y_range - y)**2)
        closest_idx = np.argmin(distances)
        return x_range[closest_idx]

    def mpc_control(self, x, y, theta):
        if self.spline is None:
            rospy.logwarn("Spline is not defined. Cannot perform MPC control.")
            return 0.0, 0.0

        # Tìm điểm gần nhất trên spline
        closest_x = self.find_closest_point_on_spline(x, y)

        # Tính điểm lookahead trên spline
        lookahead_x = closest_x + self.lookahead_distance
        if lookahead_x > self.waypoints_x[-1]:
            lookahead_x = self.waypoints_x[-1]
        elif lookahead_x < self.waypoints_x[0]:
            lookahead_x = self.waypoints_x[0]

        lookahead_y = self.spline(lookahead_x)

        # Kiểm tra nếu đã đến mục tiêu cuối cùng
        final_goal_x = self.waypoints_x[-1]
        final_goal_y = self.waypoints_y[-1]
        distance_to_final_goal = np.sqrt((x - final_goal_x)**2 + (y - final_goal_y)**2)
        if distance_to_final_goal < self.distance_threshold:
            rospy.loginfo("Reached the final goal!")
            self.reached_goal = True
            return 0.0, 0.0

        # Tính hướng của spline tại điểm lookahead (dùng đạo hàm)
        dy_dx = self.spline_derivative(lookahead_x)
        angle_to_goal = np.arctan(dy_dx)  # Góc của tiếp tuyến spline
        heading_error = angle_to_goal - theta
        heading_error = np.arctan2(np.sin(heading_error), np.cos(heading_error))

        # Tính lỗi vị trí
        x_error = lookahead_x - x
        y_error = lookahead_y - y
        distance_error = np.sqrt(x_error**2 + y_error**2)

        # Điều khiển
        heading_threshold = 0.1
        linear_speed_kp = 1.8  
        angular_speed_kp = 1.8

        if abs(heading_error) > heading_threshold:
            v = 0.0
            omega = angular_speed_kp * heading_error
        else:
            v = linear_speed_kp * distance_error
            omega = angular_speed_kp * heading_error

        v = np.clip(v, self.v_min, self.v_max)
        omega = np.clip(omega, self.omega_min, self.omega_max)

        # Tính gia tốc
        if self.start_time is not None:
            current_time = rospy.Time.now().to_sec()
            dt = current_time - self.start_time

            linear_acceleration = (v - self.last_v) / dt if dt > 0 else 0.0
            angular_acceleration = (omega - self.last_omega) / dt if dt > 0 else 0.0

            self.acceleration_data.append(linear_acceleration)
            self.angular_acceleration_data.append(angular_acceleration)
            self.velocity_data.append(v)
            self.angular_velocity_data.append(omega)
            self.time_data.append(current_time - self.start_time)

        self.last_v = v
        self.last_omega = omega

        return v, omega