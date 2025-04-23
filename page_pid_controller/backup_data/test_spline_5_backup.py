import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped, Twist
import numpy as np
import tkinter as tk
import tkinter.ttk as ttk  # Import ttk separately
import threading
import tf.transformations
import matplotlib
matplotlib.use("TkAgg")
import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from scipy.interpolate import PPoly, splrep, splev
from scipy.signal import butter, filtfilt

class MPCController:
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
        self.spline_x = None  # Spline cho x(s)
        self.spline_y = None  # Spline cho y(s)
        self.arclength = None  # Độ dài cung của spline
        self.waypoints_x = []
        self.waypoints_y = []
        self.distance_threshold = 0.2
        self.lookahead_distance = lookahead_distance
        self.reached_goal = False
        self.last_v = 0.0  # Lưu trữ vận tốc tuyến tính trước đó
        self.last_omega = 0.0  # Lưu trữ vận tốc góc trước đó
        self.velocity_data = []
        self.angular_velocity_data = []
        self.acceleration_data = []
        self.angular_acceleration_data = []
        self.time_data = []
        self.start_time = None
        self.filter_order = filter_order
        self.cutoff_frequency = cutoff_frequency  # Hz
        self.b, self.a = butter(self.filter_order, self.cutoff_frequency / (1 / (2 * self.dt)), btype='low', analog=False)

    def lowpass_filter(self, data):
        padlen = 3 * self.filter_order
        if len(data) <= padlen:  # Need enough data points for padding
            return data
        y = filtfilt(self.b, self.a, data, padlen=padlen)  # Add padlen parameter
        return y

    def pose_callback(self, msg):
        quat = msg.pose.pose.orientation
        if quat.w == 0.0 and quat.x == 0.0 and quat.y == 0.0 and quat.z == 0.0:
            rospy.logwarn("Invalid quaternion received, skipping pose update.")
            return

        try:
            euler = tf.transformations.euler_from_quaternion([quat.x, quat.y, quat.z, quat.w])
            roll, pitch, yaw = euler[0], euler[1], euler[2]
        except Exception as e:
            rospy.logerr(f"Error converting quaternion to Euler: {e}")
            return

        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
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

        # Tính tham số hóa độ dài cung
        distances = np.cumsum(np.sqrt(np.diff(self.waypoints_x)**2 + np.diff(self.waypoints_y)**2))
        distances = np.insert(distances, 0, 0)
        self.arclength = distances[-1]

        try:
            # Tạo spline bậc 5 cho x và y theo độ dài cung
            tck_x = splrep(distances, self.waypoints_x, k=5, s=0)  # k=5 cho bậc 5, s=0 cho khớp chính xác
            tck_y = splrep(distances, self.waypoints_y, k=5, s=0)

            self.spline_x = tck_x
            self.spline_y = tck_y
            return True

        except Exception as e:
            rospy.logerr(f"Error creating spline: {e}")
            return False

    def find_closest_point_on_spline(self, x, y):
        # Tìm điểm gần nhất trên spline bằng cách sử dụng độ dài cung
        s_range = np.linspace(0, self.arclength, num=1000)
        x_range = splev(s_range, self.spline_x)
        y_range = splev(s_range, self.spline_y)
        distances = np.sqrt((x_range - x)**2 + (y_range - y)**2)
        closest_idx = np.argmin(distances)
        return s_range[closest_idx]  # Trả về độ dài cung gần nhất

    def mpc_control(self, x, y, theta):
        if self.spline_x is None or self.spline_y is None:
            rospy.logwarn("Spline is not defined. Cannot perform MPC control.")
            return 0.0, 0.0

        # Tìm điểm gần nhất trên spline (trả về độ dài cung)
        closest_s = self.find_closest_point_on_spline(x, y)

        # Tính điểm lookahead trên spline
        lookahead_s = closest_s + self.lookahead_distance
        if lookahead_s > self.arclength:
            lookahead_s = self.arclength
        elif lookahead_s < 0:
            lookahead_s = 0

        lookahead_x = splev(lookahead_s, self.spline_x)
        lookahead_y = splev(lookahead_s, self.spline_y)

        # Kiểm tra nếu đã đến mục tiêu cuối cùng
        final_goal_x = self.waypoints_x[-1]
        final_goal_y = self.waypoints_y[-1]
        distance_to_final_goal = np.sqrt((x - final_goal_x)**2 + (y - final_goal_y)**2)
        if distance_to_final_goal < self.distance_threshold:
            rospy.loginfo("Reached the final goal!")
            self.reached_goal = True
            return 0.0, 0.0

        # Tính hướng của spline tại điểm lookahead (dùng đạo hàm)
        dx_ds = splev(lookahead_s, self.spline_x, der=1)
        dy_ds = splev(lookahead_s, self.spline_y, der=1)
        angle_to_goal = np.arctan2(dy_ds, dx_ds)  # Góc của tiếp tuyến spline
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

class GUI:
    def __init__(self, master, mpc_controller):
        self.master = master
        master.title("MiR100 Spline Tracking")

        self.mpc_controller = mpc_controller

        self.start_button = tk.Button(master, text="Start Tracking", command=self.start_control_loop)
        self.start_button.grid(row=0, column=0)
        self.stop_button = tk.Button(master, text="Stop Tracking", command=self.stop_control_loop, state=tk.DISABLED)
        self.stop_button.grid(row=0, column=1)
        self.clear_button = tk.Button(master, text="Clear Waypoints", command=self.clear_waypoints)
        self.clear_button.grid(row=0, column=2)

        self.result_label = tk.Label(master, text="")
        self.result_label.grid(row=1, column=0, columnspan=3)

        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

        self.control_loop_running = False

        # Tab control
        self.tabControl = ttk.Notebook(master)

        self.trajectory_tab = tk.Frame(self.tabControl)
        self.velocity_tab = tk.Frame(self.tabControl)
        self.acceleration_tab = tk.Frame(self.tabControl)

        self.tabControl.add(self.trajectory_tab, text='Trajectory')
        self.tabControl.add(self.velocity_tab, text='Velocity')
        self.tabControl.add(self.acceleration_tab, text='Acceleration')
        self.tabControl.grid(row=2, column=0, columnspan=3)

        # Trajectory Tab
        self.fig_trajectory, self.ax_trajectory = plt.subplots()
        self.ax_trajectory.set_xlim([0, 20])
        self.ax_trajectory.set_ylim([0, 20])
        self.ax_trajectory.set_xlabel("X (m)")
        self.ax_trajectory.set_ylabel("Y (m)")
        self.ax_trajectory.set_title("Robot Trajectory")
        self.line, = self.ax_trajectory.plot([], [], label="Robot Path")
        self.waypoints_plot, = self.ax_trajectory.plot([], [], 'bo', label="Waypoints")
        self.robot_plot, = self.ax_trajectory.plot([], [], 'go', label="Robot", markersize=10)
        self.spline_plot, = self.ax_trajectory.plot([], [], 'g-', label="Spline", linewidth=2)
        self.lookahead_plot, = self.ax_trajectory.plot([], [], 'yo', label="Lookahead Point", markersize=5)
        self.ax_trajectory.legend()

        self.canvas_trajectory = FigureCanvasTkAgg(self.fig_trajectory, master=self.trajectory_tab)
        self.canvas_trajectory.draw()
        self.canvas_trajectory.get_tk_widget().grid(row=0, column=0, columnspan=3)

        self.fig_trajectory.canvas.mpl_connect('button_press_event', self.onclick)

        # Velocity Tab
        self.fig_velocity, self.ax_velocity = plt.subplots()
        self.ax_velocity.set_xlabel("Time (s)")
        self.ax_velocity.set_ylabel("Velocity (m/s, rad/s)")
        self.ax_velocity.set_title("Robot Velocity")
        self.line_linear_velocity, = self.ax_velocity.plot([], [], label="Linear Velocity")
        self.line_angular_velocity, = self.ax_velocity.plot([], [], label="Angular Velocity")
        self.ax_velocity.legend()

        self.canvas_velocity = FigureCanvasTkAgg(self.fig_velocity, master=self.velocity_tab)
        self.canvas_velocity.draw()
        self.canvas_velocity.get_tk_widget().grid(row=0, column=0, columnspan=3)

        # Acceleration Tab
        self.fig_acceleration, self.ax_acceleration = plt.subplots()
        self.ax_acceleration.set_xlabel("Time (s)")
        self.ax_acceleration.set_ylabel("Acceleration (m/s^2, rad/s^2)")
        self.ax_acceleration.set_title("Robot Acceleration")
        self.line_linear_acceleration, = self.ax_acceleration.plot([], [], label="Linear Acceleration")
        self.line_angular_acceleration, = self.ax_acceleration.plot([], [], label="Angular Acceleration")
        self.ax_acceleration.legend()

        self.canvas_acceleration = FigureCanvasTkAgg(self.fig_acceleration, master=self.acceleration_tab)
        self.canvas_acceleration.draw()
        self.canvas_acceleration.get_tk_widget().grid(row=0, column=0, columnspan=3)


    def onclick(self, event):
        if event.xdata is not None and event.ydata is not None:
            x = event.xdata
            y = event.ydata

            if len(self.mpc_controller.waypoints_x) >= 1:
                self.mpc_controller.waypoints_x.append(x)
                self.mpc_controller.waypoints_y.append(y)
            self.update_waypoint_plot()
            print(f"Clicked at x={x}, y={y}")

    def start_control_loop(self):
        if not self.mpc_controller.calculate_spline():
            self.result_label.config(text="Error: Could not create spline. Add more waypoints.")
            return

        self.start_button.config(state=tk.DISABLED)
        self.stop_button.config(state=tk.NORMAL)
        self.clear_button.config(state=tk.DISABLED)

        self.mpc_controller.trajectory_x = []
        self.mpc_controller.trajectory_y = []
        self.mpc_controller.reached_goal = False

        # Reset velocity and acceleration data
        self.mpc_controller.velocity_data = []
        self.mpc_controller.angular_velocity_data = []
        self.mpc_controller.acceleration_data = []
        self.mpc_controller.angular_acceleration_data = []
        self.mpc_controller.time_data = []
        self.mpc_controller.start_time = rospy.Time.now().to_sec()

        self.control_loop_running = True

        self.update_plot()

    def stop_control_loop(self):
        self.control_loop_running = False
        self.start_button.config(state=tk.NORMAL)
        self.stop_button.config(state=tk.DISABLED)
        self.clear_button.config(state=tk.NORMAL)
        self.send_command(0.0, 0.0)

    def clear_waypoints(self):
        self.mpc_controller.waypoints_x = []
        self.mpc_controller.waypoints_y = []
        self.mpc_controller.spline_x = None
        self.mpc_controller.spline_y = None
        self.waypoints_plot.set_data([], [])
        self.spline_plot.set_data([], [])
        self.line.set_data([], [])
        self.robot_plot.set_data([], [])
        self.lookahead_plot.set_data([], [])
        self.ax_trajectory.relim()
        self.ax_trajectory.autoscale_view()
        self.canvas_trajectory.draw_idle()

    def update_waypoint_plot(self):
        self.waypoints_plot.set_data(self.mpc_controller.waypoints_x, self.mpc_controller.waypoints_y)
        self.ax_trajectory.relim()
        self.ax_trajectory.autoscale_view()
        self.canvas_trajectory.draw_idle()

    def update_plot(self):
        if not self.control_loop_running:
            return

        if self.mpc_controller.current_pose is None:
            self.result_label.config(text="Waiting for initial pose from /amcl_pose...")
            self.master.after(10, self.update_plot)
            return

        x = self.mpc_controller.x
        y = self.mpc_controller.y
        theta = self.mpc_controller.theta

        v, omega = self.mpc_controller.mpc_control(x, y, theta)
        self.send_command(v, omega)

        # Trajectory Tab
        self.line.set_data(self.mpc_controller.trajectory_x, self.mpc_controller.trajectory_y)
        self.robot_plot.set_data([x], [y])

        if self.mpc_controller.spline_x is not None and self.mpc_controller.spline_y is not None:
            s_spline = np.linspace(0, self.mpc_controller.arclength, 1000)
            x_spline = splev(s_spline, self.mpc_controller.spline_x)
            y_spline = splev(s_spline, self.mpc_controller.spline_y)
            self.spline_plot.set_data(x_spline, y_spline)

            closest_s = self.mpc_controller.find_closest_point_on_spline(x, y)
            lookahead_s = closest_s + self.mpc_controller.lookahead_distance
            if lookahead_s > self.mpc_controller.arclength:
                lookahead_s = self.mpc_controller.arclength
            elif lookahead_s < 0:
                lookahead_s = 0

            lookahead_x = splev(lookahead_s, self.mpc_controller.spline_x)
            lookahead_y = splev(lookahead_s, self.mpc_controller.spline_y)

            self.lookahead_plot.set_data([lookahead_x], [lookahead_y])

        self.update_waypoint_plot()
        self.ax_trajectory.relim()
        self.ax_trajectory.autoscale_view()
        self.canvas_trajectory.draw_idle()

        # Velocity Tab
        # Apply low-pass filter to velocity data
        filtered_velocity_data = self.mpc_controller.lowpass_filter(self.mpc_controller.velocity_data)
        filtered_angular_velocity_data = self.mpc_controller.lowpass_filter(self.mpc_controller.angular_velocity_data)

        self.line_linear_velocity.set_data(self.mpc_controller.time_data, filtered_velocity_data)
        self.line_angular_velocity.set_data(self.mpc_controller.time_data, filtered_angular_velocity_data)
        self.ax_velocity.relim()
        self.ax_velocity.autoscale_view()
        self.canvas_velocity.draw_idle()

        # Acceleration Tab
        self.line_linear_acceleration.set_data(self.mpc_controller.time_data, self.mpc_controller.acceleration_data)
        self.line_angular_acceleration.set_data(self.mpc_controller.time_data, self.mpc_controller.angular_acceleration_data)
        self.ax_acceleration.relim()
        self.ax_acceleration.autoscale_view()
        self.canvas_acceleration.draw_idle()

        self.master.update()

        if not self.mpc_controller.reached_goal:
            self.master.after(10, self.update_plot)
        else:
            self.stop_control_loop()

    def send_command(self, v, omega):
        twist_msg = Twist()
        twist_msg.linear.x = v
        twist_msg.angular.z = omega
        self.cmd_vel_pub.publish(twist_msg)
        print(f"Published to /cmd_vel: linear.x = {v:.6f}, angular.z = {omega:.6f}")

def ros_spin():
    rospy.spin()

if __name__ == '__main__':
    rospy.init_node('spline_tracking_node')

    dt = 0.1
    v_max = 2
    v_min = -2
    omega_max = 2
    omega_min = -2
    lookahead_distance = 0.1

    controller = MPCController(dt, v_max, v_min, omega_max, omega_min, lookahead_distance)

    rospy.Subscriber('/amcl_pose', PoseWithCovarianceStamped, controller.pose_callback)

    root = tk.Tk()
    gui = GUI(root, controller)

    threading.Thread(target=ros_spin, daemon=True).start()

    root.mainloop()