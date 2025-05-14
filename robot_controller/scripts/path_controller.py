#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from visualization_msgs.msg import Marker, MarkerArray
import yaml
import math
import os

class PID:
    def __init__(self, kp, ki, kd, dt):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.dt = dt
        self.integral = 0.0
        self.prev_error = 0.0

    def reset(self):
        self.integral = 0.0
        self.prev_error = 0.0

    def update(self, error):
        self.integral += error * self.dt
        derivative = (error - self.prev_error) / self.dt
        output = self.kp * error + self.ki * self.integral + self.kd * derivative
        self.prev_error = error
        return output

class PathFollower(Node):
    def __init__(self):
        super().__init__('path_follower')
        # Declare parameters
        self.declare_parameter('path_file', '/home/chayanin09/open_topic/src/OpenTopic_mapping_simulation/robot_controller/paths/path.yaml')
        self.declare_parameter('kp_lin', 0.5)
        self.declare_parameter('ki_lin', 0.01)
        self.declare_parameter('kd_lin', 0.0)
        self.declare_parameter('kp_ang', 0.4)
        self.declare_parameter('ki_ang', 0.01)
        self.declare_parameter('kd_ang', 0.0)
        self.declare_parameter('goal_tolerance', 0.5)
        self.declare_parameter('control_rate', 20.0)

        # Load parameters
        path_file = self.get_parameter('path_file').get_parameter_value().string_value
        self.goal_tolerance = self.get_parameter('goal_tolerance').get_parameter_value().double_value
        control_rate = self.get_parameter('control_rate').get_parameter_value().double_value
        dt = 1.0 / control_rate

        # Initialize PID controllers
        self.pid_lin = PID(
            self.get_parameter('kp_lin').get_parameter_value().double_value,
            self.get_parameter('ki_lin').get_parameter_value().double_value,
            self.get_parameter('kd_lin').get_parameter_value().double_value, dt
        )
        self.pid_ang = PID(
            self.get_parameter('kp_ang').get_parameter_value().double_value,
            self.get_parameter('ki_ang').get_parameter_value().double_value,
            self.get_parameter('kd_ang').get_parameter_value().double_value, dt
        )

        # Try to load path
        try:
            with open(path_file, 'r') as f:
                data = yaml.safe_load(f)
                self.waypoints = data.get('path', [])
                if not self.waypoints:
                    raise ValueError("Path list is empty")
            self.get_logger().info(f"Loaded {len(self.waypoints)} waypoints from {path_file}")
        except Exception as e:
            self.get_logger().error(f"Failed to load path file '{path_file}': {e}")
            rclpy.shutdown()
            return

        # Create publisher for cmd_vel and markers
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.marker_pub = self.create_publisher(MarkerArray, '/path_markers', 10)

        # Subscriber for ground truth odom
        self.current_pose = None
        self.create_subscription(Odometry, '/base_pose_ground_truth', self.odom_callback, 10)

        # Publish initial path markers
        self.publish_path_markers()

        # Control timer
        self.timer = self.create_timer(dt, self.control_loop)

    def odom_callback(self, msg):
        self.current_pose = msg.pose.pose

    def publish_path_markers(self):
        marker_array = MarkerArray()
        for idx, wp in enumerate(self.waypoints):
            marker = Marker()
            marker.header.frame_id = 'gt_odom'
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = 'path'
            marker.id = idx
            marker.type = Marker.ARROW
            marker.action = Marker.ADD
            marker.pose.position.x = wp['x']
            marker.pose.position.y = wp['y']
            marker.pose.position.z = 0.1
            yaw = wp['yaw']
            # quaternion from yaw
            marker.pose.orientation.x = 0.0
            marker.pose.orientation.y = 0.0
            marker.pose.orientation.z = math.sin(yaw / 2.0)
            marker.pose.orientation.w = math.cos(yaw / 2.0)
            # arrow scale
            marker.scale.x = 0.4  # length
            marker.scale.y = 0.1  # width
            marker.scale.z = 0.1  # height
            # color (green)
            marker.color.r = 0.0
            marker.color.g = 1.0
            marker.color.b = 0.0
            marker.color.a = 1.0
            marker_array.markers.append(marker)
        self.marker_pub.publish(marker_array)
        self.get_logger().info('Published path markers to /path_markers')

    def control_loop(self):
        if self.current_pose is None or not self.waypoints:
            return

        # Current pose
        x_cur = self.current_pose.position.x
        y_cur = self.current_pose.position.y
        q = self.current_pose.orientation
        yaw_cur = math.atan2(
            2.0*(q.w*q.z + q.x*q.y),
            1.0 - 2.0*(q.y*q.y + q.z*q.z)
        )

        # Target waypoint
        target = self.waypoints[0]
        x_tgt = target['x']
        y_tgt = target['y']

        # Compute errors
        dx = x_tgt - x_cur
        dy = y_tgt - y_cur
        dist_error = math.hypot(dx, dy)
        angle_error = math.atan2(dy, dx) - yaw_cur
        angle_error = math.atan2(math.sin(angle_error), math.cos(angle_error))

        # Check if reached waypoint
        if dist_error < self.goal_tolerance:
            self.get_logger().info(f"Reached waypoint: x={x_tgt:.2f}, y={y_tgt:.2f}")
            self.waypoints.pop(0)
            self.pid_lin.reset()
            self.pid_ang.reset()
            # Update markers to remove passed waypoint
            self.publish_path_markers()
            if not self.waypoints:
                self.get_logger().info("All waypoints reached, stopping.")
                self.cmd_pub.publish(Twist())
            return

        # PID control
        v = self.pid_lin.update(dist_error)
        w = self.pid_ang.update(angle_error)

        if v > 0.4:
            v = 0.4
        # Saturation for angular velocity
        w_max = 1.0  # ตั้งได้ เช่น 1.0 rad/s
        if w > w_max:
            w = w_max
        elif w < -w_max:
            w = -w_max
        
        # Publish command
        cmd = Twist()
        cmd.linear.x = v
        cmd.angular.z = w
        self.cmd_pub.publish(cmd)

def main(args=None):
    rclpy.init(args=args)
    node = PathFollower()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
