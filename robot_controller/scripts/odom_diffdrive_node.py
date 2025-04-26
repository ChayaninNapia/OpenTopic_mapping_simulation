#!/usr/bin/env python3

import math
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import QoSProfile, QoSDurabilityPolicy
from sensor_msgs.msg import JointState
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped, Twist
from std_msgs.msg import Float64MultiArray
from tf2_ros import TransformBroadcaster


class DiffDriveNode(Node):
    def __init__(self):
        super().__init__('diff_drive_node')

        # use simulation time
        self.set_parameters([Parameter('use_sim_time', Parameter.Type.BOOL, True)])

        # robot parameters
        self.declare_parameter('wheel_radius', 0.0625)
        self.declare_parameter('wheel_separation', 0.222604)
        self.declare_parameter('left_joint', 'left_wheel_joint')
        self.declare_parameter('right_joint', 'right_wheel_joint')
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_footprint')

        self.wheel_radius     = self.get_parameter('wheel_radius').value
        self.wheel_separation = self.get_parameter('wheel_separation').value
        self.left_joint       = self.get_parameter('left_joint').value
        self.right_joint      = self.get_parameter('right_joint').value
        self.odom_frame       = self.get_parameter('odom_frame').value
        self.base_frame       = self.get_parameter('base_frame').value

        # odometry state
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.last_time = self.get_clock().now()
        self.last_left = 0.0
        self.last_right = 0.0

        # PID setpoints & state
        self.des_omega_l = 0.0
        self.des_omega_r = 0.0
        self.Kp = 2.0
        self.Ki = 0.0
        self.Kd = 0.0
        self.i_err_l = 0.0
        self.i_err_r = 0.0
        self.prev_err_l = 0.0
        self.prev_err_r = 0.0

        # publishers & subscribers
        self.cmd_pub        = self.create_publisher(Float64MultiArray,
                                        'velocity_controller/commands', 10)
        self.odom_pub       = self.create_publisher(Odometry, 'odom', 10)
        self.tf_broadcaster = TransformBroadcaster(self)

        self.create_subscription(
            Twist, 'cmd_vel', self.cmd_vel_cb, 10)
        qos = QoSProfile(depth=10, durability=QoSDurabilityPolicy.VOLATILE)
        self.create_subscription(
            JointState, 'joint_states', self.joint_state_cb, qos)

        self.get_logger().info('DiffDriveNode (PID wheel control + odometry) initialized')

    def cmd_vel_cb(self, msg: Twist):
        # compute desired wheel angular velocities
        v = msg.linear.x
        w = msg.angular.z
        v_l = v - w * self.wheel_separation / 2.0
        v_r = v + w * self.wheel_separation / 2.0
        self.des_omega_l = v_l / self.wheel_radius
        self.des_omega_r = v_r / self.wheel_radius

    def joint_state_cb(self, msg: JointState):
        # find indices of our wheels
        if self.left_joint not in msg.name or self.right_joint not in msg.name:
            return
        l_idx = msg.name.index(self.left_joint)
        r_idx = msg.name.index(self.right_joint)
        left_pos  = msg.position[l_idx]
        right_pos = msg.position[r_idx]

        now = self.get_clock().now()
        dt = (now - self.last_time).nanoseconds * 1e-9
        if dt <= 0.0:
            return
        # ถ้าไม่มีคำสั่งเคลื่อนที่ (ทั้งสอง = 0) ก็ข้าม controller
        # if abs(self.des_omega_l) < 1e-6 and abs(self.des_omega_r) < 1e-6:
        #     # รีเซ็ต integrator เพื่อป้องกัน wind-up
        #     self.i_err_l = 0.0
        #     self.i_err_r = 0.0
        #     self.prev_err_l = 0.0
        #     self.prev_err_r = 0.0
        #     return

        # compute delta rotations (wrap)
        delta_left  = self._normalize_angle(left_pos  - self.last_left)
        delta_right = self._normalize_angle(right_pos - self.last_right)

        # measure actual wheel angular velocities
        omega_l_meas = delta_left  / dt
        omega_r_meas = delta_right / dt

        # PID left wheel
        err_l = self.des_omega_l - omega_l_meas
        self.i_err_l += err_l * dt
        d_err_l = (err_l - self.prev_err_l) / dt
        ctrl_l = self.Kp*err_l + self.Ki*self.i_err_l + self.Kd*d_err_l
        self.prev_err_l = err_l

        # PID right wheel
        err_r = self.des_omega_r - omega_r_meas
        self.i_err_r += err_r * dt
        d_err_r = (err_r - self.prev_err_r) / dt
        ctrl_r = self.Kp*err_r + self.Ki*self.i_err_r + self.Kd*d_err_r
        self.prev_err_r = err_r

        # publish wheel commands
        cmd = Float64MultiArray()
        cmd.data = [ctrl_l, ctrl_r]
        self.cmd_pub.publish(cmd)

        # --- forward kinematics / odometry ---
        d_left  = delta_left  * self.wheel_radius
        d_right = delta_right * self.wheel_radius
        d_center = (d_left + d_right) / 2.0
        d_theta  = (d_right - d_left) / self.wheel_separation

        self.x += d_center * math.cos(self.theta + d_theta/2.0)
        self.y += d_center * math.sin(self.theta + d_theta/2.0)
        self.theta += d_theta

        vx  = d_center / dt
        vth = d_theta  / dt

        # publish Odometry message
        odom = Odometry()
        odom.header.stamp    = now.to_msg()
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id  = self.base_frame
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.orientation = self._euler_to_quat(0,0,self.theta)
        odom.twist.twist.linear.x  = vx
        odom.twist.twist.angular.z = vth
        self.odom_pub.publish(odom)

        # broadcast TF odom->base
        tf = TransformStamped()
        tf.header.stamp    = now.to_msg()
        tf.header.frame_id = self.odom_frame
        tf.child_frame_id  = self.base_frame
        tf.transform.translation.x = self.x
        tf.transform.translation.y = self.y
        tf.transform.rotation      = odom.pose.pose.orientation
        self.tf_broadcaster.sendTransform(tf)

        # save state
        self.last_time  = now
        self.last_left  = left_pos
        self.last_right = right_pos

    def _normalize_angle(self, ang):
        return math.atan2(math.sin(ang), math.cos(ang))

    def _euler_to_quat(self, roll, pitch, yaw):
        qz = math.sin(yaw/2.0)
        qw = math.cos(yaw/2.0)
        from geometry_msgs.msg import Quaternion
        return Quaternion(x=0.0, y=0.0, z=qz, w=qw)


def main(args=None):
    rclpy.init(args=args)
    node = DiffDriveNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
