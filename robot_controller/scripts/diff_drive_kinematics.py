#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TransformStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
import tf2_ros
import math

class DiffDriveKinematicsNode(Node):
    def __init__(self):
        super().__init__('diff_drive_kinematics')
        # พารามิเตอร์ (แก้ได้จากไฟล์ YAML/launch)
        self.declare_parameter('wheel_radius', 0.0625)
        self.declare_parameter('wheel_separation', 0.37)
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('joint_states_topic', '/joint_states')
        self.declare_parameter('wheel_cmd_topic', '/velocity_controller/commands')
        self.declare_parameter('odom_topic', '/wheel_odom')
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_footprint')

        # อ่านพารามิเตอร์
        p = self.get_parameter
        r = p('wheel_radius').get_parameter_value().double_value
        L = p('wheel_separation').get_parameter_value().double_value
        cmd_vel_topic = p('cmd_vel_topic').get_parameter_value().string_value
        joint_states_topic = p('joint_states_topic').get_parameter_value().string_value
        wheel_cmd_topic = p('wheel_cmd_topic').get_parameter_value().string_value
        odom_topic = p('odom_topic').get_parameter_value().string_value
        self.odom_frame = p('odom_frame').get_parameter_value().string_value
        self.base_frame = p('base_frame').get_parameter_value().string_value

        # เก็บค่า wheel params
        self.wheel_radius = r
        self.wheel_separation = L

        # Publishers / Subscribers
        self.wheel_cmd_pub = self.create_publisher(
            Float64MultiArray, wheel_cmd_topic, 10)
        self.odom_pub = self.create_publisher(Odometry, odom_topic, 10)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        self.create_subscription(Twist, cmd_vel_topic,
                                 self.cmd_vel_callback, 10)
        self.create_subscription(JointState, joint_states_topic,
                                 self.joint_states_callback, 10)

        # State for odometry integration
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.last_stamp = None

        self.left_name = 'left_wheel_joint'
        self.right_name = 'right_wheel_joint'

    def cmd_vel_callback(self, msg: Twist):
        # inverse kinematics → wheel velocities [rad/s]
        v = msg.linear.x
        omega = msg.angular.z
        r = self.wheel_radius
        L = self.wheel_separation
        v_left = (2*v - omega*L) / (2*r)
        v_right = (2*v + omega*L) / (2*r)

        fa = Float64MultiArray()
        fa.data = [v_left, v_right]
        self.wheel_cmd_pub.publish(fa)

    def joint_states_callback(self, msg: JointState):
        # หา index ของ joint สองล้อ
        try:
            il = msg.name.index(self.left_name)
            ir = msg.name.index(self.right_name)
        except ValueError:
            return

        vl = msg.velocity[il]
        vr = msg.velocity[ir]

        # เวลา
        t = msg.header.stamp
        if self.last_stamp is None:
            self.last_stamp = t
            return
        dt = (t.sec - self.last_stamp.sec) + (t.nanosec - self.last_stamp.nanosec) * 1e-9
        self.last_stamp = t

        # forward kinematics → v, omega
        r = self.wheel_radius
        L = self.wheel_separation
        v = r * (vr + vl) / 2.0
        omega = r * (vr - vl) / L

        # integrate pose
        self.x += v * math.cos(self.theta) * dt
        self.y += v * math.sin(self.theta) * dt
        self.theta += omega * dt

        # สร้างและ publish Odometry msg
        odom = Odometry()
        odom.header.stamp = t
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id = self.base_frame
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        qz = math.sin(self.theta / 2.0)
        qw = math.cos(self.theta / 2.0)
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw
        odom.twist.twist.linear.x = v
        odom.twist.twist.angular.z = omega
        self.odom_pub.publish(odom)

        # broadcast TF
        tf = TransformStamped()
        tf.header.stamp = t
        tf.header.frame_id = odom.header.frame_id
        tf.child_frame_id = odom.child_frame_id
        tf.transform.translation.x = self.x
        tf.transform.translation.y = self.y
        tf.transform.rotation.z = qz
        tf.transform.rotation.w = qw
        self.tf_broadcaster.sendTransform(tf)


def main(args=None):
    rclpy.init(args=args)
    node = DiffDriveKinematicsNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
