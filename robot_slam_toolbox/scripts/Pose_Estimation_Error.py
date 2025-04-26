#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped
import math
import csv

class PoseErrorCalculator(Node):
    def __init__(self):
        super().__init__('pose_error_calculator')
        # Subscribe to estimated pose from SLAM Toolbox
        self.subscription_estimated = self.create_subscription(
            PoseWithCovarianceStamped,
            '/pose',
            self.listener_callback_estimated,
            10)
        # Subscribe to ground truth pose (ต้องมี publication จาก simulator หรือ external sensor)
        self.subscription_ground_truth = self.create_subscription(
            PoseStamped,
            '/ground_truth_pose',
            self.listener_callback_ground_truth,
            10)
        self.estimated_pose = None
        self.ground_truth_pose = None
        self.errors = []  # เก็บ tuple (timestamp, translational_error, angular_error)

    def listener_callback_estimated(self, msg: PoseWithCovarianceStamped):
        self.estimated_pose = msg.pose.pose
        self.calculate_error()

    def listener_callback_ground_truth(self, msg: PoseStamped):
        self.ground_truth_pose = msg.pose
        self.calculate_error()

    def calculate_error(self):
        # ต้องมีข้อมูลทั้งสองก่อน
        if self.estimated_pose is None or self.ground_truth_pose is None:
            return

        # คำนวณ Translational Error
        dx = self.estimated_pose.position.x - self.ground_truth_pose.position.x
        dy = self.estimated_pose.position.y - self.ground_truth_pose.position.y
        trans_error = math.sqrt(dx * dx + dy * dy)

        # คำนวณ Angular Error โดยดึงเฉพาะ yaw จาก quaternion
        yaw_est = self.get_yaw(self.estimated_pose.orientation)
        yaw_gt = self.get_yaw(self.ground_truth_pose.orientation)
        angle_error = abs(self.angle_diff(yaw_est, yaw_gt))

        # บันทึก timestamp (ในที่นี้ใช้เวลาปัจจุบัน)
        timestamp = self.get_clock().now().nanoseconds / 1e9
        self.errors.append((timestamp, trans_error, angle_error))
        self.get_logger().info(
            f"Timestamp: {timestamp:.3f} | Translational Error: {trans_error:.3f} m | Angular Error: {angle_error:.3f} rad"
        )

    def get_yaw(self, q):
        # แปลง quaternion เป็น yaw โดยใช้สูตร: yaw = atan2(2*(w*z + x*y), 1 - 2*(y*y + z*z))
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)

    def angle_diff(self, a, b):
        diff = a - b
        while diff > math.pi:
            diff -= 2 * math.pi
        while diff < -math.pi:
            diff += 2 * math.pi
        return diff

    def save_errors(self, filename="pose_error_log.csv"):
        with open(filename, "w", newline='') as csvfile:
            writer = csv.writer(csvfile)
            writer.writerow(["timestamp", "translational_error", "angular_error"])
            for row in self.errors:
                writer.writerow(row)
        self.get_logger().info(f"Saved error log to {filename}")

def main(args=None):
    rclpy.init(args=args)
    node = PoseErrorCalculator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Keyboard Interrupt, saving logs ...")
        node.save_errors()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
