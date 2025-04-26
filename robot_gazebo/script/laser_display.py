import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import matplotlib.pyplot as plt
import numpy as np
from rclpy.qos import QoSProfile, QoSReliabilityPolicy

class LidarVisualizer(Node):
    def __init__(self):
        super().__init__('lidar_visualizer')
        
        # เพิ่มเข้าไปใน constructor
        qos = QoSProfile(depth=10)
        qos.reliability = QoSReliabilityPolicy.BEST_EFFORT
        
        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.listener_callback,
            qos
        )
        plt.ion()  # เปิด interactive mode
        self.fig, self.ax = plt.subplots()

    def listener_callback(self, msg: LaserScan):
        angles = np.linspace(msg.angle_min, msg.angle_max, len(msg.ranges))
        ranges = np.array(msg.ranges)

        # ล้างข้อมูลที่เป็น inf/NaN
        ranges = np.nan_to_num(ranges, nan=0.0, posinf=0.0, neginf=0.0)

        # คำนวณตำแหน่ง x,y
        x = ranges * np.cos(angles)
        y = ranges * np.sin(angles)

        # เคลียร์กราฟเก่า แล้ววาดใหม่
        self.ax.clear()
        self.ax.plot(x, y, 'b.')
        self.ax.set_title('/scan topic (Lidar)')
        self.ax.set_xlabel('X (m)')
        self.ax.set_ylabel('Y (m)')
        self.ax.set_xlim(-10, 10)
        self.ax.set_ylim(-10, 10)
        self.ax.grid(True)
        plt.pause(0.001)

def main(args=None):
    rclpy.init(args=args)
    node = LidarVisualizer()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print('Shutting down...')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
