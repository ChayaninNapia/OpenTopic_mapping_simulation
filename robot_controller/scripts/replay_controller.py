#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from rclpy.serialization import deserialize_message
from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions

class ControllerReplay(Node):
    def __init__(self):
        super().__init__('controller_replay')
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        self.timer = self.create_timer(0.1, self.timer_callback)

        # Set the bag file path here (adjust accordingly)
        bag_path = '/path/to/your/bag'
        storage_options = StorageOptions(uri=bag_path, storage_id='sqlite3')
        converter_options = ConverterOptions('', '')

        # Open the bag file using rosbag2_py
        self.reader = SequentialReader()
        self.reader.open(storage_options, converter_options)
        # Optionally, set a filter to read only the cmd_vel topic
        self.reader.set_filter({'topic': 'cmd_vel'})
        
        # Read all messages from the bag file into a buffer
        self.msg_buffer = []
        while self.reader.has_next():
            topic, data, t = self.reader.read_next()
            if topic == 'cmd_vel':
                twist_msg = deserialize_message(data, Twist)
                self.msg_buffer.append((t, twist_msg))
        self.get_logger().info(f"Loaded {len(self.msg_buffer)} cmd_vel messages from bag.")
        self.index = 0

    def timer_callback(self):
        if self.index < len(self.msg_buffer):
            _, msg = self.msg_buffer[self.index]
            self.publisher_.publish(msg)
            self.get_logger().info('Publishing cmd_vel message')
            self.index += 1
        else:
            self.get_logger().info('Finished replaying bag messages.')
            self.destroy_timer(self.timer)
            # Optionally, shutdown the node if finished
            rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    node = ControllerReplay()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
