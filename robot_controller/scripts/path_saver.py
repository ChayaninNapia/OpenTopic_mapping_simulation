#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from std_srvs.srv import Trigger
from visualization_msgs.msg import Marker, MarkerArray
import yaml
import math
import os

"""
# To save the recorded path, call:
ros2 service call /save_path std_srvs/srv/Trigger {}
"""

class PathRecorder(Node):
    def __init__(self):
        super().__init__('path_recorder')
        # Parameter defining output YAML file path
        self.declare_parameter('output_file', '/home/chayanin09/open_topic/src/OpenTopic_mapping_simulation/robot_controller/paths/path.yaml')
        self.output_file = self.get_parameter('output_file').get_parameter_value().string_value

        # List to store incoming poses
        self.poses = []

        # Publisher for RViz markers
        self.marker_pub = self.create_publisher(MarkerArray, '/path_markers', 10)

        # Subscriber to /goal_pose
        self.create_subscription(
            PoseStamped,
            '/goal_pose',
            self.pose_callback,
            10
        )

        # Service to save path to YAML
        self.create_service(
            Trigger,
            '/save_path',
            self.save_path_callback
        )

        self.get_logger().info(f'PathRecorder initialized, will save to: {self.output_file}')

    def pose_callback(self, msg: PoseStamped):
        # Extract x, y, yaw
        x = msg.pose.position.x
        y = msg.pose.position.y
        q = msg.pose.orientation
        yaw = math.atan2(2.0*(q.w*q.z + q.x*q.y), 1.0 - 2.0*(q.y*q.y + q.z*q.z))
        frame_id = msg.header.frame_id

        # Store pose with frame information
        self.poses.append({'x': x, 'y': y, 'yaw': yaw, 'frame_id': frame_id})
        self.get_logger().info(f'Recorded pose #{len(self.poses)}: x={x:.2f}, y={y:.2f}, yaw={yaw:.2f}')

        # Publish visual markers for all recorded poses
        self.publish_markers()

    def publish_markers(self):
        marker_array = MarkerArray()
        for idx, p in enumerate(self.poses):
            marker = Marker()
            marker.header.frame_id = p['frame_id']
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = 'path_recorder'
            marker.id = idx
            marker.type = Marker.ARROW
            marker.action = Marker.ADD
            # Set position
            marker.pose.position.x = p['x']
            marker.pose.position.y = p['y']
            marker.pose.position.z = 0.0
            # Compute orientation from yaw
            yaw = p['yaw']
            marker.pose.orientation.x = 0.0
            marker.pose.orientation.y = 0.0
            marker.pose.orientation.z = math.sin(yaw / 2.0)
            marker.pose.orientation.w = math.cos(yaw / 2.0)
            # Arrow scale
            marker.scale.x = 0.3  # length
            marker.scale.y = 0.1  # width
            marker.scale.z = 0.1  # height
            # Color (red)
            marker.color.r = 1.0
            marker.color.g = 0.0
            marker.color.b = 0.0
            marker.color.a = 1.0
            marker_array.markers.append(marker)
        self.marker_pub.publish(marker_array)

    def save_path_callback(self, request, response):
        # Ensure directory exists
        os.makedirs(os.path.dirname(self.output_file), exist_ok=True)
        # Write YAML
        with open(self.output_file, 'w') as f:
            yaml.dump({'path': self.poses}, f)
        response.success = True
        response.message = f'Path saved with {len(self.poses)} poses to {self.output_file}'
        self.get_logger().info(response.message)
        return response


def main(args=None):
    rclpy.init(args=args)
    node = PathRecorder()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
