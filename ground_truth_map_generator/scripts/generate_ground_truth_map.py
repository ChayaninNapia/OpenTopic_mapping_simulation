#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from gazebo_map_creator_interface.srv import MapRequest
import yaml
import argparse
from geometry_msgs.msg import Point

class MapRequester(Node):
    def __init__(self, config):
        super().__init__('map_requester')
        self.cli = self.create_client(MapRequest, '/world/save_map')

        while not self.cli.wait_for_service(timeout_sec=2.0):
            self.get_logger().info('Waiting for /world/save_map service...')

        self.req = MapRequest.Request()
        self.load_config(config)

    def load_config(self, config):
        self.req.resolution = config['resolution']
        self.req.filename = config['filename']
        self.req.skip_vertical_scan = config.get('skip_vertical_scan', False)
        self.req.threshold_2d = config.get('threshold_2d', 100)
        self.req.range_multiplier = config.get('range_multiplier', 1.0)

        # Convert to geometry_msgs/Point
        self.req.lowerright.x = config['lowerright'][0]
        self.req.lowerright.y = config['lowerright'][1]
        self.req.lowerright.z = config['lowerright'][2]

        self.req.upperleft.x = config['upperleft'][0]
        self.req.upperleft.y = config['upperleft'][1]
        self.req.upperleft.z = config['upperleft'][2]

    def send_request(self):
        future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, future)
        if future.result() is not None:
            if future.result().success:
                self.get_logger().info('✅ Map saved successfully!')
            else:
                self.get_logger().error('❌ Map generation failed!')
        else:
            self.get_logger().error('🚫 Service call failed.')

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument(
        '-c', '--config',
        default='/home/chayanin09/open_topic/src/OpenTopic_mapping_simulation/ground_truth_map_generator/config/map_config.yaml',
        help='Path to config file'
    )
    args = parser.parse_args()

    with open(args.config, 'r') as f:
        config = yaml.safe_load(f)

    rclpy.init()
    node = MapRequester(config)
    node.send_request()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
