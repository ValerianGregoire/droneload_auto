#!/usr/bin/env python3

# This node reads the lidar data
# The line 'lidar_read = droneload_auto.lidar_read:main' must be added
# to the entry points in setup.py for the node to be runnable
"""
entry_points={
        'console_scripts': [
        	'lidar_read = droneload_auto.lidar_read:main'
        ],
    }
"""

# After every modification, the package must be rebuilt using
"""
cd ~/ros2_ws/
colcon build --packages-select droneload_auto
"""

# The node can be run with
"""
cd ~/ros2_ws/
ros2 run droneload_auto lidar_read
"""
# The lidar data should be written to the 'lidar_data' topic every 10 ms

import rclpy
from rclpy.node import Node

class LidarRead(Node):
    def __init__(self):
        super().__init__('lidar_read')
        self.get_logger().info('Lidar reader initialized')




def main(args=None):
    rclpy.init(args=args)

    node = LidarRead()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('KeyboardInterrupt received. Shutting down...')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
