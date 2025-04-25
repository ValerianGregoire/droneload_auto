#!/usr/bin/env python3

# This node reads the camera data
# The line 'camera_read = droneload_auto.camera_read:main' must be added
# to the entry points in setup.py for the node to be runnable
"""
entry_points={
        'console_scripts': [
        	'camera_read = droneload_auto.camera_read:main'
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
ros2 run droneload_auto camera_read
"""
# The camera data should be written to the 'image' topic every 33 ms

import rclpy
from rclpy.node import Node

class CameraRead(Node):
    def __init__(self):
        super().__init__('camera_read')
        self.get_logger().info('Camera reader initialized')




def main(args=None):
    rclpy.init(args=args)

    node = CameraRead()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('KeyboardInterrupt received. Shutting down...')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
