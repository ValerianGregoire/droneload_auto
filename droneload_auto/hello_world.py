#!/usr/bin/env python3

# This is an example node
# The line 'hello_world = droneload_auto.hello_world:main' must be added
# to the entry points in setup.py for the node to be runnable
"""
entry_points={
        'console_scripts': [
        	'hello_world = droneload_auto.hello_world:main'
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
ros2 run droneload_auto hello_world
"""
# You should see "hello, world" printed to the console

import rclpy
from rclpy.node import Node

class HelloNode(Node):
    def __init__(self):
        super().__init__('hello_world')
        self.get_logger().info('hello, world')

        rclpy.shutdown() # Additionnal command to stop the spin

def main(args=None):
    rclpy.init(args=args)

    node = HelloNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('KeyboardInterrupt received. Shutting down...')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
