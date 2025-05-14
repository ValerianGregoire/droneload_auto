#!/usr/bin/env python3

# This node detects arucos using an image
# The line 'aruco_detect = droneload_auto.aruco_detect:main' must be added
# to the entry points in setup.py for the node to be runnable
"""
entry_points={
        'console_scripts': [
        	'aruco_detect = droneload_auto.aruco_detect:main'
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
ros2 run droneload_auto aruco_detect
"""
# The lidar data should be written to the 'lidar_data' topic every 10 ms

import rclpy
from rclpy.node import Node
import cv2
import numpy as np
from droneload_interfaces.msg import ArucoData
from sensor_msgs.msg import Image


class ArucoDetect(Node):
    def __init__(self):
        super().__init__('aruco_detect')
        self.get_logger().info('Aruco detector initialized')
        self.arucos = list()
        self.x = list()
        self.y = list()

        self.r = self.g = self.b = None
        self.width = self.height = None
        self.image = None

        ##### SUBSCRIPTION BLOCK START #####
        self.timestamp = None
        self.subs_data = None
        self.subscription = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.listener_callback,
            1)
        self.subscription
        ##### SUBSCRIPTION BLOCK END #####


        #### PUBLICATION BLOCK START ####
        self.publisher = self.create_publisher(ArucoData, 'aruco_data', 2)
        #### PUBLICATION BLOCK END ####


        #### ARUCO DETECTION BLOCK START ####
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_100)
        self.parameters = cv2.aruco.DetectorParameters()
        self.detector = cv2.aruco.ArucoDetector(self.aruco_dict, self.parameters)
        #### ARUCO DETECTION BLOCK END ####

    def listener_callback(self, msg):
        # Extract timestamp from header
        self.timestamp = msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec

        # Get image dimensions
        self.width = msg.width
        self.height = msg.height

        # Convert image data to NumPy array
        image_data = np.frombuffer(msg.data, dtype=np.uint8)

        # Determine image format and reshape accordingly
        encoding = msg.encoding.upper()
        if encoding in ['RGB8', 'BGR8']:
            channels = 3
        elif encoding == 'MONO8':
            channels = 1
        else:
            self.get_logger().error(f"Unsupported image encoding: {msg.encoding}")
            return

        expected_size = self.height * self.width * channels
        if image_data.size != expected_size:
            self.get_logger().error(
                f"Image data size mismatch: got {image_data.size}, expected {expected_size}")
            return

        try:
            self.image = image_data.reshape((self.height, self.width, channels)) \
                if channels > 1 else image_data.reshape((self.height, self.width))
        except ValueError as e:
            self.get_logger().error(f"Image reshaping failed: {e}")
            return
        
        if encoding == 'RGB8':
            self.image = cv2.cvtColor(self.image, cv2.COLOR_RGB2BGR)

        self.get_logger().info(f"Received image at timestamp: {self.timestamp}")

        # Aruco detection
        x, y, ids, scales = self.detect_aruco_positions()

        # aruco_data topic publication
        self.publish_data(x, y, ids, scales)

    def publish_data(self, x, y, ids, scales):
        msg = ArucoData()
        msg.x = x.copy()
        msg.y = y.copy()
        msg.ids = ids.copy()
        msg.scales = scales.copy()
        msg.timestamp = self.get_clock().now().nanoseconds
        self.publisher.publish(msg)

    def detect_aruco_positions(self):

        # Detect the markers
        corners, ids, _ = self.detector.detectMarkers(self.image)

        markers_x = []
        markers_y = []
        detected_ids = []
        scales = []

        if ids is not None:
            total_area = self.width * self.height

            for corner, marker_id in zip(corners, ids.flatten()):
                # Compute the center point of the marker
                c = corner[0]
                center_x = np.mean(c[:, 0])
                center_y = np.mean(c[:, 1])

                # Normalize between 0 and 1
                norm_x = center_x / self.width
                norm_y = center_y / self.height

                # Compute the area of the marker using the shoelace formula
                area = 0.5 * np.abs(
                    c[0,0]*c[1,1] + c[1,0]*c[2,1] + c[2,0]*c[3,1] + c[3,0]*c[0,1]
                    - c[1,0]*c[0,1] - c[2,0]*c[1,1] - c[3,0]*c[2,1] - c[0,0]*c[3,1]
                )

                # Normalize the area between 0 and 1
                norm_area = area / total_area

                markers_x.append(norm_x)
                markers_y.append(norm_y)
                detected_ids.append(marker_id)
                scales.append(norm_area)

        return markers_x, markers_y, detected_ids, scales

def main(args=None):
    rclpy.init(args=args)

    node = ArucoDetect()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('KeyboardInterrupt received. Shutting down...')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
