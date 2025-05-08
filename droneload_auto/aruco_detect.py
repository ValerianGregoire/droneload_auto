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
from droneload_interfaces.msg import CameraData


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
            CameraData,
            'camera_data',
            self.listener_callback,
            1)
        self.subscription
        ##### SUBSCRIPTION BLOCK END #####


        #### PUBLICATION BLOCK START ####
        self.publisher = self.create_publisher(CameraData, 'aruco_data', 2)
        #### PUBLICATION BLOCK END ####


        #### ARUCO DETECTION BLOCK START ####
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_100)
        self.parameters = cv2.aruco.DetectorParameters()
        self.detector = cv2.aruco.ArucoDetector(self.aruco_dict, self.parameters)
        #### ARUCO DETECTION BLOCK END ####

    def listener_callback(self, msg):
        # Reads image data when published
        self.timestamp = msg.timestamp
        self.subs_data = msg
        self.width = msg.width
        self.height = msg.height
        self.image = np.array(msg.data).resize((self.width, self.height))
        self.get_logger().info(f"Collected image data at time: {self.timestamp}")
        
        # Aruco detection
        x, y, ids, scales = self.detect_aruco_positions()

        # aruco_data topic publication
        self.publish_data(x, y, ids, scales)

    def publish_data(self, x, y, ids, scales):
        msg = CameraData()
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
