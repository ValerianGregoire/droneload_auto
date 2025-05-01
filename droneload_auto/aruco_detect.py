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

class LidarRead(Node):
    def __init__(self):
        super().__init__('aruco_detect')
        self.get_logger().info('Aruco detector initialized')
        self.arucos = list()
        self.x = list()
        self.y = list()

    def detect_aruco_positions(self, image_path):
        # Load the image
        image = cv2.imread(image_path)
        if image is None:
            raise FileNotFoundError(f"Image at path {image_path} could not be loaded.")
        
        # Get image dimensions
        height, width, _ = image.shape

        # Define the dictionary you are using (4x4_50 is a common choice)
        aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
        parameters = cv2.aruco.DetectorParameters()

        # Create the detector
        detector = cv2.aruco.ArucoDetector(aruco_dict, parameters)

        # Detect the markers
        corners, ids, _ = detector.detectMarkers(image)

        positions = []
        detected_ids = []

        if ids is not None:
            for corner, marker_id in zip(corners, ids.flatten()):
                # Compute the center point of the marker
                c = corner[0]
                center_x = np.mean(c[:, 0])
                center_y = np.mean(c[:, 1])

                # Normalize between 0 and 1
                norm_x = center_x / width
                norm_y = center_y / height

                positions.append((norm_x, norm_y))
                detected_ids.append(marker_id)

        return positions, detected_ids




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
