#!/usr/bin/env python3

# This node detects pictures and elements from an image
# The line 'picture_detect = droneload_auto.picture_detect:main' must be added
# to the entry points in setup.py for the node to be runnable
"""
entry_points={
        'console_scripts': [
        	'picture_detect = droneload_auto.picture_detect:main'
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
ros2 run droneload_auto picture_detect
"""
# The lidar data should be written to the 'PictureData' topic when possible


import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Header
from cv_bridge import CvBridge

import cv2
import numpy as np
import os
import time

from ultralytics import YOLO
from ament_index_python.packages import get_package_share_directory

from droneload_interfaces.msg import PictureData  # ← Import your custom message


class PictureDetect(Node):
    def __init__(self):
        super().__init__('picture_detect')
        self.get_logger().info("Picture Detector Initialized")

        # === CONFIGURATION ===
        package_name = 'droneload_auto'  # Change if different
        model_file = 'yolov8_custom.pt'
        classes_file = 'classes.txt'
        self.image_topic = '/camera/image_raw'

        # === LOAD MODEL AND CLASSES ===
        package_path = get_package_share_directory(package_name)
        model_path = os.path.join(package_path, 'models', model_file)
        classes_path = os.path.join(package_path, 'models', classes_file)

        if not os.path.exists(model_path):
            self.get_logger().error(f"Model file not found: {model_path}")
            return

        if not os.path.exists(classes_path):
            self.get_logger().error(f"Classes file not found: {classes_path}")
            return

        self.model = YOLO(model_path)
        with open(classes_path, 'r') as f:
            self.class_names = [line.strip() for line in f.readlines()]

        self.bridge = CvBridge()

        # === SUBSCRIBER ===
        self.subscription = self.create_subscription(
            Image,
            self.image_topic,
            self.image_callback,
            10
        )

        # === PUBLISHER ===
        self.picture_data_pub = self.create_publisher(PictureData, '/picture_data', 10)

    def image_callback(self, msg: Image):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f"Image conversion error: {e}")
            return

        image_height, image_width = cv_image.shape[:2]
        results = self.model(cv_image, verbose=False)[0]

        if results.boxes is None or len(results.boxes) == 0:
            self.get_logger().info("No detections.")
            return

        x_centers = []
        y_centers = []
        labels = []

        for box in results.boxes:
            class_id = int(box.cls.item())
            conf = float(box.conf.item())
            class_name = self.class_names[class_id] if class_id < len(self.class_names) else f"id_{class_id}"

            x1, y1, x2, y2 = map(int, box.xyxy[0].tolist())
            x_center = (x1 + x2) / 2.0
            y_center = (y1 + y2) / 2.0

            # Normalize
            norm_x = float(x_center) / image_width
            norm_y = float(y_center) / image_height

            x_centers.append(norm_x)
            y_centers.append(norm_y)
            labels.append(class_name)

            self.get_logger().info(
                f"Detected {class_name} ({conf:.2f}) at norm pos ({norm_x:.3f}, {norm_y:.3f})"
            )

        # === CREATE AND PUBLISH PictureData ===
        picture_msg = PictureData()
        picture_msg.timestamp = int(self.get_clock().now().nanoseconds / 1e6)  # milliseconds
        picture_msg.x = x_centers
        picture_msg.y = y_centers
        picture_msg.labels = labels

        self.picture_data_pub.publish(picture_msg)


def main(args=None):
    rclpy.init(args=args)
    node = PictureDetect()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Node terminated by user.")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
