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

from ultralytics import YOLO
import cv2
import numpy as np
import os
import time

from ament_index_python.packages import get_package_share_director
from droneload_interfaces.msg import PictureData


class PictureDetect(Node):
    def __init__(self):
        super().__init__('picture_detect')
        self.get_logger().info("Picture Detector Initialized")

        # === CONFIGURATION ===
        package_name = 'droneload_auto'
        model_file = 'yolov8_custom.pt'
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
        
        x, y, clss, cf, sc = self.detect_picture(cv_image, image_height, image_width)

        # === CREATE AND PUBLISH PictureData ===
        picture_msg = PictureData()
        picture_msg.timestamp = int(self.get_clock().now().nanoseconds / 1e6)  # milliseconds
        picture_msg.x = x
        picture_msg.y = y
        picture_msg.values = clss
        picture_msg.confidences = cf
        picture_msg.scales = sc
        self.picture_data_pub.publish(picture_msg)

    def detect_picture(self, image_data, width=400, height=400):
        pictures_x = []
        pictures_y = []
        classes = []
        confidences = []
        scales = []

        # Load YOLOv8 model
        resized_img = cv2.resize(image_data, (width, height))

        # Run inference
        results = self.model(resized_img)

        # Compute the center point of the found object
        for box in results[0].boxes:
            c = box.xyxy[0]
            center_x = (c[0] + c[2]) / 2
            center_y = (c[1] + c[3]) / 2

            # Normalize between 0 and 1
            norm_x = center_x / width
            norm_y = center_y / height

            # Compute the area of the bounding box using the shoelace formula
            area = (c[2] - c[0]) * (c[3] - c[1])

            # Normalize the area between 0 and 1
            norm_area = area / (width * height)

            pictures_x.append(float(norm_x))
            pictures_y.append(float(norm_y))
            classes.append(self.model.names[int(box.cls[0])])
            confidences.append(float(box.conf[0]))
            scales.append(float(norm_area))
        return pictures_x, pictures_y, classes, confidences, scales

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
