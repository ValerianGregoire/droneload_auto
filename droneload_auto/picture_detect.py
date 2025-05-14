import cv2
import numpy as np
import matplotlib.pyplot as plt
from skimage.morphology import skeletonize
from skimage.util import invert
from collections import deque

import rclpy
from rclpy.node import Node
from droneload_interfaces.msg import PictureData
from sensor_msgs.msg import Image

class PictureDetect(Node):
    def __init__(self):
        super().__init__('picture_detect')
        
        ################ SUBSCRIBER to the camera data topic
        self.subscription = self.create_subscription(
            CameraData,
            'camera_data',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

        ################ PUBLISHER to the line picture data 
        # self.publisher = self.create_publisher(Type de data à mettre, 'line_data', 10)

# Function that get the data from the camera
    def listener_callback(self, msg):
        # To be written 
        self.get_logger().info('I heard: "%s"' % msg.data)

# Function that publish the data (path) to the topic  
    def publish_callback(self):
        # self.path
        # To be written 
        # self.publisher.publish(msg)
        pass

def main(args=None):
    rclpy.init(args=args)

    picture_detect = PictureDetect()
    rclpy.spin(picture_detect)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    picture_detect.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()