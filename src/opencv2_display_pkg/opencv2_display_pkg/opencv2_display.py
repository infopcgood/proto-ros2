import cv2
import numpy as np

import rclpy
from rclpy.node import Node

from interface_pkg.msg import Image
from interface_pkg.srv import ControlRequest


SCREEN_WIDTH = 64
SCREEN_HEIGHT = 32
SCALE = 10

class OpenCV2Display(Node):
    def __init__(self):
        super().__init__('opencv2_display_node')
        # OpenCV2 window initialization
        # Subscribe to /screen_display topic
        self.display_sub = self.create_subscription(Image, 'screen_display', self.display_sub_cb, 10)
    
    ## display_sub_cb function
    # When /screen_display topic is received, check size and display the image.
    def display_sub_cb(self, image):
        if SCREEN_WIDTH != image.width:
            return
        if SCREEN_HEIGHT != image.height:
            return
        frame = (np.array(image.data).reshape((SCREEN_HEIGHT, SCREEN_WIDTH, 3))).astype(np.uint8)
        big_frame = cv2.resize(frame, (SCREEN_WIDTH * SCALE, SCREEN_HEIGHT * SCALE), interpolation = cv2.INTER_NEAREST)
        cv2.imshow('Display', big_frame)
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    node = OpenCV2Display()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('KeyboardInterrupt detected, halting...')
    finally:
        node.stream.close()
        node.destroy_node()
        rclpy.shutdown()