import cv2
import numpy as np

import rclpy
from rclpy.node import Node

from rcl_interfaces.srv import GetParameters

from interface_pkg.msg import Image
from interface_pkg.srv import ControlRequest

SCALE = 10

class OpenCV2Display(Node):
    def __init__(self):
        super().__init__('opencv2_display_node')

        # Load settings
        self.param_cli = self.create_client(GetParameters, '/param_server_node/get_parameters')
        while not self.param_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for parameter service...')
        get_params = GetParameters.Request()
        get_params.names = ['DISPLAY_WIDTH', 'DISPLAY_HEIGHT']
        param_future = self.param_cli.call_async(get_params)
        rclpy.spin_until_future_complete(self, param_future)
        self.DISPLAY_WIDTH, self.DISPLAY_HEIGHT = param_future.result().values[0].integer_value, param_future.result().values[1].integer_value

        # OpenCV2 window initialization
        # Subscribe to /screen_display topic
        self.display_sub = self.create_subscription(Image, 'screen_display', self.display_sub_cb, 10)
    
    ## display_sub_cb function
    # When /screen_display topic is received, check size and display the image.
    def display_sub_cb(self, image):
        if self.DISPLAY_WIDTH != image.width:
            return
        if self.DISPLAY_HEIGHT != image.height:
            return
        frame = (np.array(image.data).reshape((self.DISPLAY_HEIGHT, self.DISPLAY_WIDTH, 3))).astype(np.uint8)
        big_frame = cv2.resize(frame, (self.DISPLAY_WIDTH * SCALE, self.DISPLAY_HEIGHT * SCALE), interpolation = cv2.INTER_NEAREST)
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