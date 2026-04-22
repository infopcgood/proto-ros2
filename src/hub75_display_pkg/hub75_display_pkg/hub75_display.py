import cv2
import numpy as np
import adafruit_blinka_raspberry_pi5_piomatter as piomatter

import rclpy
from rclpy.node import Node

from rcl_interfaces.srv import GetParameters

from interface_pkg.msg import Image
from interface_pkg.srv import ControlRequest

SCREEN_CHAINED = 1
ROTATE_SECOND_SCREEN = 1

SCREEN_BRIGHTNESS = 80

class HUB75Display(Node):
    def __init__(self):
        super().__init__('hub75_display_node')

        # Load settings
        self.param_cli = self.create_client(GetParameters, '/param_server_node/get_parameters')
        while not self.param_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for parameter service...')
        get_params = GetParameters.Request()
        get_params.names = ['DISPLAY_WIDTH', 'DISPLAY_HEIGHT']
        param_future = self.param_cli.call_async(get_params)
        rclpy.spin_until_future_complete(self, param_future)
        self.DISPLAY_WIDTH, self.DISPLAY_HEIGHT = param_future.result().values[0].integer_value, param_future.result().values[1].integer_value
        
        # LED Matrix initialization
        self.geometry = piomatter.Geometry(width=self.DISPLAY_WIDTH, height=self.DISPLAY_HEIGHT * (SCREEN_CHAINED + 1), n_addr_lines=4, rotation=piomatter.Orientation.Normal)
        self.framebuffer = np.zeros((self.DISPLAY_HEIGHT * (SCREEN_CHAINED + 1), self.DISPLAY_WIDTH, 3)).astype(np.uint8)
        self.matrix = piomatter.PioMatter(colorspace=piomatter.Colorspace.RGB888Packed,
                             pinout=piomatter.Pinout.AdafruitMatrixBonnet,
                             framebuffer=self.framebuffer,
                             geometry=self.geometry)
        # Subscribe to /screen_display topic
        self.display_sub = self.create_subscription(Image, 'screen_display', self.display_sub_cb, 10)
    
    ## display_sub_cb function
    # When /screen_display topic is received, check size and display the image.
    def display_sub_cb(self, image):
        if self.DISPLAY_WIDTH != image.width:
            return
        if self.DISPLAY_HEIGHT != image.height:
            return
        frame = (np.array(image.data).reshape((self.DISPLAY_HEIGHT, self.DISPLAY_WIDTH, 3)) * (SCREEN_BRIGHTNESS / 100)).astype(np.uint8)[:,:,[2, 0, 1]]
        self.framebuffer[:] = np.vstack((frame, frame[::(-1 if ROTATE_SECOND_SCREEN else 0), ::(-1 if ROTATE_SECOND_SCREEN and not image.flip_second_screen else 1), :]))
        self.matrix.show()

def main(args=None):
    rclpy.init(args=args)
    node = HUB75Display()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('KeyboardInterrupt detected, halting...')
    finally:
        node.stream.close()
        node.destroy_node()
        rclpy.shutdown()