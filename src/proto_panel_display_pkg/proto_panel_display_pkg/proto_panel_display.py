import cv2
import numpy as np
import smileds
import time

import rclpy
from rclpy.node import Node

from rcl_interfaces.srv import GetParameters

from interface_pkg.msg import Image
from interface_pkg.srv import ControlRequest

NUM_LEDS = 150
NUM_STRIPS = 8
BRIGHTNESS = 3

class ProtoPanelDisplay(Node):
    def __init__(self):
        super().__init__('proto_panel_display_node')

        # Load settings
        self.param_cli = self.create_client(GetParameters, '/param_server_node/get_parameters')
        while not self.param_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for parameter service...')
        get_params = GetParameters.Request()
        get_params.names = ['DISPLAY_WIDTH', 'DISPLAY_HEIGHT']
        param_future = self.param_cli.call_async(get_params)
        rclpy.spin_until_future_complete(self, param_future)
        self.DISPLAY_WIDTH, self.DISPLAY_HEIGHT = param_future.result().values[0].integer_value, param_future.result().values[1].integer_value

        # Check if image is correct size
        if self.DISPLAY_WIDTH != 50 or self.DISPLAY_HEIGHT != 34:
            raise ValueError()

        # smileds initialization
        smileds.leds_init(NUM_LEDS, BRIGHTNESS)
        idx = []

        with open('id.txt', 'r') as f:
            idx = [tuple(map(int, x.strip().split())) for x in f.readlines()]

        if len(idx) != NUM_LEDS * NUM_STRIPS:
            print('ERROR: Missing pixel location data!')
            exit()
        
        self.COORDS = np.array(idx)
        
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
        pixels = frame[self.COORDS[:, 1], self.COORDS[:, 0]]  # shape (N, 3)
    
        # BGR → RGB
        pixels = pixels[:, ::-1]
    
        leds = pixels.astype(np.uint8).tobytes()
    
        smileds.leds_set(leds)
        smileds.leds_send()

def main(args=None):
    rclpy.init(args=args)
    node = ProtoPanelDisplay()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('KeyboardInterrupt detected, halting...')
    finally:
        node.stream.close()
        node.destroy_node()
        smileds.leds_clear()
        rclpy.shutdown()
