import cv2
import numpy as np
import adafruit_blinka_raspberry_pi5_piomatter as piomatter

import rclpy
from rclpy.node import Node

from interface_pkg.msg import Image
from interface_pkg.srv import ControlRequest


SCREEN_WIDTH = 64
SCREEN_HEIGHT = 32
SCREEN_CHAINED = 1
ROTATE_SECOND_SCREEN = 1

SCREEN_BRIGHTNESS = 80

class HUB75Display(Node):
    def __init__(self):
        super().__init__('hub75_display_node')
        # LED Matrix initialization
        self.geometry = piomatter.Geometry(width=SCREEN_WIDTH, height=SCREEN_HEIGHT * (SCREEN_CHAINED + 1), n_addr_lines=4, rotation=piomatter.Orientation.Normal)
        self.framebuffer = np.zeros((SCREEN_HEIGHT * (SCREEN_CHAINED + 1), SCREEN_WIDTH, 3)).astype(np.uint8)
        self.matrix = piomatter.PioMatter(colorspace=piomatter.Colorspace.RGB888Packed,
                             pinout=piomatter.Pinout.AdafruitMatrixBonnet,
                             framebuffer=self.framebuffer,
                             geometry=self.geometry)
        # Subscribe to /screen_display topic
        self.display_sub = self.create_subscription(Image, 'screen_display', self.display_sub_cb, 10)
    
    ## display_sub_cb function
    # When /screen_display topic is received, check size and display the image.
    def display_sub_cb(self, image):
        if SCREEN_WIDTH != image.width:
            return
        if SCREEN_HEIGHT != image.height:
            return
        frame = (np.array(image.data).reshape((SCREEN_HEIGHT, SCREEN_WIDTH, 3)) * (SCREEN_BRIGHTNESS / 100)).astype(np.uint8)[:,:,[2, 0, 1]]
        self.framebuffer[:] = np.vstack((frame, frame[::(-1 if ROTATE_SECOND_SCREEN else 0), ::(-1 if ROTATE_SECOND_SCREEN and not image.flip_second_screen else 1), :]))
        self.matrix.show()