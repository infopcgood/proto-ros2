import numpy as np
import cv2
import adafruit_blinka_raspberry_pi5_piomatter as piomatter

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from interface_pkg.msg import Image
from interface_pkg.srv import ControlRequest

SCREEN_WIDTH = 64
SCREEN_HEIGHT = 32
SCREEN_CHAINED = 1
ROTATE_SECOND_SCREEN = 1

SCREEN_BRIGHTNESS = 80

DOMINATING_NODE_NAME_DEFAULT = 'none'

## ScreenDispatcherNode class
# The screen dispatcher controls what node is in control of the screen.
# It then receives the frame data in interface_pkg.msg.Image format and 
# displays it onto the screen
class ScreenDispatcherNode(Node):
    ## __init__ function
    # This function does not take any parameters.
    # This function also initializes the LED Matrix display and creates a subscription.
    def __init__(self):
        super().__init__('screen_dispatcher_node')
        # Set dominating node name parameter to default value
        self.declare_parameter('dominating_node_name', DOMINATING_NODE_NAME_DEFAULT)
        # LED Matrix initialization
        self.geometry = piomatter.Geometry(width=SCREEN_WIDTH, height=SCREEN_HEIGHT * (SCREEN_CHAINED + 1), n_addr_lines=4, rotation=piomatter.Orientation.Normal)
        self.framebuffer = np.zeros((SCREEN_HEIGHT * (SCREEN_CHAINED + 1), SCREEN_WIDTH, 3)).astype(np.uint8)
        self.matrix = piomatter.PioMatter(colorspace=piomatter.Colorspace.RGB888Packed,
                             pinout=piomatter.Pinout.AdafruitMatrixBonnet,
                             framebuffer=self.framebuffer,
                             geometry=self.geometry)
        # Subscribe to /screen_display topic
        self.display_sub = self.create_subscription(Image, 'screen_display', self.display_sub_cb, 10)
        # Declare /request_screen_control service
        self.request_srv = self.create_service(ControlRequest, 'request_screen_control', self.request_srv_cb)

    ## display_sub_cb function
    # When /screen_display topic is received, check if sender is controlling the screen
    # and display the image.
    def display_sub_cb(self, image):
        if SCREEN_WIDTH != image.width:
            return
        if SCREEN_HEIGHT != image.height:
            return
        if image.sender != self.get_parameter('dominating_node_name').get_parameter_value().string_value:
            return
        frame = (np.array(image.data).reshape((SCREEN_HEIGHT, SCREEN_WIDTH, 3)) * (SCREEN_BRIGHTNESS / 100)).astype(np.uint8)[:,:,[2, 0, 1]]
        self.framebuffer[:] = np.vstack((frame, frame[::(-1 if ROTATE_SECOND_SCREEN else 0), ::(-1 if ROTATE_SECOND_SCREEN and not image.flip_second_screen else 1), :]))
        self.matrix.show()

    ## request_sub_cb function
    # When /request_screen_control topic is received, give screen control.
    def request_srv_cb(self, request, response):
        self.get_logger().info(f'Node {request.sender} has asked for screen control!')
        self.set_parameters([Parameter('dominating_node_name', Parameter.Type.STRING, request.sender)])
        response.success = True
        response.message = ""
        self.get_logger().info(f'Successfully gave screen control to {request.sender}!')
        return response

def main(args=None):
    rclpy.init(args=args)
    node = ScreenDispatcherNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('KeyboardInterrupt, shutting down dispatcher...')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
