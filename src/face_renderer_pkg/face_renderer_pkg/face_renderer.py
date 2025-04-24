import numpy as np
import time
import rclpy
from rclpy.node import Node
import cv2

from interface_pkg.msg import FacialExpression, Image
from interface_pkg.srv import ControlRequest

SCREEN_WIDTH = 64
SCREEN_HEIGHT = 32
FACE_FPS = 24

AVAILABLE_FACE_PARTS = ['mouth', 'eyes']
FACE_PART_CORNER_LOCATION = [(16, 16), (0, 0)]

class FaceRendererNode(Node):
    def __init__(self, image_path):
        super().__init__('face_renderer_node')
        self.image_path = image_path
        self.declare_parameter('face_parts_state', ['idle'] * len(AVAILABLE_FACE_PARTS))
        # Ask for screen control
        request_cli = self.create_client(ControlRequest, 'request_screen_control')
        request = ControlRequest.Request()
        request.sender = 'face_renderer_node'
        self.get_logger().info('Asking screen control permission...')
        request_cli.wait_for_service()
        future = request_cli.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        if future.result().success:
            self.get_logger().info('Got screen control permission!')
            self.image_pub = self.create_publisher(Image, 'screen_display', 10)
            self.expression_sub = self.create_subscription(FacialExpression, 'facial_expression', self.expression_sub_cb, 10)
            self.override_time = 0
            self.override_part = ''
            self.override_start = 0
            self.create_timer(1 / FACE_FPS, self.render_face_timer_cb)
        else:
            self.get_logger().info('Failed to get screen control! Exiting...')
            self.destroy_node()
    
    def expression_sub_cb(self, expression):
        if expression.type in AVAILABLE_FACE_PARTS:
            if ((time.time_ns() - self.override_start) <= (self.override_time * 1000000000)) and expression.type == self.override_part:
                self.get_logger().info(f'Override part "{expression.type}"!')
                return
            face_parts_state = self.get_parameter('face_parts_state').get_parameter_value().string_array_value
            face_parts_state[AVAILABLE_FACE_PARTS.index(expression.type)] = expression.expression
            if expression.override_time > 0:
                self.override_time = expression.override_time
                self.override_start = time.time_ns()
                self.override_part = expression.type
        else:
            self.get_logger().info(f'Invalid part name "{expression.type}".')

    def render_face_timer_cb(self):
        face_parts_state = self.get_parameter('face_parts_state').get_parameter_value().string_array_value
        image = np.zeros((SCREEN_HEIGHT, SCREEN_WIDTH, 3))
        for face, loc, state in zip(AVAILABLE_FACE_PARTS, FACE_PART_CORNER_LOCATION, face_parts_state):
            temp_part = cv2.imread(self.image_path + face + '/' + state + '.png')
            thresh = cv2.threshold(cv2.cvtColor(temp_part, cv2.COLOR_BGR2GRAY), 1, 255, cv2.THRESH_BINARY)
            image = cv2.bitwise_and(image, image, mask=~thresh) + cv2.bitwise_and(temp_part, temp_part, mask=thresh)
        image_msg = Image()
        image_msg.sender = 'face_renderer_node'
        image_msg.height = SCREEN_HEIGHT
        image_msg.width = SCREEN_WIDTH
        image_msg.data = image.reshape((-1, ))
        image_msg.flip_second_screen = True
        self.image_pub.publish(image_msg)

def main(args=None):
    rclpy.init(args=args)
    node = FaceRendererNode('/home/protopi/proto-ros2/images/face/')
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('KeyboardInterrupt received, terminating node...')
    finally:
        node.destroy_node()
        rclpy.shutdown()
