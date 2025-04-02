import numpy as np
import rclpy
from rclpy.node import Node
import PIL.Image

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
            self.create_timer(1 / FACE_FPS, self.render_face_timer_cb)
        else:
            self.get_logger().info('Failed to get screen control! Exiting...')
            self.destroy_node()
    
    def expression_sub_cb(self, expression):
        if expression.type in AVAILABLE_FACE_PARTS:
            face_parts_state = self.get_parameter('face_parts_state').get_parameter_value().string_array_value
            face_parts_state[AVAILABLE_FACE_PARTS.index(expression.type)] = expression.expression
        else:
            self.get_logger().info(f'Invalid part name "{expression.type}".')

    def render_face_timer_cb(self):
        face_parts_state = self.get_parameter('face_parts_state').get_parameter_value().string_array_value
        image = PIL.Image.new('RGB', (SCREEN_WIDTH, SCREEN_HEIGHT), (0, 0, 0))
        for face, loc, state in zip(AVAILABLE_FACE_PARTS, FACE_PART_CORNER_LOCATION, face_parts_state):
            image.paste(PIL.Image.open(self.image_path + face + '/' + state + '.png'), loc)
        image_msg = Image()
        image_msg.sender = 'face_renderer_node'
        image_msg.height = SCREEN_HEIGHT
        image_msg.width = SCREEN_WIDTH
        image_msg.data = np.array(image).reshape((-1, ))
        image_msg.flip_second_screen = True
        self.image_pub.publish(image_msg)

def main(args=None):
    rclpy.init(args=args)
    node = FaceRendererNode('/home/info/proto-ros2/images/face/')
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('KeyboardInterrupt received, terminating node...')
    finally:
        node.destroy_node()
        rclpy.shutdown()
