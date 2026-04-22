import rclpy
from rclpy.node import Node

class ParamServer(Node):
    def __init__(self):
        super().__init__('param_server_node')
        self.declare_parameter('DISPLAY_WIDTH', 50)
        self.declare_parameter('DISPLAY_HEIGHT', 34)
    
def main(args=None):
    rclpy.init(args=args)
    node = ParamServer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('KeyboardInterrupt detected, halting...')
    finally:
        node.stream.close()
        node.destroy_node()
        rclpy.shutdown()