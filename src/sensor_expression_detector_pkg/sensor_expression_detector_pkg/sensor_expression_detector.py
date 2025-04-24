import rclpy
from rclpy.node import Node

from interface_pkg.msg import ProximityTrigger, FacialExpression

class SensorExpressionDetectorNode(Node):
    def __init__(self):
        super().__init__('sensor_expression_detector_node')
        self.create_subscription(ProximityTrigger, 'proximity_trigger', self.proximity_trigger_cb, 10)
        self.expression_pub = self.create_publisher(FacialExpression, 'facial_expression', 10)
    
    def proximity_trigger_cb(self, data):
        if data.state == 'rising':
            expression = FacialExpression()
            expression.type = 'eyes'
            expression.expression = 'excited'
            expression.override_time = 1.25
            self.expression_pub.publish(expression)

def main(args=None):
    rclpy.init(args=args)
    node = SensorExpressionDetectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('KeyboardInterrupt received, terminating node...')
    finally:
        node.destroy_node()
        rclpy.shutdown()
