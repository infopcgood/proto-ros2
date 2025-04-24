import rclpy
from rclpy.node import Node

from interface_pkg.msg import InputAudioFragment, FacialExpression

class AudioExpressionDetectorNode(Node):
    def __init__(self):
        super().__init__('audio_expression_detector_node')
        self.create_subscription(InputAudioFragment, 'mic_input', self.mic_input_cb, 10)
        self.expression_pub = self.create_publisher(FacialExpression, 'facial_expression', 10)
    
    def mic_input_cb(self, fragment):
        if fragment.amplitude >= 0.125:
            flag = False
            i = 0
            while i < 10 and fragment.frequency_amp[i] >= fragment.frequency_amp[0] * 0.8:
                if fragment.frequency_hz[i] >= 900:
                    flag = True
                    break
                i += 1
            expression = FacialExpression()
            expression.type='eyes'
            expression.override_time = 0.33
            expression.expression = 'excited' if flag else 'idle'
            self.expression_pub.publish(expression)

        expression = FacialExpression()
        expression.type = 'mouth'
        if fragment.amplitude >= 0.1:
            expression.expression = 'open'
            expression.override_time = float(0.125)
        elif fragment.amplitude >= 0.05:
            expression.expression = 'half-open'
            expression.override_time = float(0.1)
        else:
            expression.expression = 'idle'
            expression.override_time = float(0)
        self.expression_pub.publish(expression)

def main(args=None):
    rclpy.init(args=args)
    node = AudioExpressionDetectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('KeyboardInterrupt received, terminating node...')
    finally:
        node.destroy_node()
        rclpy.shutdown()
