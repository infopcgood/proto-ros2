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
            if flag:
                expression = FacialExpression()
                expression.type='eyes'
                expression.override_time = 0.33
                expression.expression = 'smile'
                self.expression_pub.publish(expression)
                print('hi!')
            else:
                print('no.')

        expression = FacialExpression()
        expression.type = 'mouth'
        expression.override_time = float(0.125)
        if fragment.amplitude >= 0.8:
            expression.expression = 'open'
        elif fragment.amplitude >= 0.5:
            expression.expression = 'half-open'
        else:
            expression.expression = 'idle'
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
