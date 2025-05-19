import rclpy
from rclpy.node import Node

import numpy as np
import pandas as pd
import tensorflow as tf
import librosa
from tensorflow import keras

from interface_pkg.msg import InputAudioFragment, FacialExpression

VOWELS_STR = "AUH, OOUU, IEEU, AEE, UNGM, NL, idle"
VOWELS_LIST = VOWELS_STR.split(', ')

class AudioExpressionDetectorNode(Node):
    def __init__(self):
        super().__init__('audio_expression_detector_node')
        self.create_subscription(InputAudioFragment, 'mic_input', self.mic_input_cb, 10)
        self.model = tf.keras.models.load_model('/home/protopi/proto-ros2/src/audio_expression_detector_pkg/SoundTrainer_E50.keras')
        self.expression_pub = self.create_publisher(FacialExpression, 'facial_expression', 10)
        self.buffer = []
    
    def mic_input_cb(self, fragment):
        expression = FacialExpression()
        expression.type = 'mouth'
        if fragment.amplitude < 0.1:
            expression.expression = 'idle'
        else:
            mfcc_data = np.array(fragment.mfcc)[1:].reshape((1, 19))
            preds = np.argmax(self.model.predict(mfcc_data, verbose=0))
            if len(self.buffer) < 5:
                self.buffer += [preds]
            else:
                self.buffer = self.buffer[1:] + [preds]
            pred = stats.mode(self.buffer)
            expression.expression = VOWELS_LIST[pred.mode]
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
