import numpy as np
import math
import time
import librosa
import pyaudio

import rclpy
from rclpy.node import Node

from interface_pkg.msg import InputAudioFragment

class MicInputNode(Node):
    def __init__(self):
        super().__init__('mic_input_node')
        self.mic_pub = self.create_publisher(InputAudioFragment, 'mic_input', 10)
        self.pyaudio = pyaudio.PyAudio()
        self.start_time = time.time_ns()
        self.before_time = time.time_ns()
        self.stream = self.pyaudio.open(format=pyaudio.paFloat32,
                      channels=1,
                      rate=96000,
                      input=True,
                      output=False,
                      stream_callback=self.mic_cb,
                      frames_per_buffer=4096)

    def mic_cb(self, in_data, frame_count, time_info, flag):
        if (time.time_ns() - self.before_time) < 5000000:
            return None, pyaudio.paContinue
        
        numpy_array = np.frombuffer(in_data, dtype=np.float32)
        mfcc_data = librosa.feature.mfcc(y=numpy_array, sr=96000).reshape((-1, )).astype(np.float32)

        fft = np.fft.fft(numpy_array)[19:48]
        freqs = np.fft.fftfreq(numpy_array.size, d=1/96000)[19:48]
        frequencies = freqs[np.argsort(np.abs(fft))[::-1]]

        # print((frequencies[0]) if np.max(np.abs(numpy_array)) > 0.1 else 0, f'Time: {(time.time_ns() - self.before_time) / 1000000 :.3f} ms, {(time.time_ns() - self.start_time) / 1000000 :.3f} ms')
        self.before_time = time.time_ns()

        fragment = InputAudioFragment()
        
        fragment.data = numpy_array
        fragment.amplitude = float(np.max(np.abs(numpy_array)))
        fragment.mfcc = mfcc_data

        fragment.frequency_amp = np.sort(np.abs(fft))[::-1].astype(np.float32)
        fragment.frequency_hz = frequencies.astype(np.float32)
        fragment.phoneme = ''

        self.mic_pub.publish(fragment)

        return None, pyaudio.paContinue

def main(args=None):
    rclpy.init(args=args)
    node = MicInputNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('KeyboardInterrupt detected, halting...')
    finally:
        node.stream.close()
        node.destroy_node()
        rclpy.shutdown()