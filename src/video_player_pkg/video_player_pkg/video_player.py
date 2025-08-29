import numpy as np
import cv2
from PIL import Image, ImageDraw
import time
from pydub import AudioSegment
from pydub.playback import play
import threading

import rclpy
from rclpy.node import Node
from interface_pkg.msg import Image
from interface_pkg.srv import ControlRequest

SCREEN_WIDTH = 64
SCREEN_HEIGHT = 32

## VideoPlayerNode class.
# Inherits the Node class from rclpy, and plays a video from the given filename.
# Plays the audio at the same time with the video.
# The video will stop at either the end of the video or an error, but the audio
# will only stop at the end of the video (it will keep playing when an error happens)
class VideoPlayerNode(Node):
    ## __init__ function.
    # Gets filename as string parameter, and automatically detects file format.
    # It also detects the FPS of the given video, and will set the timer according to it.
    # Audio is played at the same time by using the 'threading' library.
    def __init__(self, filename):
        super().__init__('video_player_node')
        # Get video capture and data
        self.cap = cv2.VideoCapture(filename)
        fps = self.cap.get(cv2.CAP_PROP_FPS)
        # Make publisher and load audio
        self.pub = self.create_publisher(Image, 'request_screen_display', 10)
        self.audio = AudioSegment.from_file(filename, filename.strip().split('.')[-1])
        # Ask for screen control and play video/audio
        request_cli = self.create_client(ControlRequest, 'request_screen_control')
        request = ControlRequest.Request()
        request.sender = 'video_player_node'
        self.get_logger().info('Asking screen control permission...')
        request_cli.wait_for_service()
        future = request_cli.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        if future.result().success:
            self.get_logger().info('Got screen control permission!')
            self.create_timer(1 / fps, self.frame_timer_cb)
            threading.Thread(target=play, args=(self.audio,)).start()
        else:
            self.get_logger().info('Failed to get screen control! Exiting...')
            self.destroy_node()

    ## frame_timer_cb function
    # Executed each frame (1/FPS).
    # Tries to read a frame from the loaded video and if successful publishes the frame
    # as an object of interface_pkg.msg.Image
    def frame_timer_cb(self):
        if not self.cap.isOpened(): # If video has ended
            self.destroy_node()
            return
        ret, frame = self.cap.read() # Try to read frame from video
        if not ret: # If frame could not be read
            self.destroy_node()
            return
        # Make an Image msg object to send
        image = Image()
        image.sender = 'video_player_node'
        image.height = SCREEN_HEIGHT
        image.width = SCREEN_WIDTH
        image.flip_second_screen = False
        image.data = (cv2.resize(frame, (SCREEN_WIDTH, SCREEN_HEIGHT))).astype(np.uint8).reshape((-1,)) # Data needs to be reshaped into a 1D array
        self.pub.publish(image) # Send object

# Simple main function
def main(args=None):
    rclpy.init(args=args)
    node = VideoPlayerNode(input('Input video filename: '))
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('KeyboardInterrupt has occured, shutting down.')
    finally:
        if node:
            node.destroy_node()
        rclpy.shutdown()

