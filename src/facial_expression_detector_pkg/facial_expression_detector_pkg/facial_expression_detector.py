from imutils import face_utils
import numpy as np
import argparse
import imutils
import dlib
import cv2
import math

import rclpy
from rclpy.node import Node

from interface_pkg.msg import FacialExpression

def length(a, b):
    return math.sqrt((a[0] - b[0]) ** 2 + (a[1] - b[1]) ** 2)

class FacialExpressionDetectorNode(Node):
    def __init__(self):
        super().__init__('facial_expression_detector_node')
        self.detector = dlib.get_frontal_face_detector()
        self.predictor = dlib.shape_predictor('/home/protopi/proto-ros2/src/facial_expression_detector_pkg/shape_predictor_68_face_landmarks.dat')
        self.cap = cv2.VideoCapture(0)
        self.continuous_frames = 0
        self.expression_pub = self.create_publisher(FacialExpression, 'facial_expression', 10)
        self.create_timer(1 / 60, self.capture_expression_cb)

    def capture_expression_cb(self):
        self.continuous_frames += 1
        
        ret, frame = self.cap.read()
        if not ret:
            return
        frame_gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        rects = self.detector(frame_gray, 1)
        if len(rects) == 0:
            return
        rect = rects[0]
        frame_gray = (frame_gray - np.min(frame_gray[rect.top():rect.bottom(),rect.left():rect.right()])) * (255 / (np.max(frame_gray[rect.top():rect.bottom(),rect.left():rect.right()]) - np.min(frame_gray[rect.top():rect.bottom(),rect.left():rect.right()]))).astype(np.uint8)
        shape = self.predictor(frame_gray, rect)
        shape = face_utils.shape_to_np(shape)
        left_eye_opened = (length(shape[37], shape[41]) + length(shape[38], shape[40])) / (2 * length(shape[36], shape[39]))
        right_eye_opened = (length(shape[43], shape[47]) + length(shape[44], shape[46])) / (2 * length(shape[42], shape[45]))
        eye_opened = (left_eye_opened + right_eye_opened) / 2
        if self.continuous_frames >= 30:
            expression = FacialExpression()
            expression.type = 'eyes'
            if eye_opened >= 0.28:
                expression.expression = 'idle'
            elif eye_opened >= 0.16:
                expression.expression = 'half-closed'
            else:
                expression.expression = 'closed'
            self.expression_pub.publish(expression)
            self.continuous_frames = 0
        mouth_opened = length(shape[62], shape[66]) / length(shape[60], shape[64])
        expression = FacialExpression()
        expression.type = 'mouth'
        if mouth_opened >= 0.4:
            expression.expression = 'open'
        elif mouth_opened >= 0.135:
            expression.expression = 'half-open'
        else:
            expression.expression = 'idle'
        self.expression_pub.publish(expression)

def main(args=None):
    rclpy.init(args=args)
    node = FacialExpressionDetectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('KeyboardInterrupt detected, halting...')
    finally:
        node.cap.release()
        node.destroy_node()
        rclpy.shutdown()