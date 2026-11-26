"""Optional atlas preview owned entirely by Prootface."""
import numpy as np
import rclpy
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.node import Node
from interface_pkg.msg import Image


class PreviewDisplay(Node):
    def __init__(self, **kwargs):
        super().__init__('prootface_preview_node', **kwargs)
        self._closed = False
        self.cv = None
        self.window = 'Prootface atlas'
        self.rejected = 0
        try:
            for name, value in dict(scale=2, expected_width=0, expected_height=0).items():
                self.declare_parameter(name, value, ParameterDescriptor(read_only=True))
            self.scale = self.get_parameter('scale').value
            self.width = self.get_parameter('expected_width').value
            self.height = self.get_parameter('expected_height').value
            if self.scale <= 0 or self.width < 0 or self.height < 0:
                raise ValueError('Scale must be positive and expected dimensions nonnegative')
            import cv2
            self.cv = cv2
            self.subscription = self.create_subscription(Image, 'screen_display', self.on_frame, 1)
        except BaseException:
            self.destroy_node()
            raise

    def on_frame(self, message):
        width, height = message.width, message.height
        if (width <= 0 or height <= 0 or message.flip_second_screen
                or (self.width and width != self.width)
                or (self.height and height != self.height)
                or len(message.data) != width * height * 3):
            self.rejected += 1
            return
        frame = np.frombuffer(message.data, dtype=np.uint8).reshape(height, width, 3)
        scaled = self.cv.resize(frame, (width * self.scale, height * self.scale),
                                interpolation=self.cv.INTER_NEAREST)
        self.cv.imshow(self.window, scaled)
        self.cv.waitKey(1)

    def destroy_node(self):
        if self._closed:
            return
        self._closed = True
        try:
            if self.cv is not None:
                self.cv.destroyAllWindows()
        finally:
            super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = PreviewDisplay()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            if node is not None:
                node.destroy_node()
        finally:
            if rclpy.ok():
                rclpy.shutdown()
