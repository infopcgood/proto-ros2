"""Dense WS panel output with an explicit map; legacy display entry stays intact."""
import math
import time

import rclpy
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.clock import Clock
from rclpy.clock_type import ClockType
from rclpy.node import Node
from interface_pkg.msg import Image
from .atlas_mapping import AtlasMap


class AtlasDisplay(Node):
    def __init__(self, **kwargs):
        super().__init__('prootface_atlas_display_node', **kwargs)
        self.started = False
        self.driver = None
        self.last_frame = None
        self.cleared = False
        self._closed = False
        self.rejected = 0
        try:
            defaults = dict(
                panel_map='', brightness=3, driver_strips=8, stale_timeout=1.0,
                expected_side_width=0, expected_side_height=0,
            )
            for key, value in defaults.items():
                self.declare_parameter(key, value, ParameterDescriptor(read_only=True))
            path = self.get_parameter('panel_map').value
            if not path:
                raise ValueError('panel_map must name the actual dense-WS LED coordinate file')
            self.mapping = AtlasMap.load(path)
            for key, actual in [('expected_side_width', self.mapping.width),
                                ('expected_side_height', self.mapping.height)]:
                expected = self.get_parameter(key).value
                if expected and expected != actual:
                    raise ValueError('Panel map does not match renderer ' + key)
            if self.mapping.num_strips != self.get_parameter('driver_strips').value:
                raise ValueError('Panel-map strip count must match the installed smileds driver build')
            self.stale_timeout = float(self.get_parameter('stale_timeout').value)
            if not math.isfinite(self.stale_timeout) or self.stale_timeout <= 0:
                raise ValueError('stale_timeout must be finite and positive')
            import smileds
            self.driver = smileds
            self.driver.leds_init(self.mapping.leds_per_strip,
                                  self.get_parameter('brightness').value)
            self.started = True
            self.subscription = self.create_subscription(Image, 'screen_display', self.on_frame, 1)
            self.timer = self.create_timer(
                .1, self.watchdog, clock=Clock(clock_type=ClockType.STEADY_TIME)
            )
        except BaseException:
            self.destroy_node()
            raise

    def on_frame(self, message):
        try:
            payload = self.mapping.rgb_bytes(
                message.data, message.width, message.height, message.flip_second_screen
            )
        except (ValueError, TypeError):
            self.rejected += 1
            return
        self.driver.leds_set(payload)
        self.driver.leds_send()
        self.last_frame = time.monotonic()
        self.cleared = False

    def watchdog(self):
        if (self.last_frame is not None and not self.cleared
                and time.monotonic() - self.last_frame > self.stale_timeout):
            self.driver.leds_clear()
            self.cleared = True

    def destroy_node(self):
        if self._closed:
            return
        self._closed = True
        try:
            if self.started:
                self.started = False
                self.driver.leds_clear()
        finally:
            super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = AtlasDisplay()
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
