"""Publish a computer's desktop through the existing screen dispatcher."""

from array import array
import math

import mss
from mss.exception import ScreenShotError
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rcl_interfaces.msg import ParameterDescriptor, ParameterType
from rcl_interfaces.srv import GetParameters

from interface_pkg.msg import Image
from interface_pkg.srv import ControlRequest

from desktop_streaming_pkg.frame_utils import prepare_frame
from desktop_streaming_pkg.portal_capture import PortalCapture, select_backend


class DesktopStreamingNode(Node):
    """Capture one monitor and publish frames while the dispatcher owns routing."""

    def __init__(self):
        super().__init__('desktop_streaming_node')
        self.capture = None
        self.timer = None

    def start(self):
        """Read configuration, open capture, and request control before streaming."""
        defaults = {
            'backend': 'auto',
            'monitor': 1,
            'fps': 24.0,
            'preserve_aspect_ratio': True,
            'flip_second_screen': False,
            'service_timeout': 10.0,
            'portal_timeout': 120.0,
        }
        values = {
            name: self.declare_parameter(
                name, default, ParameterDescriptor(read_only=True),
            ).value
            for name, default in defaults.items()
        }
        fps = values['fps']
        self.service_timeout = values['service_timeout']
        for name, value in [('fps', fps),
                            ('service_timeout', self.service_timeout),
                            ('portal_timeout', values['portal_timeout'])]:
            if not math.isfinite(value) or value <= 0:
                raise ValueError(f'{name} must be finite and greater than zero.')
        monitor_index = values['monitor']
        if monitor_index < 0:
            raise ValueError('monitor must be 0 (all monitors) or a positive index.')
        self.preserve_aspect_ratio = values['preserve_aspect_ratio']
        self.flip_second_screen = values['flip_second_screen']
        backend = select_backend(values['backend'])

        request = GetParameters.Request()
        request.names = ['DISPLAY_WIDTH', 'DISPLAY_HEIGHT']
        response = self.call_service(
            GetParameters, '/param_server_node/get_parameters', request,
        )
        if len(response.values) != 2 or any(
            value.type != ParameterType.PARAMETER_INTEGER or
            not 0 < value.integer_value <= 0xffffffff
            for value in response.values
        ):
            raise ValueError('Parameter server must provide positive integer '
                             'DISPLAY_WIDTH and DISPLAY_HEIGHT values.')
        self.width, self.height = [v.integer_value for v in response.values]

        try:
            if backend == 'portal':
                self.get_logger().info(
                    'Choose a screen in the desktop sharing dialog. '
                    'The monitor parameter applies only to the MSS backend.'
                )
                self.capture = PortalCapture(
                    timeout=values['portal_timeout'],
                    frame_timeout=self.service_timeout, is_running=rclpy.ok,
                )
                self.capture.start()
                self.monitor = None
            else:
                self.capture = mss.MSS()
                monitors = self.capture.monitors
                if len(monitors) <= 1 or monitor_index >= len(monitors):
                    raise ValueError(
                        f'Monitor {monitor_index} is unavailable; found '
                        f'{max(0, len(monitors) - 1)} monitor(s). '
                        'Use 0 for the combined desktop or 1..N for a monitor.'
                    )
                self.monitor = monitors[monitor_index]
            # Verify capture and conversion before taking over the screen.
            self.capture_frame()
        except ScreenShotError as exc:
            raise RuntimeError(
                'Cannot capture the desktop. Run in an active graphical '
                'session with screen-recording access (X11 on Linux).'
            ) from exc

        self.publisher = self.create_publisher(Image, 'request_screen_display', 1)
        request = ControlRequest.Request()
        request.sender = self.get_name()
        response = self.call_service(
            ControlRequest, 'request_screen_control', request,
        )
        if not response.success:
            raise RuntimeError(f'Screen control was denied: {response.message}')
        self.timer = self.create_timer(1.0 / fps, self.publish_frame)
        self.get_logger().info(
            f'Streaming desktop using {backend} at up to {fps:g} FPS '
            f'to {self.width}x{self.height}.'
        )

    def call_service(self, service_type, service_name, request):
        """Wait for a service and its response with bounded startup time."""
        client = self.create_client(service_type, service_name)
        try:
            if not client.wait_for_service(timeout_sec=self.service_timeout):
                raise RuntimeError(f'Service unavailable: {service_name}')
            future = client.call_async(request)
            rclpy.spin_until_future_complete(
                self, future, timeout_sec=self.service_timeout,
            )
            if not future.done():
                future.cancel()
                raise RuntimeError(f'Service response timed out: {service_name}')
            response = future.result()
            if response is None:
                raise RuntimeError(f'No response from service: {service_name}')
            return response
        finally:
            self.destroy_client(client)

    def capture_frame(self):
        """Capture and resize on the same thread that opened MSS."""
        return prepare_frame(
            self.capture.grab(self.monitor), self.width, self.height,
            self.preserve_aspect_ratio,
        )

    def publish_frame(self):
        """Publish packed BGR bytes and dispatcher-compatible sender metadata."""
        frame = self.capture_frame()
        message = Image()
        message.sender = self.get_name()
        message.width = self.width
        message.height = self.height
        message.flip_second_screen = self.flip_second_screen
        message.data = array('B', frame)
        self.publisher.publish(message)

    def close(self):
        """Stop capture on normal shutdown and partial startup failures."""
        if self.timer is not None:
            self.timer.cancel()
            self.timer = None
        if self.capture is not None:
            self.capture.close()
            self.capture = None


def main(args=None):
    """Run capture and ROS callbacks in the same single-threaded executor."""
    rclpy.init(args=args)
    node = None
    try:
        node = DesktopStreamingNode()
        node.start()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except Exception as exc:
        if node is not None:
            node.get_logger().error(f'Desktop streaming failed: {exc}')
        raise
    finally:
        try:
            if node is not None:
                try:
                    node.close()
                finally:
                    node.destroy_node()
        finally:
            rclpy.try_shutdown()


if __name__ == '__main__':
    main()
