"""Single shared UDP input, with bounded reads and clean shutdown."""
import socket
import struct
import threading

import rclpy
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.node import Node
from interface_pkg.msg import OSFData
from .protocol import decode_datagram


class OSFReceiver(Node):
    def __init__(self, **kwargs):
        super().__init__('osf_receiver_node', **kwargs)
        self.socket = None
        self.loop_thread = None
        self.stopping = threading.Event()
        self._closed = False
        self.rejected = 0
        try:
            self.declare_parameter('udp_host', '127.0.0.1', ParameterDescriptor(read_only=True))
            self.declare_parameter('udp_port', 11573, ParameterDescriptor(read_only=True))
            self.pub = self.create_publisher(OSFData, 'osf_data', 39)
            self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self.socket.bind((self.get_parameter('udp_host').value,
                              self.get_parameter('udp_port').value))
            self.socket.settimeout(.1)
            self.loop_thread = threading.Thread(target=self.loop, daemon=True)
            self.loop_thread.start()
        except BaseException:
            self.destroy_node()
            raise

    def loop(self):
        while not self.stopping.is_set():
            try:
                data, _ = self.socket.recvfrom(65535)
                for message in decode_datagram(data):
                    self.pub.publish(message)
            except socket.timeout:
                continue
            except (ValueError, struct.error, OverflowError):
                self.rejected += 1
            except OSError:
                if self.stopping.is_set():
                    break
                raise

    def destroy_node(self):
        if self._closed:
            return
        self._closed = True
        try:
            self.stopping.set()
            if self.socket is not None:
                self.socket.close()
            if self.loop_thread is not None and self.loop_thread.is_alive():
                self.loop_thread.join(1)
        finally:
            super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = OSFReceiver()
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
