"""Single shared UDP input, with bounded reads and clean shutdown."""
import socket
import struct
import threading

import rclpy
from rclpy.node import Node
from interface_pkg.msg import OSFData
from .protocol import decode_datagram


class OSFReceiver(Node):
    def __init__(self, **kwargs):
        super().__init__('osf_receiver_node', **kwargs)
        self.declare_parameter('udp_host','127.0.0.1')
        self.declare_parameter('udp_port',11573)
        self.pub=self.create_publisher(OSFData,'osf_data',1)
        self.socket=socket.socket(socket.AF_INET,socket.SOCK_DGRAM)
        self.socket.bind((self.get_parameter('udp_host').value,self.get_parameter('udp_port').value))
        self.socket.settimeout(.1);self.stopping=threading.Event();self.rejected=0
        self.loop_thread=threading.Thread(target=self.loop,daemon=True);self.loop_thread.start()

    def loop(self):
        while not self.stopping.is_set():
            try:
                data,_=self.socket.recvfrom(65535)
                for message in decode_datagram(data):
                    self.pub.publish(message)
            except socket.timeout:
                continue
            except (ValueError,struct.error,OverflowError):
                self.rejected+=1
            except OSError:
                if self.stopping.is_set():break
                raise

    def destroy_node(self):
        self.stopping.set();self.socket.close();self.loop_thread.join(1)
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args);node=OSFReceiver()
    try:rclpy.spin(node)
    except KeyboardInterrupt:pass
    finally:
        node.destroy_node()
        if rclpy.ok():rclpy.shutdown()
