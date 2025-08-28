import rclpy
from rclpy.node import Node

from interface_pkg.msg import OSFData
from geometry_msgs.msg import Point

import socket
import struct
import threading

SERVER_IP = "127.0.0.1"
SERVER_PORT = 11573

def readBool(data: bytes, pos: int, return_new_pos: bool = True):
    value = struct.unpack('?', data[pos:pos+1])[0]
    if return_new_pos:
        return value, pos + 1
    else:
        return value

def readInt32(data: bytes, pos: int, return_new_pos: bool = True):
    value = struct.unpack('i', data[pos:pos+4])[0]
    if return_new_pos:
        return value, pos + 4
    else:
        return value

def readFloat32(data: bytes, pos: int, return_new_pos: bool = True):
    value = struct.unpack('f', data[pos:pos+4])[0]
    if return_new_pos:
        return value, pos + 4
    else:
        return value

def readFloat64(data: bytes, pos: int, return_new_pos: bool = True):
    value = struct.unpack('d', data[pos:pos+8])[0]
    if return_new_pos:
        return value, pos + 8
    else:
        return value

def readVector2(data: bytes, pos: int, return_new_pos: bool = True):
    value = (readFloat32(data, pos, False), readFloat32(data, pos+4, False))
    if return_new_pos:
        return value, pos + 8
    else:
        return value

def readVector3(data: bytes, pos: int, return_new_pos: bool = True):
    value = (readFloat32(data, pos, False), -readFloat32(data, pos+4, False), readFloat32(data, pos+8, False))
    if return_new_pos:
        return value, pos + 12
    else:
        return value

def readQuaternion(data: bytes, pos: int, return_new_pos: bool = True):
    value = (readFloat32(data, pos, False), readFloat32(data, pos+4, False), \
             readFloat32(data, pos+8, False), readFloat32(data, pos+12, False))
    if return_new_pos:
        return value, pos + 16
    else:
        return value

class OSFReceiver(Node):
    def __init__(self):
        super().__init__('osf_receiver_node')
        self.pub = self.create_publisher(OSFData, 'osf_data', 10)
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.socket.bind((SERVER_IP, SERVER_PORT))
        self.loop_thread = threading.Thread(target = self.loop, daemon=True)
        self.loop_thread.start()

    def loop(self):
        while True:
            data, addr = self.socket.recvfrom(1786)
            
            # decode data
            msg = OSFData()
            pos = 0

            msg.time, pos = readFloat64(data, pos)
            msg.id, pos = readInt32(data, pos)

            msg.resolution, pos = readVector2(data, pos)

            msg.right_eye_open, pos = readFloat32(data, pos)
            msg.left_eye_open, pos = readFloat32(data, pos)

            msg.got_3d_points, pos = readBool(data, pos)

            msg.fit_3d_error, pos = readFloat32(data, pos)
            msg.raw_quaternion, pos = readQuaternion(data, pos)
            msg.raw_euler, pos = readVector3(data, pos)

            msg.rotation = msg.raw_euler
            msg.rotation[2] = (msg.rotation[2] - 90) % 360
            msg.rotation[0] = -(msg.rotation[0] + 180) % 360

            x, y, z = readVector3(data, pos, False)
            pos += 12
            msg.translation = (-y, x, -z)
            
            for i in range(68):
                msg.confidence[i], pos = readFloat32(data, pos)

            for i in range(68):
                pnts, pos = readVector2(data, pos)
                msg.points[i] = Point()
                msg.points[i].x, msg.points[i].y = pnts
                msg.points[i].z = float(0)
            
            for i in range(70):
                pnts3d, pos = readVector3(data, pos)
                msg.points3d[i] = Point()
                msg.points3d[i].x, msg.points3d[i].y, msg.points3d[i].z = pnts3d

            # TODO: Quaternion calculation to implement gaze detection
            msg.right_gaze = (0, 0, 0, 0)
            msg.left_gaze = (0, 0, 0, 0)

            msg.eye_left, pos = readFloat32(data, pos)
            msg.eye_right, pos = readFloat32(data, pos)
            msg.eyebrow_steepness_left, pos = readFloat32(data, pos)
            msg.eyebrow_updown_left, pos = readFloat32(data, pos)
            msg.eyebrow_quirk_left, pos = readFloat32(data, pos)
            msg.eyebrow_steepness_right, pos = readFloat32(data, pos)
            msg.eyebrow_updown_right, pos = readFloat32(data, pos)
            msg.eyebrow_quirk_right, pos = readFloat32(data, pos)
            msg.mouth_corner_updown_left, pos = readFloat32(data, pos)
            msg.mouth_corner_inout_left, pos = readFloat32(data, pos)
            msg.mouth_corner_updown_right, pos = readFloat32(data, pos)
            msg.mouth_corner_inout_right, pos = readFloat32(data, pos)
            msg.mouth_open, pos = readFloat32(data, pos)
            msg.mouth_wide, pos = readFloat32(data, pos)
            self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = OSFReceiver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Keyboard Interrupt')
    finally:
        node.destroy_node()
        rclpy.shutdown()
