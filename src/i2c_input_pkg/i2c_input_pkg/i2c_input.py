import rclpy
from rclpy.node import Node
from BMI160_i2c import Driver

from interface_pkg.msg import GyroAccelLiveData

READS_PER_SEC = 48

class I2CInputNode(Node):
    def __init__(self):
        super().__init__('i2c_input_node')
        self.sensor = Driver(0x69)
        self.data_pub = self.create_publisher(GyroAccelLiveData, 'gyro_accel_live_data', 10)
        self.create_timer(1 / READS_PER_SEC, self.data_cb)
    
    def data_cb(self):
        data = GyroAccelLiveData()
        data.gx, data.gy, data.gz, data.ax, data.ay, data.az = map(float, self.sensor.getMotion6())
        self.data_pub.publish(data)
        self.get_logger().info(f'{data.gx, data.gy, data.gz, data.ax, data.ay, data.az}')

def main(args=None):
    rclpy.init(args=args)
    node = I2CInputNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('KeyboardInterrupt detected, halting...')
    finally:
        node.destroy_node()
        rclpy.shutdown()