import rclpy
from rclpy.node import Node
from interface_pkg.msg import ProximityLiveData, ProximityTrigger
import board
import busio
import time
import adafruit_apds9960.apds9960

READS_PER_SEC = 48
THRESHOLD = 150

class APDS9960InputNode(Node):
    def __init__(self):
        super().__init__('APDS9960_input_node')
        self.i2c = busio.I2C(board.SCL, board.SDA)
        self.sensor = adafruit_apds9960.apds9960.APDS9960(self.i2c)
        self.proximity_live_pub = self.create_publisher(ProximityLiveData, 'proximity_live_data', 10)
        self.proximity_trigger_pub = self.create_publisher(ProximityTrigger, 'proximity_trigger', 10)
        self.previous_proximity = 255
        self.create_timer(1 // READS_PER_SEC, self.read_cb)
        self.sensor.enable_proximity = True
    
    def read_cb(self):
        proximity = self.sensor.proximity
        if proximity < THRESHOLD and self.previous_proximity >= THRESHOLD:
            trigger = ProximityTrigger()
            trigger.state = 'falling'
            self.proximity_trigger_pub.publish(trigger)
        elif proximity >= THRESHOLD and self.previous_proximity < THRESHOLD:
            trigger = ProximityTrigger()
            trigger.state = 'rising'
            self.proximity_trigger_pub.publish(trigger)
        
        livedata = ProximityLiveData()
        livedata.proximity = proximity
        self.proximity_live_pub.publish(livedata)

        self.previous_proximity = proximity

def main(args=None):
    rclpy.init(args=args)
    node = APDS9960InputNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('KeyboardInterrupt received, terminating node...')
    finally:
        node.destroy_node()
        rclpy.shutdown()
