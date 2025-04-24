import rclpy
from rclpy.node import node
from interface_pkg.msg import ProximityLiveData, ProximityTrigger
import board
import busio
import time
import adafruit_apds9960.apds9960

class APDS9960InputNode(Node):
    def __init__(self):
        super().__init__('APDS9960_input_node')
        self.i2c = busio.I2C(board.SCL, board.SDA)
        self.sensor = adafruit_apds9960.apds9960.APDS9960(i2c)
        self.proximity_live_pub = self.create_publisher(ProximityLiveData, 'proximity_live_data', 10)
        self.proximity_trigger_pub = self.create_publisher(ProximityTrigger, 'proximity_trigger', 10)
        self.previous_proximity = 255
        self.
