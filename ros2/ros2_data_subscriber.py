# this file from from chatGPT

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Int32, Int16, Float32, Bool
from basic_srv_types.msg import EulerAngles
from sensor_msgs.msg import NavSatFix  # Example topic type

class ROS2DataSubscriber(Node):
    def __init__(self):
        super().__init__('ros2_data_subscriber')
        
        # Data storage dictionary
        self.data = {
            "gps_data": None,
            "cht" : 0,
            "egt" : 0,
            "altitude" : 0,
            "fuel" : 0.0,
            "ldoor" : False,
            "rdoor" : False,
            "flgnd" : False,
            "frgnd" : False,
            "rlgnd" : False,
            "rrgnd" : False,
            "erpm" : 0,
            "rrpm" : 0,
            "attitude" : None
        }
        
        # Subscription to topics
        self.create_subscription(NavSatFix, 'gps_topic', self.gps_callback, 10)
        self.create_subscription(Int16, 'cht', self.cht_callback,10)
        self.create_subscription(Int16, 'egt', self.egt_callback,10)
        self.create_subscription(Int16, 'altitude', self.alt_callback,10)
        self.create_subscription(Float32, 'fuel', self.fuel_callback,10)
        self.create_subscription(Bool, 'door_state/front', self.ldoor_callback,10)
        self.create_subscription(Bool, 'door_state/rear', self.rdoor_callback,10)
        self.create_subscription(Bool, 'ground_state/frontleft', self.flgnd_callback,10)
        self.create_subscription(Bool, 'ground_state/frontright', self.frgnd_callback,10)
        self.create_subscription(Bool, 'ground_state/rearleft', self.rlgnd_callback,10)
        self.create_subscription(Bool, 'ground_state/rearright', self.rrgnd_callback,10)
        self.create_subscription(Int16, 'erpm', self.erpm_callback,10)
        self.create_subscription(EulerAngles, '/craft_attitude/euler_angles_degrees', self.attitude_callback,10)

    def cht_callback(self, msg):
        self.data['cht'] = msg.data

    def egt_callback(self, msg):
        self.data['egt'] = msg.data

    def alt_callback(self, msg):
        self.data['altitude'] = msg.data

    def fuel_callback(self, msg):
        self.data['fuel'] = int(msg.data * 100)

    def ldoor_callback(self, msg):
        self.data['ldoor'] = msg.data

    def rdoor_callback(self, msg):
        self.data['rdoor'] = msg.data

    def flgnd_callback(self, msg):
        self.data['flgnd'] = msg.data

    def frgnd_callback(self, msg):
        self.data['frgnd'] = msg.data

    def rlgnd_callback(self, msg):
        self.data['rlgnd'] = msg.data

    def rrgnd_callback(self, msg):
        self.data['rrgnd'] = msg.data

    def erpm_callback(self, msg):
        self.data['erpm'] = msg.data
        self.data['rrpm'] = int(self.data['erpm'] * (540/6000))

    def attitude_callback(self, msg):
        self.data['attitude'] = { 'roll': msg.roll + 90, 'pitch': msg.pitch, 'yaw': msg.heading }
       
    def gps_callback(self, msg):
        self.data['gps_data'] = { 'latitude': msg.latitude, 'longitude': msg.longitude, 'altitude': msg.altitude }

        #self.get_logger().info(f'GPS data received: {msg.latitude}, {msg.longitude}, {msg.altitude}')
  
    def get_data(self):
        return self.data  # Expose data for API
