# this file from from chatGPT

import rclpy
from rclpy.node import Node
from std_msgs.msg import String  # Use appropriate message types
from sensor_msgs.msg import NavSatFix  # Example topic type

class ROS2DataSubscriber(Node):
    def __init__(self):
        super().__init__('ros2_data_subscriber')
        
        # Data storage dictionary
        self.data = {
            "string_data": None,
            "gps_data": None
        }
        
        # Subscription to a string topic
        self.create_subscription(String, 'string_topic', self.string_callback, 10)
        
        # Subscription to a GPS topic
        self.create_subscription(NavSatFix, 'gps_topic', self.gps_callback, 10)

    def string_callback(self, msg):
        self.data['string_data'] = msg.data
        self.get_logger().info(f'String data received: {msg.data}')
    
    def gps_callback(self, msg):
        self.data['gps_data'] = {
            'latitude': msg.latitude,
            'longitude': msg.longitude,
            'altitude': msg.altitude
        }
        self.get_logger().info(f'GPS data received: {msg.latitude}, {msg.longitude}, {msg.altitude}')
    
    def get_data(self):
        return self.data  # Expose data for API
