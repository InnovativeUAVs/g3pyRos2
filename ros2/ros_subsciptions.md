'''

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32, Int16, Float32, Bool
from basic_srv_types.msg import EulerAngles


       self.subscription = self.create_subscription(
            Int16,
            'cht',  
            self.cht_callback,
            10
        )
        self.subscription = self.create_subscription(
            Int16,
            'egt',  
            self.egt_callback,
            10
        )
        self.subscription = self.create_subscription(
            Int16,
            'altitude',  
            self.alt_callback,
            10
        )
        self.subscription = self.create_subscription(
            Float32,
            'fuel',  
            self.fuel_callback,
            10
        )
        self.subscription = self.create_subscription(
            Bool,
            'door_state/front',  
            self.ldoor_callback,
            10
        )
        self.subscription = self.create_subscription(
            Bool,
            'door_state/rear',  
            self.rdoor_callback,
            10
        )
        self.subscription = self.create_subscription(
            Bool,
            'ground_state/frontleft',  
            self.flgnd_callback,
            10
        )

        self.subscription = self.create_subscription(
            Bool,
            'ground_state/frontright',  
            self.frgnd_callback,
            10
        )

        self.subscription = self.create_subscription(
            Bool,
            'ground_state/rearleft',  
            self.rlgnd_callback,
            10
        )

        self.subscription = self.create_subscription(
            Bool,
            'ground_state/rearright',  
            self.rrgnd_callback,
            10
        )

        self.subscription = self.create_subscription(
            Int16,
            'erpm',  
            self.erpm_callback,
            10
        )

        self.subscription = self.create_subscription(
            EulerAngles,
            '/craft_attitude/euler_angles_degrees',
            self.attitude_callback,
            10
        )
'''