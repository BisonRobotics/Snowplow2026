import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist, Vector3
from jetson_pkg.apriltag_interpretation import apriltag_interpretation

import math
import numpy as np
import threading
import cv2


class gpsConversion(Node):
    def __init__(self):
        super().__init__('gps_conversion')
        self.gps_conversion_publisher_ = self.create_publisher(Twist, '/gps_conversion', 10)
        timer_period = 0.5

        #needs to be hardcodded to the real field when we practice
        self.leftCornerLatitude = 44.973215
        self.leftCornerLongitude = -93.291071
        #bottom left corner represents -8,0
        
        self.rightCornerLatitude = 44.973072        
        self.rightCornerLongitude = -93.291068
        #bottom right corner represents 8,0

        self.unitX_variables()

        self.get_logger().info('GPS Conversion Node has started.')

        self.latest_gps_pose = None

        #FIXME change to gps subscriber
        self.gps_subscriber = self.create_subscription(
            Vector3,
            '/gps_data',
            self.gps_callback,
            10
        )

        self.timer = self.create_timer(timer_period, self.timer_callback)

    def gps_callback(self, msg: Vector3):
        self.latest_gps_pose = msg

    def unitX_variables(self):

        #how much the latitude changes from one unit change of X
        #one unit of X = 1 meter (aka, change when the robot goes one unit to the left or right)
        self.UnitX_Lat = (self.rightCornerLatitude - self.leftCornerLatitude) / 16
        #how much the longitude changes from one unit change of X
        self.UnitX_Long = (self.rightCornerLongitude - self.leftCornerLongitude) / 16    

    def coords_distance(self):
        #compares latitude and longitude of robot to (0,0) 
        self.deltaLatitude = self.latest_gps_pose.y - ((self.rightCornerLatitude + self.leftCornerLatitude) / 2)
        self.deltaLongitude = self.latest_gps_pose.x - ((self.rightCornerLongitude + self.leftCornerLongitude) / 2)

    def timer_callback(self):
        self.coords_distance()

        #converts our latitude and longitude change to our X and Y based off of our Units of X
        UnitX_Matrix = np.array(([self.UnitX_Lat, self.UnitX_Long],
                                 [self.UnitX_Long, -self.UnitX_Lat]))
        UnitX_Matrix_inv = np.linalg.inv(UnitX_Matrix)

        deltaMatrix = np.array(([self.deltaLatitude],
                                [self.deltaLongitude]))
        
        self.result = UnitX_Matrix_inv @ deltaMatrix
        #output is our X and Y is consistent with our apriltag coordinate system 
        msg = Twist()
        msg.linear.x = self.result[0, 0]
        msg.linear.y = self.result[1, 0]
        self.gps_conversion_publisher_.publish(msg)

        self.get_logger().debug(f'{msg}')
        self.gps_conversion_publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    gps_conversion = gpsConversion()
    rclpy.spin(gps_conversion)
    
    gps_conversion.destroy_node()
    rclpy.shutdown()
    
if __name__ == '__main__':
    main()     
