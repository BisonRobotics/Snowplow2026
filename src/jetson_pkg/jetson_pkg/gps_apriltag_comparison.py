import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from jetson_pkg.apriltag_interpretation import apriltag_interpretation

import math
import numpy as np
import threading
import cv2

#how off the coordinates can be from each other
distanceError = 0.25 

class coordinateComparison(Node):
    def __init__(self):
        super().__init__('coordinate_comparison_node')
        self.get_logger().info('Coordinate Comparison Node has started.')
    
        self.latest_apriltag_pose = None
        self.latest_gps_pose = None

        self.apriltag_subscriber = self.create_subscription(
            Twist,
            '/apriltag',
            self.apriltag_callback,
            10
        )

        #FIXME when you create the publisher for the gps
        self.gps_subscriber = self.create_subscription(
            Twist,
            '/gps',
            self.gps_callback,
            10
        )


    def apriltag_callback(self, msg: Twist):
        self.latest_apriltag_pose = msg

    def gps_callback(self, msg: Twist):
        self.latest_gps_pose = msg

    #compares the apriltag and gps and checks if they are within the valid range4
    def coords_distance(self, apriltag_coords: Twist, gps_coords: Twist):
        apriltagX = apriltag_coords.linear.x
        apriltagY = apriltag_coords.linear.y

        gpsX = gps_coords.linear.x
        gpsY = gps_coords.linear.y

        #turns the x and y into a circle
        distance = math.sqrt((apriltagX - gpsX)**2 + (apriltagY - gpsY)**2)
        return distance


    def is_coords_in_range(self, distance):
        return distance <= distanceError
    
    
    def coords_comparison(self):
        if self.latest_gps_pose is None or self.latest_apriltag_pose is None:
            return
        
        poseDifference = self.coords_distance(self.latest_apriltag_pose, self.latest_gps_pose)

        if self.is_coords_in_range(poseDifference):
            self.get_logger().info(f'Coords are in range. Distance from each other: {poseDifference: .2f}')
        else:
            self.get_logger().warn(f'Coords are out of range. they are {poseDifference: .2f}m. apart')
        

def main(args=None):
    rclpy.init(args=args)
    node = coordinateComparison()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
    
if __name__ == '__main__':
    main()
