import rclpy
from rclpy.node import Node

from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Polygon, Point32

import numpy as np

from .filter_scan import filter_polar
from .local_minima import find_minima
from .obstacle_real_location import location

class LaserScanerNode(Node):

    def __init__(self):
        self.scan_data = None

        super().__init__('laser_scanner_node')

        self.pub = self.create_publisher(
            Polygon,
            '/obstacle_locations',
            10
        )

        self.sub = self.create_subscription(
            LaserScan,
            '/scan',
            self.listener_callback,
            10
        )

        self.timer = self.create_timer(0.1, self.timer_callback)

    def timer_callback(self):
        # publish converted data
        if self.scan_data:
            self.pub.publish(self.scan_data)
        
    def listener_callback(self, msg: LaserScan):
        # get init data
        if not self.scan_data:
            msg = polar_manipulation(msg)
            msg = find_minima(msg)
            
            #add in pathplanning for obs x and y
            locations = [location(0, -0.5 , 90, range, azimuth) for range, azimuth in msg] 
            locations = filter_polar(locations)
            
            output = Polygon()

            for x, y in locations:
                point = Point32()

                point.x = x
                point.y = y

                output.points.append(point)

            self.scan_data = output
def polar_manipulation(scan_msg: LaserScan):
    angle_min = scan_msg.angle_min
    angle_max = scan_msg.angle_max
    angle_increment = scan_msg.angle_increment

    amount = int((abs(angle_max) + abs(angle_min)) // angle_increment)

    return [
        (dist, ang) for dist, ang in zip(scan_msg.ranges, np.linspace(angle_min, angle_max, amount))
    ]


def main(args=None):
    rclpy.init(args=args)

    laser_scanner_node = LaserScanerNode()

    rclpy.spin(laser_scanner_node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    laser_scanner_node.destroy_node()
    rclpy.shutdown()




if __name__== '__main__':
    main()