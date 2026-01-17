import rclpy
from rclpy.node import Node

from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Polygon, Point, Point32

import numpy as np

from .filter_scan import filter_polar
from .local_minima import find_minima
from .obstacle_real_location import location

class LaserScanerNode(Node):

    def __init__(self):
        self.location_data: Point | None = None

        super().__init__('laser_scanner_node')
        self.get_logger().info("Starting Laser Scan Node")

        self.pub = self.create_publisher(
            Polygon,
            '/obstacle_locations',
            10
        )

        self.location_sub = self.create_subscription(
            Point,
            '/location_calculate',
            self.location_callback,
            10
        )

        self.scan_sub = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10
        )

    def location_callback(self, msg: Point):
        self.location_data = msg
        
    def scan_callback(self, msg: LaserScan):
        self.get_logger().info("Receive scanned data")

        if self.location_data is None:
            return
        
        self.get_logger().info(f"Current Location: x={self.location_data.x}, y={self.location_data.y}, z={self.location_data.z}")

        msg = polar_manipulation(msg)
        self.get_logger().info(f"Total scan points received: {len(msg)}", throttle_duration_sec=5.0)
        msg = find_minima(msg)
        
        self.get_logger().info(f"Total minima found: {len(msg)}", throttle_duration_sec=5.0)
        #add in pathplanning for obs x and y
        locations = [
            location(self.location_data.x, self.location_data.y, self.location_data.z, range, azimuth) for range, azimuth in msg
        ] 
        self.get_logger().info(f"Total obstacle locations calculated: {len(locations)}", throttle_duration_sec=5.0)
        locations = filter_polar(locations)
        
        output = Polygon()

        self.get_logger().info(f"Publishing {len(locations)} obstacle locations", throttle_duration_sec=5.0)

        for x, y in locations:
            point = Point32()

            point.x = x
            point.y = y

            output.points.append(point)

        self.pub.publish(output)

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