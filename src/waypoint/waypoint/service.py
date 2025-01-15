from waypoint.waypoint import Waypoint

from geometry_msgs.msg import Point
from custom_msgs.srv import WaypointServ

import rclpy
from rclpy.node import Node

import json

class WaypointService(Node):

    def __init__(self):
        super().__init__('waypoint_service')

        with open('src/waypoint/waypoint/waypoints.json') as json_file:
            data = json.load(json_file)

        self.waypoints: list[Waypoint] = [Waypoint(point["x"], point["y"]) for point in data]

        self.srv = self.create_service(WaypointServ, 'waypoint', self.waypoint_callback)

    def waypoint_callback(self, request, response: Point):
        self.get_logger().info('Incoming request\na: %d b: %d' % (request.position.center.x, request.position.center.y))

        point = request.position.center

        # Check if the waypoints are within range
        if self.waypoints[0].within_range(point.x, point.y):

            # Circular for now
            self.waypoints.append(self.waypoints.pop(0))

        # Create the response
        response.x = self.waypoints[0].x
        response.y = self.waypoints[0].y
        response.z = 0

        return response


def main(args=None):
    rclpy.init(args=args)

    waypoint_service = WaypointService()

    rclpy.spin(waypoint_service)

    rclpy.shutdown()