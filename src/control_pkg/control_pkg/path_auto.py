from typing import Literal
import rclpy
import time

from control_pkg.cone_zone_detection import find_zone
from control_pkg.path_planning import turn_path, Path
from geometry_msgs.msg import Point32, Polygon, Twist
from rclpy.node import Node
from std_msgs.msg import Float32, Int8
from utilities.tools import Tools

MAX_TURN_ANGLE = 18.624

class PathAuto(Node):
    
    def __init__(self):
        super().__init__('path_auto')
        
        self.degree_deadband = 0.2
        self.obstacles: Polygon | None = None
        self.obstacles_updated = False
        self.path: Path | None = None
        self.pivot_position = 0
        self.pivot_updated = False
        self.position = None
        self.position_updated = False
        self.running = False
        self.state = 0
        self.sub_state = 0

        self.waypoint_array = [Twist() for _ in range(8)]

        # This for loop generates the waypoints for the squares in the course.
        for i, waypoint in enumerate(self.waypoint_array, start=1):
            waypoint.linear.x = 2.0*(-(i%2)+0.5)*(5-((i+i%2)/2)) - ((-(i%2) + 0.5)*(2))
            waypoint.linear.z = 2.0
            waypoint.angular.y = (i % 2) * 180.0
            self.get_logger().info(f"Waypoint {i}, X: {waypoint.linear.x}, Z: {waypoint.linear.z}, Y: {waypoint.angular.y}")

        self.left_waypoint: Twist | None = None
        self.right_waypoint: Twist | None = None

        self.left_swoop = True
        self.right_swoop = True
        
        self.speed_publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.pivot_publisher = self.create_publisher(Int8, '/vehicle/pivot', 10)
        
        self.create_subscription(Float32, '/sensor/pivot', self.update_pivot, 10)
        self.create_subscription(Twist, '/apriltag', self.update_position, 10)
        self.create_subscription(Polygon, '/obstacle_locations', self.update_obstacles, 10)

        self.create_timer(0.01, self.determine_control)
        
    def update_pivot(self, msg: Float32):
        self.pivot_updated = True
        self.pivot_position = Tools.potentiometer_to_degrees(msg.data)
        
    def update_position(self, msg: Twist):
        self.position_updated = True
        self.position = msg

    def update_obstacles(self, msg: Polygon) -> None:
        self.obstacles_updated = True
        self.obstacles = msg
        
    def update_waypoints(self):
        if self.obstacles is None:
            raise ValueError("Obstacles have not been set.")

        internal_cone: int = 1  # Area the cone is located internally
        external_cone: int = -3  # Area the cone is located externally

        for point in self.obstacles.points:
            x = point.x
            y = point.y
            zone = find_zone(x, y)

            # If zero, it is out of playfield
            if zone == 0:
                continue

            # Within the playfield squares
            if zone > 0:
                internal_cone = zone
                continue
            
            # Else, outer board by apriltags
            # Determines whether it is immediately left or right of garage
            if external_cone == -1 or external_cone == -7:
                continue  
            
            external_cone = zone

        self.get_logger().info(f"Internal cone: {internal_cone}, External cone: {external_cone}")

        # Setting left and right waypoints based on detected cones
        # If the internal cone is greater than 8, it means internal cones are in the center
        if internal_cone > 8:
            return
        
        if internal_cone % 2 == 1:
            self.left_waypoint = self.waypoint_array[internal_cone - 1]
            self.left_swoop = False
        else:
            self.right_waypoint = self.waypoint_array[internal_cone - 1]
            self.right_swoop = False

    def determine_control(self):
        if not self.pivot_updated or not self.position_updated or not self.obstacles_updated:
            return
        
        self.get_logger().info(f"State: {self.state}")

        if self.state == 0:
            self.update_waypoints()

        match self.state:
            case 0:
                self.get_logger().info('turning to 0 degrees')
                self.turn_to_degrees(0)
                return
            case 1:
                self.get_logger().info('running first segment of the path')
                self.drive_distance(1, 3)
                return
            case 2:
                self.get_logger().info('waiting for 2 seconds')
                self.wait_time(2)
                return
            case 3:
                self.get_logger().info('running first segment of the path')
                self.drive_distance(-1, 2.75)
                return
            case 4:
                self.get_logger().info('waiting for 2 seconds')
                self.wait_time(2)
                return
            case 5:
                self.get_logger().info('turning to 0 degrees')
                self.turn_to_degrees(0)
                return
            case 6:
                self.get_logger().info('waiting for 2 seconds')
                self.wait_time(2)
                return
            case 7:
                self.get_logger().info('generating path')
                self.path = turn_path(start_point=(self.position.linear.x, self.position.linear.z), start_direction=self.position.angular.y, end_point=(self.left_waypoint.linear.x, self.left_waypoint.linear.z), end_direction=self.left_waypoint.angular.y)
                self.state += 1
                return
            case 8:
                self.get_logger().info('running first segment of the path')
                if self.path.segment1.distance < 0.2:
                    self.state += 3
                else:
                    self.turn_to_degrees(self.path.segment1.turn_angle * MAX_TURN_ANGLE)
                return
            case 9:
                self.get_logger().info('waiting for 2 seconds')
                self.wait_time(2)
                return
            case 10:
                self.get_logger().info('running first segment of the path')
                self.drive_distance(self.path.segment1.direction, self.path.segment1.distance)
                return
            case 11:
                self.get_logger().info('waiting for 2 seconds')
                self.wait_time(2)
                return
            case 12:
                self.get_logger().info('running second segment of the path')
                if self.path.segment2.distance < 0.2:
                    self.state += 3
                else:
                    self.turn_to_degrees(self.path.segment2.turn_angle * MAX_TURN_ANGLE)
                return
            case 13:
                self.get_logger().info('waiting for 2 seconds')
                self.wait_time(2)
                return
            case 14:
                self.get_logger().info('running second segment of the path')
                self.drive_distance(self.path.segment2.direction, self.path.segment2.distance)
                return
            case 15:
                self.get_logger().info('waiting for 2 seconds')
                self.wait_time(2)
                return
            case 16:
                self.get_logger().info('running the third segment of the path')
                if self.path.segment3.distance < 0.2:
                    self.state += 3
                else:
                    self.turn_to_degrees(self.path.segment3.turn_angle * MAX_TURN_ANGLE)
                return
            case 17:
                self.get_logger().info('waiting for 2 seconds')
                self.wait_time(2)
                return
            case 18:
                self.get_logger().info('running the third segment of the path')
                self.drive_distance(self.path.segment3.direction, self.path.segment3.distance)
                return
            case 19:
                self.get_logger().info('waiting for 2 seconds')
                self.wait_time(2)
                self.path = None
                return
            case 20:
                self.get_logger().info('turning to 0 degrees')
                self.turn_to_degrees(0)
                return
            case 21:
                self.get_logger().info('waiting for 2 seconds')
                self.wait_time(2)
                return
            case 22:
                self.get_logger().info('driving at 100% speed for 2 meters')
                if self.left_swoop:
                    self.drive_distance(1, 2)
                else:
                    self.state += 11
                return
            case 23:
                self.get_logger().info('waiting for 2 seconds')
                self.wait_time(2)
                return
            case 24:
                self.get_logger().info('turning to -22 degrees')
                self.turn_to_degrees(-22)
                return
            case 25:
                self.get_logger().info('waiting for 2 seconds')
                self.wait_time(2)
                return
            case 26:
                self.get_logger().info('driving at 100% speed for 1/8th of a turn')
                self.drive_distance(1, 3.14 * 1.90475 / 4)
                return
            case 27:
                self.get_logger().info('waiting for 2 seconds')
                self.wait_time(2)
                return
            case 28:
                self.get_logger().info('driving at -100% speed for 1/8th of a turn')
                self.drive_distance(-1, 3.14 * 1.90475 / 4)
                return
            case 29:
                self.get_logger().info('waiting for 2 seconds')
                self.wait_time(2)
                return
            case 30:
                self.get_logger().info('turning to 0 degrees')
                self.turn_to_degrees(0)
                return
            case 31:
                self.get_logger().info('waiting for 2 seconds')
                self.wait_time(2)
                return
            case 32:
                self.get_logger().info('driving at -100% speed for 1.5 meters')
                self.drive_distance(-1, 1.5)
                return
            case 33:
                self.get_logger().info('waiting for 2 seconds')
                self.wait_time(2)
                return
            case 34:
                self.get_logger().info('turning to -MAX_TURN_ANGLE degrees')
                self.turn_to_degrees(-MAX_TURN_ANGLE)
                return
            case 35:
                self.get_logger().info('waiting for 2 seconds')
                self.wait_time(2)
                return
            case 36:
                self.get_logger().info('driving at -100% speed for 90 degree turn')
                self.drive_distance(-1, 2.25 * 3.14 / 2)
                return
            case 37:
                self.get_logger().info('waiting for 2 seconds')
                self.wait_time(2)
                return
            case 38:
                self.get_logger().info('turning to 0 degrees')
                self.turn_to_degrees(0)
                return
            case 39:
                self.get_logger().info('waiting for 2 seconds')
                self.wait_time(2)
                return
            case 40:
                self.get_logger().info('driving at -100% speed for 2 meters')
                self.drive_distance(-1, 1.5)
                return
            case 41:
                self.get_logger().info('waiting for 2 seconds')
                self.wait_time(2)
                return
            case 42:
                self.get_logger().info('generating path')
                self.path = turn_path(start_point=(self.position.linear.x, self.position.linear.z), start_direction=self.position.angular.y, end_point=(self.right_waypoint.linear.x, self.right_waypoint.linear.z), end_direction=self.right_waypoint.angular.y)
                self.state += 1
                return
            case 43:
                self.get_logger().info('running first segment of the path')
                if self.path.segment1.distance < 0.2:
                    self.state += 3
                else:
                    self.turn_to_degrees(self.path.segment1.turn_angle * MAX_TURN_ANGLE)
                return
            case 44:
                self.get_logger().info('waiting for 2 seconds')
                self.wait_time(2)
                return
            case 45:
                self.get_logger().info('running first segment of the path')
                self.drive_distance(self.path.segment1.direction, self.path.segment1.distance)
                return
            case 46:
                self.get_logger().info('waiting for 2 seconds')
                self.wait_time(2)
                return
            case 47:
                self.get_logger().info('running second segment of the path')
                if self.path.segment2.distance < 0.2:
                    self.state += 3
                else:
                    self.turn_to_degrees(self.path.segment2.turn_angle * MAX_TURN_ANGLE)
                return
            case 48:
                self.get_logger().info('waiting for 2 seconds')
                self.wait_time(2)
                return
            case 49:
                self.get_logger().info('running second segment of the path')
                self.drive_distance(self.path.segment2.direction, self.path.segment2.distance)
                return
            case 50:
                self.get_logger().info('waiting for 2 seconds')
                self.wait_time(2)
                return
            case 51:
                self.get_logger().info('running the third segment of the path')
                if self.path.segment3.distance < 0.2:
                    self.state += 3
                else:
                    self.turn_to_degrees(self.path.segment3.turn_angle * MAX_TURN_ANGLE)
                return
            case 52:
                self.get_logger().info('waiting for 2 seconds')
                self.wait_time(2)
                return
            case 53:
                self.get_logger().info('running the third segment of the path')
                self.drive_distance(self.path.segment3.direction, self.path.segment3.distance)
                return
            case 54:
                self.get_logger().info('waiting for 2 seconds')
                self.wait_time(2)
                self.path = None
                return
            case 55:
                self.get_logger().info('turning to 0 degrees')
                self.turn_to_degrees(0)
                return
            case 56:
                self.get_logger().info('waiting for 2 seconds')
                self.wait_time(2)
                return
            case 57:
                self.get_logger().info('driving at 100% speed for 2 meters')
                if self.right_swoop:
                    self.drive_distance(1, 2)
                else:
                    self.state += 11
                return
            case 58:
                self.get_logger().info('waiting for 2 seconds')
                self.wait_time(2)
                return
            case 59:
                self.get_logger().info('turning to -22 degrees')
                self.turn_to_degrees(-22)
                return
            case 60:
                self.get_logger().info('waiting for 2 seconds')
                self.wait_time(2)
                return
            case 61:
                self.get_logger().info('driving at 100% speed for 1/8th of a turn')
                self.drive_distance(1, 3.14 * 1.90475 / 4)
                return
            case 62:
                self.get_logger().info('waiting for 2 seconds')
                self.wait_time(2)
                return
            case 63:
                self.get_logger().info('driving at -100% speed for 1/8th of a turn')
                self.drive_distance(-1, 3.14 * 1.90475 / 4)
                return
            case 64:
                self.get_logger().info('waiting for 2 seconds')
                self.wait_time(2)
                return
            case 65:
                self.get_logger().info('turning to 0 degrees')
                self.turn_to_degrees(0)
                return
            case 66:
                self.get_logger().info('waiting for 2 seconds')
                self.wait_time(2)
                return
            case 67:
                self.get_logger().info('driving at -100% speed for 2.5 meters')
                self.drive_distance(-1, 2.5)
                return
            case 68:
                self.get_logger().info('waiting for 2 seconds')
                self.wait_time(2)
                return
            case 69:
                self.get_logger().info('turning to MAX_TURN_ANGLE degrees')
                self.turn_to_degrees(MAX_TURN_ANGLE)
                return
            case 70:
                self.get_logger().info('waiting for 2 seconds')
                self.wait_time(2)
                return
            case 71:
                self.get_logger().info('driving at -100% speed for 90 degree turn')
                self.drive_distance(-1, 2.25 * 3.14 / 2)
                return
            case 72:
                self.get_logger().info('waiting for 2 seconds')
                self.wait_time(2)
                return
            case 73:
                self.get_logger().info('turning to 0 degrees')
                self.turn_to_degrees(0)
                return
            case 74:
                self.get_logger().info('waiting for 2 seconds')
                self.wait_time(2)
                return
            case 75:
                self.get_logger().info('driving at -100% speed for 2 meters')
                self.drive_distance(-1.0, 1.5)
                return
            case _:
                self.get_logger().info('done')
                return
            
    def turn_to_degrees(self, degrees: float):
        msg = Int8()
        if abs(self.pivot_position - degrees) <= self.degree_deadband:
            self.state += 1
            msg.data = 0
        else:
            msg.data = 1 if self.pivot_position < degrees else -1
        self.pivot_publisher.publish(msg)
        
    def drive_distance(self, speed: Literal[-1, 1], distance: float):
        run_time = 0
        if distance >= 2/3:
            run_time = (1/3) + distance - (2/3)
        else:
            run_time = ((distance / (2/3)) ** 2) / 3
        self.drive_speed_time(speed, run_time)
    
    def drive_speed_time(self, speed: float, run_time: float):
        if not self.running:
            self.end_time = time.time() + run_time
            self.running = True
        msg = Twist()
        if time.time() >= self.end_time:
            msg.linear.x = 0.0
            self.state += 1
            self.running = False
        else:
            msg.linear.x = float(speed)
        self.speed_publisher.publish(msg)

    def wait_time(self, run_time: float):
        if not self.running:
            self.end_time = time.time() + run_time
            self.running = True
        if time.time() >= self.end_time:
            self.state += 1
            self.running = False

def main():
    rclpy.init()
    
    node = PathAuto()
    rclpy.spin(node)
    
    node.destroy_node()
    rclpy.shutdown()
    
if __name__ == '__main__':
    main()