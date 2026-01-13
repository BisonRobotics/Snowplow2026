import rclpy

from rclpy.node import Node
from std_msgs.msg import Float32, Int8
from geometry_msgs.msg import Twist, Polygon, Point32 

from control_pkg.commands import Runner, Command
from control_pkg.drive_commands import DriveDistanceCommand, DriveToWaypointCommand, DriveBackwardsToWaypointCommand
from control_pkg.wait_commands import WaitCommand, WaitUntilCommand
from control_pkg.turn_command import TurnToDegreesCommand
from control_pkg.cone_zone_detection import find_zone 

class Auto(Node):
    def __init__(self):
        super().__init__('auto')
        
        # Set up ROS stuff
        self.pivot_position = None
        self.position = None
        self.obsticals = None
        
        self.pivot_position_updated = False
        self.position_updated = False
        self.obsticals_updated = False
        
        self.pivot_publisher = self.create_publisher(Int8, '/vehicle/pivot', 10)
        self.speed_publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        
        self.create_subscription(Twist, '/apriltag', self.update_position, 10)
        self.create_subscription(Float32, '/sensor/pivot', self.update_pivot_position, 10)
        self.create_subscription(Polygon, '/obstacle_locations', self.update_obsticals, 10)
        # Start runner
        self.runner = Runner()
        self.create_timer(0.01, self.runner.run)
        
        # Start auto command
        self.runner.start_command(self.get_auto_command())
        
    def get_auto_command_2(self) -> Command:
        """
        Generates and returns the command to start when auto is started
        !!!! This code has not been tested yet !!!!
        !!!! need to read the obstacle data from obstical ovidance branch instead of hard coded cone location !!!!
        

        Returns:
            Command: Command to start when auto is started
        """
        # Waypoint after left turn out
        left_waypoint = Twist()
        left_waypoint.linear.x = -2.5
        left_waypoint.linear.z = 2
        left_waypoint.angular.y = 180
        
        # Waypoint after right turn out
        right_waypoint = Twist()
        right_waypoint.linear.x = 2.5
        right_waypoint.linear.z = 2
        right_waypoint.angular.y = 0

        # Garage Waypoint
        garage_waypoint = Twist()
        garage_waypoint.linear.x = 0
        garage_waypoint.linear.z = -1.2
        garage_waypoint.angular.y = 90

        waypoint1, waypoint2, waypoint3, waypoint4, waypoint5, waypoint6, waypoint7, waypoint8 = Twist()
        waypoint_array = [waypoint1, waypoint2, waypoint3, waypoint4, waypoint5, waypoint6, waypoint7, waypoint8]
        i = 1
        for waypoint in waypoint_array:
            waypoint.linear.x = 2*(-(i%2)+0.5)*(5-((i+i%2)/2)) - (i%2) + 0.5
            waypoint.linear.z = 2
            waypoint.angular.y = 90
            i += 1
        
        
        # Factory functions for removing redundancy
        wait = lambda time_seconds : WaitCommand(time_seconds)
        turn_to_degrees = lambda degrees : TurnToDegreesCommand(degrees, self.get_pivot_position, self.drive_pivot)
        drive_to_waypoint = lambda waypoint : DriveToWaypointCommand(waypoint, self.get_position, self.get_pivot_position, self.drive_pivot, self.drive)
        drive_backwards_to_waypoint = lambda waypoint : DriveBackwardsToWaypointCommand(waypoint, self.get_position, self.get_pivot_position, self.drive_pivot, self.drive)
        drive_distance = lambda speed, distance : DriveDistanceCommand(speed, distance, self.drive)
        wait_until = lambda condition : WaitUntilCommand(condition)

        # Creating segments

        ConeInternal  # Area the cone is located internally
        ConeExternal  # Area the cone is located externally
        
        for point in self.obsticals.points:
            x = point.x
            y = point.y
            zone = find_zone(x,y)
            if(zone > 0):
                ConeInternal = zone
            elif(zone < 0):
                ConeExternal = zone

        
        TurningAngle = 18.624 

        #Segment 1
        if(ConeInternal < 8.5):
            # Path A
            segment1 = turn_to_degrees(TurningAngle)\
                    .and_then(wait(2))\
                    .and_then(drive_distance(1, 0.2))\
                    .and_then(wait(2))\
                    .and_then(turn_to_degrees(-TurningAngle))\
                    .and_then(wait(2))\
                    .and_then(drive_distance(1, 0.2))\
                    .and_then(wait(2))\
                    .and_then(turn_to_degrees(0))\
                    .and_then(wait(2))\
                    .and_then(drive_distance(1, 2.75))\
                    .and_then(wait(2))\
                    .and_then(drive_distance(-1, 2.75))\
                    .and_then(wait(2))\
                    .and_then(turn_to_degrees(-TurningAngle))\
                    .and_then(wait(2))\
                    .and_then(drive_distance(-1, 0.2))\
                    .and_then(wait(2))\
                    .and_then(turn_to_degrees(TurningAngle))\
                    .and_then(wait(2))\
                    .and_then(drive_distance(-1, 0.2))\
                    .and_then(wait(4))\
                    .and_then(turn_to_degrees(-TurningAngle))\
                    .and_then(wait(2))\
                    .and_then(drive_distance(1, 0.2))\
                    .and_then(wait(2))\
                    .and_then(turn_to_degrees(TurningAngle))\
                    .and_then(wait(2))\
                    .and_then(drive_distance(1, 0.2))\
                    .and_then(wait(2))\
                    .and_then(turn_to_degrees(0))\
                    .and_then(wait(2))\
                    .and_then(drive_distance(1, 2.75))\
                    .and_then(wait(2))\
                    .and_then(drive_distance(-1, 2.75))\
                    .and_then(wait(2))\
                    .and_then(turn_to_degrees(TurningAngle))\
                    .and_then(wait(2))\
                    .and_then(drive_distance(-1, 0.2))\
                    .and_then(wait(2))\
                    .and_then(turn_to_degrees(-TurningAngle))\
                    .and_then(wait(2))\
                    .and_then(drive_distance(-1, 0.2))
            
        elif (ConeInternal >8.5):
            if(ConeInternal%2 ==1 ):
                # Path B Right
                segment1 = turn_to_degrees(TurningAngle)\
                    .and_then(wait(2))\
                    .and_then(drive_distance(1, 0.2))\
                    .and_then(wait(2))\
                    .and_then(turn_to_degrees(-TurningAngle))\
                    .and_then(wait(2))\
                    .and_then(drive_distance(1, 0.2))\
                    .and_then(wait(2))\
                    .and_then(turn_to_degrees(0))\
                    .and_then(wait(2))\
                    .and_then(drive_distance(1, 2.75))\
                    .and_then(wait(2))\
                    .and_then(drive_distance(-1, 2.75))\
                    .and_then(wait(2))\
                    .and_then(turn_to_degrees(-TurningAngle))\
                    .and_then(wait(2))\
                    .and_then(drive_distance(-1, 0.2))\
                    .and_then(wait(2))\
                    .and_then(turn_to_degrees(TurningAngle))\
                    .and_then(wait(2))\
                    .and_then(drive_distance(-1, 0.2))
            else:
                # Path B Left
                segment1 = turn_to_degrees(-TurningAngle)\
                    .and_then(wait(2))\
                    .and_then(drive_distance(1, 0.2))\
                    .and_then(wait(2))\
                    .and_then(turn_to_degrees(TurningAngle))\
                    .and_then(wait(2))\
                    .and_then(drive_distance(1, 0.2))\
                    .and_then(wait(2))\
                    .and_then(turn_to_degrees(0))\
                    .and_then(wait(2))\
                    .and_then(drive_distance(1, 2.75))\
                    .and_then(wait(2))\
                    .and_then(drive_distance(-1, 2.75))\
                    .and_then(wait(2))\
                    .and_then(turn_to_degrees(TurningAngle))\
                    .and_then(wait(2))\
                    .and_then(drive_distance(-1, 0.2))\
                    .and_then(wait(2))\
                    .and_then(turn_to_degrees(-TurningAngle))\
                    .and_then(wait(2))\
                    .and_then(drive_distance(-1, 0.2))
            
            
        # Segment 2 
        if((ConeExternal == -7) and (ConeInternal%2 == 1)):
            #Path D Right
            segment2 = turn_to_degrees(0)\
                .and_then(wait(2))\
                .and_then(drive_to_waypoint(right_waypoint))\
                .and_then(wait(2))\
                .and_then(turn_to_degrees(0))\
                .and_then(wait(2))\
                .and_then(drive_distance(1, 2))\
                .and_then(wait(2))\
                .and_then(turn_to_degrees(-22))\
                .and_then(wait(2))\
                .and_then(drive_distance(1, 3.14 * 1.90475 / 4))\
                .and_then(wait(2))\
                .and_then(drive_distance(-1, 3.14 * 1.90475 / 4))\
                .and_then(wait(2))\
                .and_then(turn_to_degrees(0))\
                .and_then(wait(2))\
                .and_then(drive_distance(-1, 1.5))\
                .and_then(wait(2))\
                .and_then(turn_to_degrees(18.624))\
                .and_then(wait(2))\
                .and_then(drive_distance(-1, 2.25 * 3.14 / 2))\
                .and_then(wait(2))\
                .and_then(turn_to_degrees(0))\
                .and_then(wait(2))\
                .and_then(drive_distance(-1, 1.5))
        elif((ConeExternal == -1) and (ConeInternal%2 == 0)):
          # Path D Left
            segment2 = turn_to_degrees(0)\
                .and_then(wait(2))\
                .and_then(drive_to_waypoint(left_waypoint))\
                .and_then(wait(2))\
                .and_then(turn_to_degrees(0))\
                .and_then(wait(2))\
                .and_then(drive_distance(1, 2))\
                .and_then(wait(2))\
                .and_then(turn_to_degrees(-22))\
                .and_then(wait(2))\
                .and_then(drive_distance(1, 3.14 * 1.90475 / 4))\
                .and_then(wait(2))\
                .and_then(drive_distance(-1, 3.14 * 1.90475 / 4))\
                .and_then(wait(2))\
                .and_then(turn_to_degrees(0))\
                .and_then(wait(2))\
                .and_then(drive_distance(-1, 1.5))\
                .and_then(wait(2))\
                .and_then(turn_to_degrees(-18.624))\
                .and_then(wait(2))\
                .and_then(drive_distance(-1, 2.25 * 3.14 / 2))\
                .and_then(wait(2))\
                .and_then(turn_to_degrees(0))\
                .and_then(wait(2))\
                .and_then(drive_distance(-1, 1.5))
        else:
           if(ConeInternal%2 == 1):
               # Path C Right
                segment2 = turn_to_degrees(0)\
                    .and_then(wait(2))\
                    .and_then(drive_to_waypoint(waypoint8))\
                    .and_then(wait(2))\
                    .and_then(turn_to_degrees(0))\
                    .and_then(wait(2))\
                    .and_then(drive_backwards_to_waypoint(garage_waypoint))\
                    .and_then(wait(2))\
                    .and_then(turn_to_degrees(0))\
                    .and_then(wait(2))\
                    .and_then(drive_to_waypoint(waypoint6))\
                    .and_then(wait(2))\
                    .and_then(turn_to_degrees(0))\
                    .and_then(wait(2))\
                    .and_then(drive_backwards_to_waypoint(garage_waypoint))\
                    .and_then(wait(2))\
                    .and_then(turn_to_degrees(0))\
                    .and_then(wait(2))\
                    .and_then(drive_to_waypoint(waypoint4))\
                    .and_then(wait(2))\
                    .and_then(turn_to_degrees(0))\
                    .and_then(wait(2))\
                    .and_then(drive_backwards_to_waypoint(garage_waypoint))\
                    .and_then(wait(2))\
                    .and_then(turn_to_degrees(0))\
                    .and_then(wait(2))\
                    .and_then(drive_to_waypoint(waypoint2))\
                    .and_then(wait(2))\
                    .and_then(turn_to_degrees(0))\
                    .and_then(wait(2))\
                    .and_then(drive_backwards_to_waypoint(garage_waypoint))\
                    .and_then(wait(2))\
                    .and_then(turn_to_degrees(0))
           else:
               # Path C Left
                segment2 = turn_to_degrees(0)\
                    .and_then(wait(2))\
                    .and_then(drive_to_waypoint(waypoint7))\
                    .and_then(wait(2))\
                    .and_then(turn_to_degrees(0))\
                    .and_then(wait(2))\
                    .and_then(drive_backwards_to_waypoint(garage_waypoint))\
                    .and_then(wait(2))\
                    .and_then(turn_to_degrees(0))\
                    .and_then(wait(2))\
                    .and_then(drive_to_waypoint(waypoint5))\
                    .and_then(wait(2))\
                    .and_then(turn_to_degrees(0))\
                    .and_then(wait(2))\
                    .and_then(drive_backwards_to_waypoint(garage_waypoint))\
                    .and_then(wait(2))\
                    .and_then(turn_to_degrees(0))\
                    .and_then(wait(2))\
                    .and_then(drive_to_waypoint(waypoint3))\
                    .and_then(wait(2))\
                    .and_then(turn_to_degrees(0))\
                    .and_then(wait(2))\
                    .and_then(drive_backwards_to_waypoint(garage_waypoint))\
                    .and_then(wait(2))\
                    .and_then(turn_to_degrees(0))\
                    .and_then(wait(2))\
                    .and_then(drive_to_waypoint(waypoint1))\
                    .and_then(wait(2))\
                    .and_then(turn_to_degrees(0))\
                    .and_then(wait(2))\
                    .and_then(drive_backwards_to_waypoint(garage_waypoint))\
                    .and_then(wait(2))\
                    .and_then(turn_to_degrees(0))
        
        #Segment 3
        if((ConeExternal == -1 and ConeInternal%2 == 1) or (ConeExternal == -7 and ConeInternal%2 == 0)):
            # Path X # More Conditional Logic tobe developed.
            segment3 = turn_to_degrees(0).and_then(drive_distance(1, 2)) # PlaceHolder
        elif(ConeInternal%2 == 1):
            '''
            This code will build segment 3 with each path, 
            leaving out the square where the cone is located.
            '''
            if(not(ConeInternal == 8)):
                # Path C.1 Right
                segment3 = segment3\
                    .and_then(turn_to_degrees(0))\
                    .and_then(wait(2))\
                    .and_then(drive_to_waypoint(waypoint8))\
                    .and_then(wait(2))\
                    .and_then(turn_to_degrees(0))\
                    .and_then(wait(2))\
                    .and_then(drive_backwards_to_waypoint(garage_waypoint))\
                    .and_then(wait(2))\
                    .and_then(turn_to_degrees(0))
            if(not(ConeInternal == 6)):
                # Path C.2 Right
                segment3 = segment3\
                    .and_then(turn_to_degrees(0))\
                    .and_then(wait(2))\
                    .and_then(drive_to_waypoint(waypoint6))\
                    .and_then(wait(2))\
                    .and_then(turn_to_degrees(0))\
                    .and_then(wait(2))\
                    .and_then(drive_backwards_to_waypoint(garage_waypoint))\
                    .and_then(wait(2))\
                    .and_then(turn_to_degrees(0))
            if(not(ConeInternal == 4)):
                # Path C.3 Right
                segment3 = segment3\
                    .and_then(turn_to_degrees(0))\
                    .and_then(wait(2))\
                    .and_then(drive_to_waypoint(waypoint4))\
                    .and_then(wait(2))\
                    .and_then(turn_to_degrees(0))\
                    .and_then(wait(2))\
                    .and_then(drive_backwards_to_waypoint(garage_waypoint))\
                    .and_then(wait(2))\
                    .and_then(turn_to_degrees(0))
            if(not(ConeInternal == 2)):
                # Path C.4 Right
                segment3 = segment3\
                    .and_then(turn_to_degrees(0))\
                    .and_then(wait(2))\
                    .and_then(drive_to_waypoint(waypoint2))\
                    .and_then(wait(2))\
                    .and_then(turn_to_degrees(0))\
                    .and_then(wait(2))\
                    .and_then(drive_backwards_to_waypoint(garage_waypoint))\
                    .and_then(wait(2))\
                    .and_then(turn_to_degrees(0))
        else:
            '''
            This code will build segment 3 with each path, 
            leaving out the square where the cone is located.
            '''
            if(not(ConeInternal == 7)):
                # Path C.1 Left
                segment3 = segment3\
                    .and_then(turn_to_degrees(0))\
                    .and_then(wait(2))\
                    .and_then(drive_to_waypoint(waypoint7))\
                    .and_then(wait(2))\
                    .and_then(turn_to_degrees(0))\
                    .and_then(wait(2))\
                    .and_then(drive_backwards_to_waypoint(garage_waypoint))\
                    .and_then(wait(2))\
                    .and_then(turn_to_degrees(0))
            if(not(ConeInternal == 5)):
                # Path C.2 Left
                segment3 = segment3\
                    .and_then(turn_to_degrees(0))\
                    .and_then(wait(2))\
                    .and_then(drive_to_waypoint(waypoint5))\
                    .and_then(wait(2))\
                    .and_then(turn_to_degrees(0))\
                    .and_then(wait(2))\
                    .and_then(drive_backwards_to_waypoint(garage_waypoint))\
                    .and_then(wait(2))\
                    .and_then(turn_to_degrees(0))
            if(not(ConeInternal == 3)):
                # Path C.3 Left
                segment3 = segment3\
                    .and_then(turn_to_degrees(0))\
                    .and_then(wait(2))\
                    .and_then(drive_to_waypoint(waypoint3))\
                    .and_then(wait(2))\
                    .and_then(turn_to_degrees(0))\
                    .and_then(wait(2))\
                    .and_then(drive_backwards_to_waypoint(garage_waypoint))\
                    .and_then(wait(2))\
                    .and_then(turn_to_degrees(0))
            if(not(ConeInternal == 1)):
                # Path C.4 Left
                segment3 = segment3\
                    .and_then(turn_to_degrees(0))\
                    .and_then(wait(2))\
                    .and_then(drive_to_waypoint(waypoint1))\
                    .and_then(wait(2))\
                    .and_then(turn_to_degrees(0))\
                    .and_then(wait(2))\
                    .and_then(drive_backwards_to_waypoint(garage_waypoint))\
                    .and_then(wait(2))\
                    .and_then(turn_to_degrees(0))


        
         
        # Creating and returning command
        return wait_until(lambda : self.position_updated and self.pivot_position_updated)\
                .and_then(segment1)\
                .and_then(wait(2))\
                .and_then(segment2)\
                .and_then(wait(2))\
                .and_then(segment3)\



    def get_auto_command(self) -> Command:
        """
        Generates and returns the command to start when auto is started

        Returns:
            Command: Command to start when auto is started
        """
        # Waypoint after left turn out
        left_waypoint = Twist()
        left_waypoint.linear.x = -2.5
        left_waypoint.linear.z = 2
        left_waypoint.angular.y = 180
        
        # Waypoint after right turn out
        right_waypoint = Twist()
        right_waypoint.linear.x = 2.5
        right_waypoint.linear.z = 2
        right_waypoint.angular.y = 0
        
        # Factory functions for removing redundancy
        wait = lambda time_seconds : WaitCommand(time_seconds)
        turn_to_degrees = lambda degrees : TurnToDegreesCommand(degrees, self.get_pivot_position, self.drive_pivot)
        drive_to_waypoint = lambda waypoint : DriveToWaypointCommand(waypoint, self.get_position, self.get_pivot_position, self.drive_pivot, self.drive)
        drive_distance = lambda speed, distance : DriveDistanceCommand(speed, distance, self.drive)
        wait_until = lambda condition : WaitUntilCommand(condition)

        # Creating and returning command
        return wait_until(lambda : self.position_updated and self.pivot_position_updated)\
                .and_then(turn_to_degrees(0))\
                .and_then(wait(2))\
                .and_then(drive_to_waypoint(left_waypoint))\
                .and_then(wait(2))\
                .and_then(turn_to_degrees(0))\
                .and_then(wait(2))\
                .and_then(drive_distance(1, 2))\
                .and_then(wait(2))\
                .and_then(turn_to_degrees(-22))\
                .and_then(wait(2))\
                .and_then(drive_distance(1, 3.14 * 1.90475 / 4))\
                .and_then(wait(2))\
                .and_then(drive_distance(-1, 3.14 * 1.90475 / 4))\
                .and_then(wait(2))\
                .and_then(turn_to_degrees(0))\
                .and_then(wait(2))\
                .and_then(drive_distance(-1, 1.5))\
                .and_then(wait(2))\
                .and_then(turn_to_degrees(-18.624))\
                .and_then(wait(2))\
                .and_then(drive_distance(-1, 2.25 * 3.14 / 2))\
                .and_then(wait(2))\
                .and_then(turn_to_degrees(0))\
                .and_then(wait(2))\
                .and_then(drive_distance(-1, 1.5))\
                .and_then(wait(2))\
                .and_then(drive_to_waypoint(right_waypoint))\
                .and_then(wait(2))\
                .and_then(turn_to_degrees(0))\
                .and_then(wait(2))\
                .and_then(drive_distance(1, 2))\
                .and_then(wait(2))\
                .and_then(turn_to_degrees(-22))\
                .and_then(wait(2))\
                .and_then(drive_distance(1, 3.14 * 1.90475 / 4))\
                .and_then(wait(2))\
                .and_then(drive_distance(-1, 3.14 * 1.90475 / 4))\
                .and_then(wait(2))\
                .and_then(turn_to_degrees(0))\
                .and_then(wait(2))\
                .and_then(drive_distance(-1, 1.5))\
                .and_then(wait(2))\
                .and_then(turn_to_degrees(18.624))\
                .and_then(wait(2))\
                .and_then(drive_distance(-1, 2.25 * 3.14 / 2))\
                .and_then(wait(2))\
                .and_then(turn_to_degrees(0))\
                .and_then(wait(2))\
                .and_then(drive_distance(-1, 1.5))
    
    def drive_pivot(self, speed) -> None:
        """
        Drives the pivot left or right or stops the pivot

        Args:
            speed (int): -1 for left, 1 for right, 0 for stop
        """
        msg = Int8()
        msg.data = speed
        self.pivot_publisher.publish(msg)
        
    def update_pivot_position(self, msg: Float32) -> None:
        """
        Updates the pivot position when a new message is heard

        Args:
            msg (Float32): Message containing the new pivot position
        """
        self.pivot_position_updated = True
        self.pivot_position = msg.data
    
    def update_obsticals(self, msg: Polygon) -> None:
        """
        Updates the obstical locations when a new message is heard

        Args:
            msg (Polygon): Message containing the new obstical locations
        """
        self.obsticals_updated = True
        self.obsticals = msg.data
        
    def get_pivot_position(self) -> float:
        """
        Gets the current pivot position

        Returns:
            float: Current pivot position
        """
        return self.pivot_position
    
    def update_position(self, msg: Twist) -> None:
        """
        Updates the position of the vehicle when a new message is heard

        Args:
            msg (Twist): Message containing the new position
        """
        self.position_updated = True
        self.position = msg
        
    def get_position(self) -> float:
        """
        Gets the current position of the vehicle

        Returns:
            float: Current position of the vehicle
        """
        return self.position
        
    def drive(self, speed: float) -> None:
        """
        Drives the vehicle at the given speed

        Args:
            speed (float): Speed to drive at in range [-1..1]
        """
        msg = Twist()
        msg.linear.x = speed
        self.speed_publisher.publish(msg)
        
def main():
    rclpy.init()
    
    node = Auto()
    rclpy.spin(node)
    
    node.destroy_node()
    rclpy.shutdown()
    
if __name__ == '__main__':
    main()
