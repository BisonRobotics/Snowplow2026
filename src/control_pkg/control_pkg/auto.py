import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Int8
from geometry_msgs.msg import Twist, Polygon, Point32, Point, Vector3
from threading import Thread

from geometry_msgs.msg import Twist, Polygon, Point32 

from control_pkg.commands import Runner, Command
from control_pkg.drive_commands import DriveDistanceCommand, DriveToWaypointCommand, DriveBackwardsToWaypointCommand
from control_pkg.wait_commands import WaitCommand, WaitUntilCommand
from control_pkg.turn_command import TurnToDegreesCommand
from control_pkg.path_planning import turn_path,direction,direction_degress,pivot_tuple
from custom_msgs.srv import WaypointServ

import math
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
        self.create_subscription(Vector3, '/location_calculate',self.update_robot_location, 10)
        
        self.create_subscription(Polygon, '/obstacle_locations', self.update_obsticals, 10)
        # Start runner
        self.runner = Runner()
        self.create_timer(0.01, self.runner.run)
        
        # Start auto command. This code was commented out because we are doing this in process path method
        #self.runner.start_command(self.get_auto_command())  
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
            print("Path A Used")
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
                print("Path B Right Used")
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
                print("Path B Left Used")
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
            print("Path D Right Used")
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
            print("Path D Left Used")
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
                print("Path C Right Used")
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
                print("Path C Left Used")   
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
            print("Path X Used - Error Condition")
            segment3 = turn_to_degrees(0).and_then(drive_distance(1, 2)) # PlaceHolder
        elif(ConeInternal%2 == 1):
            '''
            This code will build segment 3 with each path, 
            leaving out the square where the cone is located.
            '''
            if(not(ConeInternal == 8)):
                # Path C.1 Right
                print("Segment 3 Path C.1 Right Used")
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
                print("Segment 3 Path C.2 Right Used")
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
                print("Segment 3 Path C.3 Right Used")
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
                print("Segment 3 Path C.4 Right Used")
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
                print("Segment 3 Path C.1 Left Used")
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
                print("Segment 3 Path C.2 Left Used")
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
                print("Segment 3 Path C.3 Left Used")
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
                print("Segment 3 Path C.4 Left Used")
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
        print("Auto Commands Created")
        return wait_until(lambda : self.position_updated and self.pivot_position_updated)\
                .and_then(segment1)\
                .and_then(wait(2))\
                .and_then(segment2)\
                .and_then(wait(2))\
                .and_then(segment3)\


                 
    # def get_auto_command(self) -> Command:
    #     """
    #     Generates and returns the command to start when auto is started
    #     """
    #     # Variables for Comparator
    #     self.target_angle = 0.0
    #     self.target_x_position = 0.0
    #     self.target_y_position = 0.0

    #     # Creating and returning command
    #     return wait_until(lambda : self.position_updated and self.pivot_position_updated)\
    #             .and_then(turn_to_degrees(0))\
    #             .and_then(wait(2))\
    #             .and_then(drive_to_waypoint(left_waypoint))\
    #             .and_then(wait(2))\
    #             .and_then(turn_to_degrees(0))\
    #             .and_then(wait(2))\
    #             .and_then(drive_distance(1, 2))\
    #             .and_then(wait(2))\
    #             .and_then(turn_to_degrees(-22))\
    #             .and_then(wait(2))\
    #             .and_then(drive_distance(1, 3.14 * 1.90475 / 4))\
    #             .and_then(wait(2))\
    #             .and_then(drive_distance(-1, 3.14 * 1.90475 / 4))\
    #             .and_then(wait(2))\
    #             .and_then(turn_to_degrees(0))\
    #             .and_then(wait(2))\
    #             .and_then(drive_distance(-1, 1.5))\
    #             .and_then(wait(2))\
    #             .and_then(turn_to_degrees(-18.624))\
    #             .and_then(wait(2))\
    #             .and_then(drive_distance(-1, 2.25 * 3.14 / 2))\
    #             .and_then(wait(2))\
    #             .and_then(turn_to_degrees(0))\
    #             .and_then(wait(2))\
    #             .and_then(drive_distance(-1, 1.5))\
    #             .and_then(wait(2))\
    #             .and_then(drive_to_waypoint(right_waypoint))\
    #             .and_then(wait(2))\
    #             .and_then(turn_to_degrees(0))\
    #             .and_then(wait(2))\
    #             .and_then(drive_distance(1, 2))\
    #             .and_then(wait(2))\
    #             .and_then(turn_to_degrees(-22))\
    #             .and_then(wait(2))\
    #             .and_then(drive_distance(1, 3.14 * 1.90475 / 4))\
    #             .and_then(wait(2))\
    #             .and_then(drive_distance(-1, 3.14 * 1.90475 / 4))\
    #             .and_then(wait(2))\
    #             .and_then(turn_to_degrees(0))\
    #             .and_then(wait(2))\
    #             .and_then(drive_distance(-1, 1.5))\
    #             .and_then(wait(2))\
    #             .and_then(turn_to_degrees(18.624))\
    #             .and_then(wait(2))\
    #             .and_then(drive_distance(-1, 2.25 * 3.14 / 2))\
    #             .and_then(wait(2))\
    #             .and_then(turn_to_degrees(0))\
    #             .and_then(wait(2))\
    #             .and_then(drive_distance(-1, 1.5))
    
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
    
    def update_position(self, msg: Twist) -> None: # Change the Twist to what path tracing publishes 
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
   
    def instructions_to_displacement(start_turn,direction_movement_start, distance) -> float:
        """
        This method takes driving instructions
        returns:
            the instructions in the amount of angle 
            needed to change in pivot point and distance to drive forward.
        """
        turn_radius = 2.25 # Robot turn radius.
        if (start_turn < 0 ): # Left turn  
            angle_change = (distance/(turn_radius*2*math.pi()))*360 
            displacement = turn_radius*math.sqrt(1 - math.cos(angle_change)^2+math.sin(angle_change)^2)
        elif(start_turn > 0): # Right turn
            angle_change = -(distance/(turn_radius*2*math.pi()))*360
            displacement = turn_radius*math.sqrt(1 - math.cos(angle_change)^2+math.sin(angle_change)^2)
        else: # Middle case
            angle_change = 0
            displacement = distance * direction_movement_start
        return(angle_change,displacement)
        # Careful when using, apply half of angle change, then displacements, then the other half of angle change - Nolan
    
    def instructions_to_commands(self,start_turn,forwardbackward, distance) -> Command:
        """
        Generates and returns the command to navigate the next section of the path
        forwardbackward variable is either 1 or -1 for forward or backward

        Returns:
            Command: Command to drive next section of the path.
        """
        
        # Factory functions for removing redundancy
        wait = lambda time_seconds : WaitCommand(time_seconds)
        turn_to_degrees = lambda degrees : TurnToDegreesCommand(degrees, self.get_pivot_position, self.drive_pivot)
        drive_distance = lambda speed, distance : DriveDistanceCommand(speed, distance, self.drive)

        # Creating and returning command
        return  turn_to_degrees(18.624*start_turn)\
                .and_then(wait(2))\
                .and_then(drive_distance(forwardbackward,distance))\
                .and_then(wait(2))\
                
    def comparator(self,angle_change,displacement,orientation,robot_x,robot_y) -> None:
        """
        Checks to see if the robots actual position is differen't from our 
        expected position and will call a correction method if there is a large enough difference.
        """
        if(displacement >= 17):
            self.target_angle = orientation
            self.target_x_position = robot_x
            self.target_y_position = robot_y
            return
        else:
            self.target_angle += angle_change/2
            self.target_x_position += math.cos(self.target_angle)*displacement
            self.target_y_position += math.sin(self.target_angle)*displacement
            self.target_angle += angle_change/2
            if((self.target_angle >= orientation + 15) or (self.target_angle <= orientation - 15) or (self.target_x_position >= robot_x + .25) or (self.target_x_position <= robot_x - .25) or (self.target_y_position >= robot_y + .25) or (self.target_y_position <= robot_y - .25)):
                self.corrections(self.target_angle,self.target_x_position,self.target_y_position) # Call Correction method to fix miss alinement.
            
        
    def corrections(self,target_angle_corrections,target_x_corrections,target_y_corrections):
        self.primary_method(target_x_corrections,target_y_corrections)
        # This method was intended to find a new path that would correct distance from where we are to where we want to be.
        pass
    
    """
    This part marks the begginning of the advance path planning portion of auto.
    Some parts were combined as nesciscary (the main functions, the init functions and the imports )
    Note - This part is really messy to look at, bewarned before entering
    """ 
    
    def primary_method(self,target_x,target_y):
        """
        This method will take the target waypoint x and y and then create and test a path to get there
        it will then send this information to process path to run that path.
        """
        # Saving target data for using in case of needing to reroute 
        self.saved_target_X = target_x
        self.saved_target_y = target_y
        # Calculate and save target orientation
        target_orientation = ((direction_degress((self.robot_X,self.robot_y),(target_x,target_y)))*3 - self.orientation)/2
        self.saved_target_orientation = target_orientation    
         
        robot_instructions = self.path_checker(self.instruction_bundler(*turn_path((self.robot_X,self.robot_y),self.orientation,(target_x,target_y),target_orientation)))
        if(robot_instructions != None):
            self.process_path(robot_instructions)
        
        

    def instruction_bundler(self,start_turn, direction_movement_start, distance1, middle_turn, direction_movement_middle, distance2, end_turn, direction_movement_end, distance3):
        """
        Used to convert instructions into bundles.
        """
        # Take all the instructions and put them in tuples
        instructions_part_start = (start_turn,direction_movement_start,distance1)
        instructions_part_middle = (middle_turn, direction_movement_middle, distance2)
        instructions_part_end = (end_turn, direction_movement_end, distance3)
        #Put the bundled instructiosn into another bundle.
        instructions_bundle = (instructions_part_start,instructions_part_middle,instructions_part_end)
        return instructions_bundle
    def instruction_bundler_four(self,start_turn, direction_movement_start, distance1, middle_turn, direction_movement_middle, distance2, end_turn, direction_movement_end, distance3,preturn,predirection_movemnet_start,distance0):
        """
        Used to convert instructions into bundles.
        """
        # Take all the instructions and put them in tuples
        instructions_prior_start = (preturn,predirection_movemnet_start,distance0)
        instructions_part_start = (start_turn,direction_movement_start,distance1)
        instructions_part_middle = (middle_turn, direction_movement_middle, distance2)
        instructions_part_end = (end_turn, direction_movement_end, distance3)
        #Put the bundled instructiosn into another bundle.
        instructions_bundle = (instructions_prior_start,instructions_part_start,instructions_part_middle,instructions_part_end)
        return instructions_bundle
    


    def path_checker(self,instructions_bundle):
        """
        This method will check to see if the robots current path will 
        take it into an object or out of bounds and call the replacement instructino to get new instructions.
        """
        # if there is a instruction before the start one we mark interation down one to show its the prior.
        if(len(instructions_bundle) == 4):
            failed_iteration-1
        else:
            failed_iteration = 0
        current_x = self.robot_X
        current_y = self.robot_y
        current_orientation = self.orientation
        for instructions in instructions_bundle:
            if(instructions[0] == 0): # Start of striaght case
                # find the change in both cordinates
                change_x = math.cos(self.orientation)*instructions[2]*instructions[1]
                change_y = math.sin(self.orientation)*instructions[2]*instructions[1]
                # Calculate the final position of robot
                end_x = change_x + current_x
                end_y = change_y + current_y
                end_orientation = current_orientation 
                # checks for end destination is out of bounds 
                if(abs(end_x) > 7.4 or end_y > 3.4 or (abs(end_x) < 1.9 and end_y < .6)):
                    return self.replace_instructions(failed_iteration,0)
                else:
                    slope = change_y/change_x
                    y_intercept = end_y-slope*end_x
                    for obstacle in self.obstacles:
                        obstacle_distance = (abs(-slope*current_x+current_y-y_intercept))/(math.sqrt(slope**2+1))
                        if(obstacle_distance < .61):
                           close_x = ((current_x + slope*current_y) - slope*y_intercept)/(slope**2 + 1)
                           if((close_x > current_x and close_x < end_x)or (close_x < current_x and close_x > end_x)):
                               return self.replace_instructions(failed_iteration,1)
            # End of straight case
            else: # Start checking for curve cases
                angle_change,displacement = self.instructions_to_displacement(instructions[0],instructions[1],instructions[2])
                end_x = current_x + math.cos(angle_change)*displacement
                end_y = current_y + math.sin(angle_change)*displacement
                circle_center , _ = pivot_tuple((current_x,current_y),(end_x,end_y),current_orientation)
                if((circle_center[0] + 2.25 )> 7.4):
                    if((current_y > circle_center[1] and end_y < circle_center[1] and ((current_orientation*instructions[1])%360< 90 or (current_orientation*instructions[1])%360 >270))or (current_y < circle_center[1] and end_y > circle_center[1] and ((current_orientation*instructions[1])%360> 90 and (current_orientation*instructions[1])%360 <270))):
                        return self.replace_instructions(failed_iteration,0)   
                elif((circle_center[0] - 2.25) < -7.4):
                    if((current_y < circle_center[1] and end_y > circle_center[1] and ((current_orientation*instructions[1])%360< 90 or (current_orientation*instructions[1])%360 >270))or (current_y > circle_center[1] and end_y < circle_center[1] and ((current_orientation*instructions[1])%360> 90 and (current_orientation*instructions[1])%360 <270))):
                        return self.replace_instructions(failed_iteration,0)   
                if(circle_center[1] + 2.25 > 3.4):
                    if((current_x > circle_center[0] and end_x < circle_center[0] and ((current_orientation*instructions[1])%360)< 180)or (current_x < circle_center[0] and end_x > circle_center[0] and ((current_orientation*instructions[1])%360)> 180)): 
                        return self.replace_instructions(failed_iteration,0)    
                if(circle_center[1] - 2.25 < .6):
                    if((current_x < circle_center[0] and end_x > circle_center[0] and ((current_orientation*instructions[1])%360)< 180)or (current_x > circle_center[0] and end_x < circle_center[0] and ((current_orientation*instructions[1])%360)> 180)): # figure out if statement for garage side 
                        if(current_y > .6 and end_y >.6):
                            return self.replace_instructions(failed_iteration,2)
                        elif((abs(end_x) > 1.9 and end_y < .6) or end_y < -2.4):
                            return self.replace_instructions(failed_iteration,2)
                        elif(math.dist(circle_center,(2.5,0)) < 2.85):
                            corner_cliiping = abs((180+current_orientation*instructions[1] - direction_degress((current_x,current_y),(2.5,0)))%360-180) < abs((180-current_orientation*instructions[1] - direction_degress((current_x,current_y),(2.5,0)))%360-180)
                            if((current_y > .6 or end_y > .6) and corner_cliiping):
                                return self.replace_instructions(failed_iteration,2)
                for obstacle in self.obstacles:
                    if(math.dist(circle_center,obstacle) > 1.65 and math.dist(circle_center,obstacle) < 2.85):
                        # finding the nerest point of our path to an object. Then checks if we are going to go through that point.
                        nearest_point =((math.cos(direction(circle_center,obstacle))*2.25),math.sin(direction(circle_center,obstacle))*2.25)
                        object_cliipping = abs((direction_degress((current_x,current_y),(nearest_point[0],nearest_point[1]))- current_orientation+180)%360-180) < abs((direction_degress((end_x,end_y),(nearest_point[0],nearest_point[1]))- current_orientation+180)%360-180)
                        if(object_cliipping):
                            return self.replace_instructions(failed_iteration,1)
                
            # add one to failed iteration to say that istructions was good.
            failed_iteration += 1
            current_x = end_x
            current_y = end_y
            current_orientation = end_orientation 
        
        return instructions_bundle
    


    def replace_instructions(self,failed_iteration,failed_reason):
        """
        This method will replace bad instructions with ones that don't 
        run into things or go out of bounds
        Args:
            which part of the path failed and the reason it failed 
            reason failed 0 - out of bounds, 1 - colides with obstacle, 2 - indicates that we are going out of bounds on side near garage.
        Returns:
            Instruction bundle that doesn't go out of bounds.
        """
        self.attempts += 1 # The attempts variables - interates for each time we have to try something different
        if(self.attempts == 1):
            if(failed_reason == 2): # A Case - Failed out of bounds, near garage, attempting to go backwards
                instructions = (turn_path((self.robot_X,self.robot_y),self.orientation,(self.saved_target_X,self.saved_target_y),self.saved_target_orientation + 180))
                return self.path_checker(self.instruction_bundler(instructions[0],-instructions[1],instructions[2],instructions[3],-instructions[4],instructions[5],instructions[6],-instructions[7],instructions[8]))
            elif(failed_reason == 1): # B Case - Path would hit an object, attempt to change target orientation
                new_orientation = ((direction_degress((self.robot_X,self.robot_y),(self.saved_target_X,self.saved_target_y))) + self.orientation)/2
                return self.path_checker(self.instruction_bundler((turn_path((self.robot_X,self.robot_y),self.orientation,(self.saved_target_X,self.saved_target_y),new_orientation ))))
            else:
                instructions_bundle = self.instruction_bundler_four(*turn_path((self.robot_X,self.robot_y),self.orientation,(self.saved_target_X,self.saved_target_y),self.saved_target_orientation),0,-1,1)
                return self.path_checker(instructions_bundle) # C Case, we attempt to add an step proir to start which moves the robot forward or backards
        elif(self.attempts == 2):
            if(failed_reason == 1): # B Case - Path would hit an object, attempt ot change target orientation with a degree shift
                return self.path_checker(self.instruction_bundler((turn_path((self.robot_X,self.robot_y),self.orientation,(self.saved_target_X,self.saved_target_y),self.saved_target_orientation + 45 ))))
            else:
                instructions_bundle = self.instruction_bundler_four(*turn_path((self.robot_X,self.robot_y),self.orientation,(self.saved_target_X,self.saved_target_y),self.saved_target_orientation),0,1,1)
                return self.path_checker(instructions_bundle) # C Case, we attempt to add an step proir to start which moves the robot forward or backards
        elif(self.attempts == 3):
            if(failed_reason == 1):
                instructions_bundle = self.instruction_bundler_four(*turn_path((self.robot_X,self.robot_y),self.orientation,(self.saved_target_X,self.saved_target_y),self.saved_target_orientation),0,1,.5)
                return self.path_checker(instructions_bundle) # C Case, we attempt to add an step proir to start which moves the robot forward or backards
            else: # do A Case 
                instructions_bundle = self.instruction_bundler(*turn_path((self.robot_X,self.robot_y),self.orientation,(self.saved_target_X,self.saved_target_y),self.saved_target_orientation + 180))
                return self.path_checker(instructions_bundle)
        else:
            instructions_bundle = self.instruction_bundler_four(*turn_path((self.robot_X,self.robot_y),self.orientation,(self.saved_target_X,self.saved_target_y),self.saved_target_orientation),0,-1,.5)
            return (instructions_bundle) # C Case, we move it back a litle bit and send it.
    



        
            

        
def main():
    rclpy.init()
    
    node = Auto()
    rclpy.spin(node)
    
    node.destroy_node()
    rclpy.shutdown()
    
if __name__ == '__main__':
    main()
