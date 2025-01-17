import rclpy
import math
from rclpy.node import Node
from std_msgs.msg import Float32, Int8
from geometry_msgs.msg import Twist

from control_pkg.commands import Runner, Command
from control_pkg.drive_commands import DriveDistanceCommand, DriveToWaypointCommand
from control_pkg.wait_commands import WaitCommand, WaitUntilCommand
from control_pkg.turn_command import TurnToDegreesCommand

class Auto(Node):
    def __init__(self):
        super().__init__('auto')
        
        # Set up ROS stuff
        self.pivot_position = None
        self.position = None
        
        self.pivot_position_updated = False
        self.position_updated = False
        
        self.pivot_publisher = self.create_publisher(Int8, '/vehicle/pivot', 10)
        self.speed_publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        
        self.create_subscription(Twist, '/apriltag', self.update_position, 10)
        self.create_subscription(Float32, '/sensor/pivot', self.update_pivot_position, 10)
        
        # Start runner
        self.runner = Runner()
        self.create_timer(0.01, self.runner.run)
        
        # Start auto command. This code was commented out because we are doing this in process path method
        #self.runner.start_command(self.get_auto_command())  

        # Variables for Comparator
        self.target_angle = 0.0
        self.target_x_position = 0.0
        self.target_y_position = 0.0
    
    def process_path(self,instructions) -> None:
        # check subscription to get current posistion 
        # plug current position into the comparator
        self.comparator(0,20,0,0,0) # reseting comparator, last three 0's should be robots actual position values
        for set in instructions:
            self.runner.start_command(self.instructions_to_commands(set[0],set[1],set[2]))
            # check subscription to get current posistion 
            # plug current position into the comparator
            self.comparator(self.instructions_to_displacement())
        return()
    

        
    # def get_auto_command(self) -> Command:
    #     """
    #     Generates and returns the command to start when auto is started

    #     Returns:
    #         Command: Command to start when auto is started
    #     """
    #     # Waypoint after left turn out
    #     left_waypoint = Twist()
    #     left_waypoint.linear.x = -2.5
    #     left_waypoint.linear.z = 2
    #     left_waypoint.angular.y = 180
        
    #     # Waypoint after right turn out
    #     right_waypoint = Twist()
    #     right_waypoint.linear.x = 2.5
    #     right_waypoint.linear.z = 2
    #     right_waypoint.angular.y = 0
        
    #     # Factory functions for removing redundancy
    #     wait = lambda time_seconds : WaitCommand(time_seconds)
    #     turn_to_degrees = lambda degrees : TurnToDegreesCommand(degrees, self.get_pivot_position, self.drive_pivot)
    #     drive_to_waypoint = lambda waypoint : DriveToWaypointCommand(waypoint, self.get_position, self.get_pivot_position, self.drive_pivot, self.drive)
    #     drive_distance = lambda speed, distance : DriveDistanceCommand(speed, distance, self.drive)
    #     wait_until = lambda condition : WaitUntilCommand(condition)

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
    #             .and_then(drive_distance(-1, 3.14 * 1.90471 / 4))\
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

    # Written by Nolan + Garret (next three)    
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
            return 
                

        
            

        
def main():
    rclpy.init()
    
    node = Auto()
    rclpy.spin(node)
    
    node.destroy_node()
    rclpy.shutdown()
    
if __name__ == '__main__':
    main()
