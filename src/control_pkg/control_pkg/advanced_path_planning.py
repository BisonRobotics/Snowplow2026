import rclpy
from rclpy.node import Node
from threading import Thread

import math
from control_pkg.auto import Auto
from control_pkg.path_planning import turn_path,direction,direction_degress,pivot_tuple

from geometry_msgs.msg import Polygon, Point32, Point
from custom_msgs.srv import WaypointServ

class AdvancedPathPlanning(Node):

    def __init__(self):
        super().__init__('advance_path_planning')
        self.obstacles = None
        self.robot_X = 0.0 # Placeholder until we get actual live data.
        self.robot_y = 0.0
        self.orientation = 0.0
        self.auto = Auto()
        self.attempts = 0.0

        self.waypoint_client = self.create_client(WaypointServ, 'waypoint')
        while not self.waypoint_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.waypoint_request = WaypointServ.Request()

    def send_request(self):
        self.waypoint_request.point.x = self.robot_X
        self.waypoint_request.point.y = self.robot_y

        return self.waypoint_client.call(self.waypoint_request)

        
    def primary_method(self,target_x,target_y):
        #  Get data from path tracing node
        self.robot_X = 0.0 # Placeholder until we get actual live data.
        self.robot_y = 0.0
        self.orientation = 0.0
        #
        target_orientation = ((direction_degress((self.robot_X,self.robot_y),(target_x,target_y))) + self.orientation)/2

         
        robot_instructions = self.math_magic(turn_path((self.robot_X,self.robot_y),self.orientation,(target_x,target_y),target_orientation))
        if(robot_instructions != None):
            self.auto.process_path(robot_instructions)
        


        

    def math_magic(self,start_turn, direction_movement_start, distance1, middle_turn, direction_movement_middle, distance2, end_turn, direction_movement_end, distance3):
        # Take all the instructions and put them in tuples
        instructions_part_start = (start_turn,direction_movement_start,distance1)
        instructions_part_middle = (middle_turn, direction_movement_middle, distance2)
        instructions_part_end = (end_turn, direction_movement_end, distance3)
        #Put the bundled instructiosn into another bundle.
        instructions_bundle = (instructions_part_start,instructions_part_middle,instructions_part_end)
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
                    return self.replace_instructions(instructions_bundle,failed_iteration,0)
                else:
                    slope = change_y/change_x
                    y_intercept = end_y-slope*end_x
                    for obstacle in self.obstacles:
                        obstacle_distance = (abs(-slope*current_x+current_y-y_intercept))/(math.sqrt(slope**2+1))
                        if(obstacle_distance < .61):
                           close_x = ((current_x + slope*current_y) - slope*y_intercept)/(slope**2 + 1)
                           if((close_x > current_x and close_x < end_x)or (close_x < current_x and close_x > end_x)):
                               return self.replace_instructions(instructions_bundle,failed_iteration,1)
            # End of straight case
            else: # Start checking for curve cases
                angle_change,displacement = Auto.instructions_to_displacement(instructions[0],instructions[1],instructions[2])
                end_x = current_x + math.cos(angle_change)*displacement
                end_y = current_y + math.sin(angle_change)*displacement
                circle_center , garbage_variable = pivot_tuple((current_x,current_y),(end_x,end_y),current_orientation) # garbage_variable is trash, do not touch
                if((circle_center[0] + 2.25 )> 7.4):
                    if((current_y > circle_center[1] and end_y < circle_center[1] and ((current_orientation*instructions[1])%360< 90 or (current_orientation*instructions[1])%360 >270))or (current_y < circle_center[1] and end_y > circle_center[1] and ((current_orientation*instructions[1])%360> 90 and (current_orientation*instructions[1])%360 <270))):
                        return self.replace_instructions(instructions_bundle,failed_iteration,0)   
                elif((circle_center[0] - 2.25) < -7.4):
                    if((current_y < circle_center[1] and end_y > circle_center[1] and ((current_orientation*instructions[1])%360< 90 or (current_orientation*instructions[1])%360 >270))or (current_y > circle_center[1] and end_y < circle_center[1] and ((current_orientation*instructions[1])%360> 90 and (current_orientation*instructions[1])%360 <270))):
                        return self.replace_instructions(instructions_bundle,failed_iteration,0)   
                if(circle_center[1] + 2.25 > 3.4):
                    if((current_x > circle_center[0] and end_x < circle_center[0] and ((current_orientation*instructions[1])%360)< 180)or (current_x < circle_center[0] and end_x > circle_center[0] and ((current_orientation*instructions[1])%360)> 180)): 
                        return self.replace_instructions(instructions_bundle,failed_iteration,0)    
                if(circle_center[1] + 2.25 > 3.4):
                    if((current_x < circle_center[0] and end_x > circle_center[0] and ((current_orientation*instructions[1])%360)< 180)or (current_x > circle_center[0] and end_x < circle_center[0] and ((current_orientation*instructions[1])%360)> 180)): # figure out if statement for garage side 
                        if(current_y > .6 and end_y >.6):
                            return self.replace_instructions(instructions_bundle,failed_iteration,0)
                        elif((abs(end_x) > 1.9 and end_y < .6) or end_y < -2.4):
                            return self.replace_instructions(instructions_bundle,failed_iteration,0)
                        elif(math.dist(circle_center,(2.5,0)) < 2.85):
                            corner_cliiping = abs((180+current_orientation*instructions[1] - direction_degress((current_x,current_y),(2.5,0)))%360-180) < abs((180-current_orientation*instructions[1] - direction_degress((current_x,current_y),(2.5,0)))%360-180)
                            if((current_y > .6 or end_y > .6) and corner_cliiping):
                                return self.replace_instructions(instructions_bundle,failed_iteration,0)
                for obstacle in self.obstacles:
                    if(math.dist(circle_center,obstacle) > 1.65 and math.dist(circle_center,obstacle) < 2.85):
                        # finding the nerest point of our path to an object. Then checks if we are going to go through that point.
                        nearest_point =((math.cos(direction(circle_center,obstacle))*2.25),math.sin(direction(circle_center,obstacle))*2.25)
                        object_cliipping = abs((direction_degress((current_x,current_y),(nearest_point[0],nearest_point[1]))- current_orientation+180)%360-180) < abs((direction_degress((end_x,end_y),(nearest_point[0],nearest_point[1]))- current_orientation+180)%360-180)
                        if(object_cliipping):
                            return self.replace_instructions(instructions_bundle,failed_iteration,1)
                
            # add one to failed iteration to say that istructions was good.
            failed_iteration += 1
            current_x = end_x
            current_y = end_y
            current_orientation = end_orientation 
        
        return instructions_bundle



    def replace_instructions(self,instructions_bundle,failed_iteration,failed_reason):
        
        """
        This method will replace bad instructions with ones that don't 
        run into things or go out of bounds
        Args:
            boundle of instructions, which part of the bundle failed,the reason it failed 
            reason failed 0 - out of bounds, 1 - colides with obstacle
        Returns:
            Instruction bundle that doesn't go out of bounds.
        """
        self.attempts += 1

          
        self.attempts = 0
        return instructions_bundle




def main():
    rclpy.init()

    advanced_path_client = AdvancedPathPlanning()

    spin_thread = Thread(target=rclpy.spin, args=(advanced_path_client,))
    spin_thread.start()

    # Call this in your method to run (switch advanced_path_client with self)
    response = advanced_path_client.send_request()
    advanced_path_client.get_logger().info(f'Waypoint: {response.point}')

    advanced_path_client.destroy_node()
    rclpy.shutdown()            
