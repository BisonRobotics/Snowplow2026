import math
from control_pkg.auto import Auto
from control_pkg.path_planning import turn_path,direction

class Advanced_path_planning(Node):

    def __init__(self):
        super().__init__('advance_path_planning')
        self.obstacles
        self.robot_X = 0.0 # Placeholder until we get actual live data.
        self.robot_y = 0.0
        self.orientation = 0.0

        
    def primary_method(self,target_x,target_y):
        #  Get data from path tracing node
        self.robot_X = 0.0 # Placeholder until we get actual live data.
        self.robot_y = 0.0
        self.orientation = 0.0
        #
        target_orientation = (direction((self.robot_X,self.robot_y),(target_x,target_y)) *(180/math.pi()) + self.orientation)/2

         
        robot_instructions = self.math_magic(turn_path((self.robot_X,self.robot_y),self.orientation,(target_x,target_y),target_orientation))
        Auto

    def math_magic(self,start_turn, direction_movement_start, distance1, middle_turn, direction_movement_middle, distance2, end_turn, direction_movement_end, distance3):
        # Take all the instructions and put them in tuples
        instructions_part_start = (start_turn,direction_movement_start,distance1)
        instructions_part_middle = (middle_turn, direction_movement_middle, distance2)
        instructions_part_end = (end_turn, direction_movement_end, distance3)
        #Put the bundled instructiosn into another bundle.
        instructions_bundle = (instructions_part_start,instructions_part_middle,instructions_part_end)
        failed_iteration = 0
        for instructions in instructions_bundle:
            if(instructions[0] == 0):
                # find the change in both cordinates
                change_x = math.cos(self.orientation)*instructions[2]*instructions[1]
                change_y = math.sin(self.orientation)*instructions[2]*instructions[1]
                # Calculate the final position of robot
                end_x = change_x + self.robot_X
                end_y = change_y + self.robot_y
                # checks for end destination is out of bounds 
                if(abs(end_x) > 7.8 or end_y > 3.8 or (abs(end_x) < 2 and end_y < 0)):
                    return self.replace_instructions(instructions_bundle,failed_iteration,0)
                else:
                    for obstacle in self.obstacles:
                        obstacle[0] 
                        # do some stuff. WIP
                        


                
            # add one to failed iteration to say that istructions was good.
            failed_iteration += 1



    def replaced_instructions(self,instructions_bundle,failed_iteration,failed_reason): 
        """
        This method will replace bad instructions with ones that don't 
        run into things or go out of bounds
        Args:
            boundle of instructions, which part of the bundle failed,the reason it failed 
            reason failed 0 - out of bounds, 1 - colides with obstacle
        Returns:
            Instruction bundle that doesn't go out of bounds.
        """




        
            
