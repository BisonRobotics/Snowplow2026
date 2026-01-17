import math
from dataclasses import dataclass
from enum import Enum
from typing import Literal

turn_radius = 2.25
radians = (math.pi / 180)

def direction(start_point, end_point):
    change = (end_point[0] - start_point[0], end_point[1] - start_point[1])
    
    if change[0] == 0:
        return math.pi / 2 if change[1] > 0 else 3* math.pi / 2
    if change [1] == 0:
        return math.pi if change[0] < 0 else 0
    return math.pi + math.atan(change[1] / change[0]) if change[0] < 0 else math.atan(change[1] / change[0])

def direction_degress(start_point, end_point):
    return direction(start_point,end_point) * (180/math.pi())

# finds center point of a circle for the robot to drive around and the direction it should take
def pivot_tuple(point_a, point_b, direction):
    #defining the circle

    # find first tuple
    rad_a = math.radians(direction + 90)
    tuple_a = (point_b[0] + turn_radius * math.cos(rad_a), point_b[1] + turn_radius * math.sin(rad_a))
    
    # find second tuple
    rad_b = math.radians(direction - 90)
    tuple_b = (point_b[0] + turn_radius * math.cos(rad_b), point_b[1] + turn_radius * math.sin(rad_b))
    
    # determine which tuple should be used and which direction
    if math.dist(point_a, tuple_a) <= math.dist(point_a, tuple_b):
        return tuple_a, -1 #turn left one of these could possibly be inverted need to test
    else:
        return tuple_b, 1 # turn right

# determines the direction the robot should move in
def final_direction(tuple, point, direction):
    rad = math.radians(direction)
    tuple_a = (tuple[0] + turn_radius * math.cos(rad), tuple[1] + turn_radius * math.sin(rad))
    tuple_b = (tuple[0] - turn_radius * math.cos(rad), tuple[1] - turn_radius * math.sin(rad))

    if math.dist(point, tuple_a) <= math.dist(point, tuple_b):
        return 1 #forward
    else:
        return -1 #backward

@dataclass
class Segment:
    turn_angle: float
    direction: Literal[-1, 1]
    distance: float

@dataclass
class Path:
    segment1: Segment
    segment2: Segment
    segment3: Segment

class TurnDirection(Enum):
    LEFT = 0
    RIGHT = 1

def turn_path(start_point: tuple[float, float], start_direction: float, end_point: tuple[float, float], end_direction: float) -> Path:
    #defining the end circle
    end_tuplea = (math.cos(math.radians(end_direction + 90)), math.sin(math.radians(end_direction + 90)))
    end_tuplea = (end_point[0] + turn_radius * end_tuplea[0], end_point[1] + turn_radius * end_tuplea[1])
    
    end_tupleb = (math.cos(math.radians(end_direction - 90)), math.sin(math.radians(end_direction - 90)))
    end_tupleb = (end_point[0] + turn_radius * end_tupleb[0], end_point[1] + turn_radius * end_tupleb[1])

    if math.dist(start_point, end_tuplea) <= math.dist(start_point, end_tupleb):
        end_tuple = end_tuplea
        end_turn = TurnDirection.LEFT
    else:
        end_tuple = end_tupleb
        end_turn = TurnDirection.RIGHT

    #define the start circle
    start_tuplea = (math.cos(math.radians(start_direction + 90)), math.sin(math.radians(start_direction + 90)))
    start_tuplea = (start_point[0] + turn_radius * start_tuplea[0], start_point[1] + turn_radius * start_tuplea[1])

    start_tupleb = (math.cos(math.radians(start_direction - 90)), math.sin(math.radians(start_direction-90)))
    start_tupleb = (start_point[0] + turn_radius * start_tupleb[0], start_point[1] + turn_radius * start_tupleb[1])
    

    if math.dist(start_tuplea, end_tuple) <= math.dist(start_tupleb, end_tuple):
        start_tuple = start_tuplea
        start_turn = TurnDirection.LEFT
    else:
        start_tuple = start_tupleb
        start_turn = TurnDirection.RIGHT

    if start_turn == end_turn:            
        tan_point_starta = (start_tuple[0] + turn_radius * (math.cos(direction(start_tuple, end_tuple) -math.pi / 2)), start_tuple[1] + turn_radius * (math.sin(direction(start_tuple, end_tuple) -math.pi / 2))) #counterclockwise
        tan_point_startb = (start_tuple[0] + turn_radius * (math.cos(direction(start_tuple, end_tuple) + math.pi / 2)) , start_tuple[1] + turn_radius * (math.sin(direction(start_tuple, end_tuple) + math.pi / 2))) #clockwise
            
        tan_point_endb = (end_tuple[0] + turn_radius * (math.cos(direction(end_tuple, start_tuple) -math.pi / 2)), end_tuple[1] + turn_radius * (math.sin(direction(end_tuple, start_tuple) -math.pi / 2))) #counterclockwise
        tan_point_enda = (end_tuple[0] + turn_radius * (math.cos(direction(end_tuple, start_tuple) + math.pi / 2)), end_tuple[1] + turn_radius * (math.sin(direction(end_tuple, start_tuple) + math.pi / 2))) #clockwise

        if abs(direction(start_tuple, tan_point_starta) - direction(start_tuple, start_point) + math.radians(start_direction) - direction(tan_point_starta, tan_point_enda) )% (2* math.pi) <= 0.0001:
            tan_point_start = tan_point_starta
            tan_point_end = tan_point_enda
        else:
            tan_point_start = tan_point_startb
            tan_point_end = tan_point_endb
    else:
        hypotenuse = math.dist(start_tuple, end_tuple)
        short = 2 * turn_radius
        theta = math.atan2(end_tuple[1] - start_tuple[1], end_tuple[0] - start_tuple[0]) + math.asin(short / hypotenuse) -math.pi / 2
        tan_point_starta = (start_tuple[0] + turn_radius * math.cos(theta), start_tuple[1] + turn_radius * math.sin(theta))
        tan_point_enda = (end_tuple[0] + turn_radius * math.cos(theta + math.pi), end_tuple[1] + turn_radius * math.sin(theta + math.pi))
        if abs((direction(start_tuple, tan_point_starta) - direction(start_tuple, start_point) + math.radians(start_direction) - direction(tan_point_starta, tan_point_enda))) % (2* math.pi) <= 0.0001:
            tan_point_start = tan_point_starta
            tan_point_end = tan_point_enda
        else: 
                
            if end_tuple[0]-start_tuple[0] == 0:
                inv_slope = 0
                b2 = tan_point_starta[1] - inv_slope * tan_point_starta[0]
                b3 = tan_point_enda[1] - inv_slope * tan_point_enda[0] 
                tan_point_startb = 2 * start_tuple[0] - tan_point_starta[0], b2
                tan_point_endb = 2 * start_tuple[0] - tan_point_enda[0], b3
            else: 
                slope = (end_tuple[1] - start_tuple[1]) / (end_tuple[0] - start_tuple[0])
                b1 = start_tuple[1] - slope * start_tuple[0]
                
                if slope != 0:
                    inv_slope = -1 / slope
                    b2 = tan_point_starta[1] - inv_slope * tan_point_starta[0]
                    b3 = tan_point_enda[1] - inv_slope * tan_point_enda[0] 
                    intersectionstart = ((b2 - b1) / (slope - inv_slope), 0)
                    intersectionend = ((b3 - b1) / (slope - inv_slope), 0)
                    intersectionstart= (intersectionstart[0], intersectionstart[0] * slope + b1)
                    intersectionend = (intersectionend[0], intersectionend[0] * slope + b1) 
                    tan_point_startb = (2 * intersectionstart[0]- tan_point_starta[0], 2 * intersectionstart[1] - tan_point_starta[1])
                    tan_point_endb = (2 * intersectionend[0] - tan_point_enda[0], 2 * intersectionend[1]- tan_point_enda[1])
                else:
                    tan_point_startb = (tan_point_starta[0], start_tuple[1]*2-tan_point_starta[1])
                    tan_point_endb = (tan_point_enda[0], end_tuple[1]*2-tan_point_enda[1])

            tan_point_start = tan_point_startb
            tan_point_end = tan_point_endb
  
    
    placeholder_tuple1 = (start_tuple[0]+turn_radius*math.cos(math.radians(start_direction)), start_tuple[1]+turn_radius*math.sin(math.radians(start_direction)))
    placeholder_tuple2 = (start_tuple[0]-turn_radius*math.cos(math.radians(start_direction)), start_tuple[1]-turn_radius*math.sin(math.radians(start_direction)))

    if math.dist(tan_point_start, placeholder_tuple1) <= math.dist(tan_point_start, placeholder_tuple2):
        direction_movement_start = 1 #forward
    
    else:
        direction_movement_start = -1 #backward

    if math.dist(tan_point_end, (end_tuple[0] - turn_radius * math.cos(math.radians(end_direction)), end_tuple[1] - turn_radius * math.sin(math.radians(end_direction)))) <= math.dist(tan_point_end, (end_tuple[0] + turn_radius * math.cos(math.radians(end_direction)), end_tuple[1] + turn_radius * math.sin(math.radians(end_direction)))):
        direction_movement_end = 1 #forward
    
    else:
        direction_movement_end = -1 #backward
    
    # --- UNTESTED CODE TO REDUCE THE STUFF BEFORE ---
    # Find the initial direction the robot needs to move in
    # direction_movement_start = final_direction(start_tuple, tan_point_start, start_direction)
    # direction_movement_end = final_direction(end_tuple, tan_point_end, end_direction)
    distance1, distance3 = 0, 0
    
    match start_turn:
        case TurnDirection.LEFT:
            distance1 = round(turn_radius * (round((direction(start_tuple, tan_point_start) - direction(start_tuple, start_point)), 3) % (2 * math.pi)), 3)
            if distance1 > turn_radius * math.pi:
                distance1 = 2 * turn_radius * math.pi - distance1
                direction_movement_start *= -1

        case TurnDirection.RIGHT:
            distance1 = round(turn_radius * (round((direction(start_tuple, start_point) - direction(start_tuple, tan_point_start)), 3) % (2 * math.pi)), 3)
            if distance1 > turn_radius * math.pi:
                distance1 = 2 * turn_radius * math.pi - distance1
                direction_movement_start *= -1
    
    distance2 = round(math.dist(tan_point_start, tan_point_end), 3)
    
    match end_turn:
        case TurnDirection.LEFT:
            distance3 = round(turn_radius * (round((direction(end_tuple, end_point) - direction(end_tuple, tan_point_end)), 3) % (2 * math.pi)), 3)
            if distance3 > turn_radius * math.pi:
                distance3 = round(2 * turn_radius * math.pi - distance3 , 3)
                direction_movement_end *= -1

        case TurnDirection.RIGHT:
            distance3 = round(turn_radius * (round((direction(end_tuple, tan_point_end) - direction(end_tuple, end_point)), 3) % (2 * math.pi)), 3)
            if distance3 > turn_radius * math.pi:
                distance3 = round(2 * turn_radius * math.pi - distance3 , 3)
                direction_movement_end *= -1
    
    segment1_turn_angle = (-1 if start_turn == TurnDirection.LEFT else 1)
    segment3_turn_angle = (-1 if end_turn == TurnDirection.LEFT else 1)

    return Path(
        segment1=Segment(segment1_turn_angle, direction_movement_start, distance1), 
        segment2=Segment(0, 1, distance2), 
        segment3=Segment(segment3_turn_angle, direction_movement_end, distance3)
    )

if __name__ == "__main__":
    print(turn_path((0.14, -2.5), 90, (-2.5, 2), 180))
