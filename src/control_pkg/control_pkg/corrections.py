import math

def angle_to_displacement(imu_angle: float, string_pot_angle: float, target_orientation:float) -> float:
    '''
    *Only use this method when turning. If used when going forward it will return zero.
    Args:
        imu converted angle, string pot anlgle, and the final targer angle.
    Returns:
        Distance to mave foward to reach target orienation.
    '''
    wheel_base:float = 1 # place holder.
    turn_radius:float =  wheel_base/math.sin(string_pot_angle*math.pi/180)
    robot_orienation = imu_angle + string_pot_angle/2
    return abs((robot_orienation-target_orientation)/180*turn_radius*math.pi())



