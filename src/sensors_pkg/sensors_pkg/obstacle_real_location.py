import math

def location(robot_x,robot_y,robot_orientation,range,azimuth):
    obstacle_rel_x = range * math.cos(azimuth)
    obstacle_rel_y = range * math.sin(azimuth)

    obstacle_abs_direction = robot_orientation - azimuth
    obstacle_distance = math.sqrt(obstacle_rel_x ** 2 + obstacle_rel_y ** 2)
    obstacle_x = robot_x + obstacle_distance* math.cos(obstacle_abs_direction * (math.pi/180))
    obstacle_y = robot_y + obstacle_distance * math.sin(obstacle_abs_direction * (math.pi/180))

    #For testing
    # print(f"obstacle abs direction: {obstacle_abs_direction}")
    # print(f"obstacle distance: {obstacle_distance}")
    # print(f"obstacle x: {obstacle_x}")
    # print(f"obstacle y: {obstacle_y}")

    return obstacle_x, obstacle_y # treating lidar output as left is negative and right is positive


# if __name__ == "__main__":
#     location(0,0,90,2,2,-45)