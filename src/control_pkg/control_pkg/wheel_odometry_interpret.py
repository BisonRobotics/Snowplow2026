import math

#angle with reference to start direction maybe radians maybe degrees
# orientation = 90

#location relative to start in m
#
#+y = foreward, -y = backwards at orientation 90
#+x = right, -x = left at orientation 90
# location_x = 0
# location_y = 0

#minutes between signals
# time_frame = 1/600

#wheel circumference in meters
wc = 0.43 * math.pi

#wheel track in meters
wt = .915

def update_odometry(left:float, right:float, time_frame=1/600):
    #use subscription to sensor_picker to grap accurate position and orientation

    #put in to update right and left here
    

    #math to determine variables
    distance_right = right * time_frame * wc
    distance_left = left * time_frame * wc

    change_orientation = (distance_right - distance_left) / wt

    distance_avg = (distance_left + distance_right) / 2.0


    if distance_left != distance_right:
        turning_radius = distance_left / change_orientation + wt / 2.0
        direct = 2 * turning_radius * turning_radius * (1 - math.cos(change_orientation * math.pi / 180)) / (change_orientation / 360 * turning_radius)
    else:
        direct = 0.0

    return change_orientation, distance_avg, direct


    # If going straight
    # if distance_left == 0:
    #     #update just location
    #     location_y = location_y + distance_avg * math.sin(orientation * math.pi / 180)
    #     location_x = location_x + distance_avg * math.cos(orientation * math.pi / 180)
    # else:
    #     #direct is the ratio of direct distance to the distance along a circle
    #     direct = 2 * turning_radius * turning_radius * (1 - math.cos(change_orientation * math.pi / 180)) / (change_orientation/360 * turning_radius)

    #     #updates location x,y
    #     location_y = location_y + math.sin((orientation + .5 * change_orientation) * math.pi / 180) * distance_avg * direct
    #     location_x = location_x + math.cos((orientation + .5 * change_orientation) * math.pi / 180) * distance_avg * direct
    #     #updates orientation
    #     orientation = (orientation + change_orientation) % 360

    #publish location and orientation

