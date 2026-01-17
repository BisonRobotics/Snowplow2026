
def find_zone(x: float, y: float) -> int:
    # Zones are numbered Externally towards the inside with, .
    # Exterial Zones are numbered -1 to -7 clockwise starting from the garage. 
    # Returns 0 if Cone is outside of all zones.

    # Checking for Cone in one of the 14 Squares
    # Top Left Squares
    if -5 < x <= -4 and 1.5 < y < 2.5:
        return 1 # Square 1 
    elif -4 < x <= -3 and 1.5 < y < 2.5:
        return 3 # Square 3
    elif -3 < x <= -2 and 1.5 < y < 2.5:
        return 5 # Square 5
    elif -2 < x <= -1 and 1.5 < y < 2.5: 
        return 7 # Square 7
    elif -1 < x <=  0 and 1.5 < y < 2.5: 
        return 9 # Square 9
    # Top Right Squares
    elif  0 < x <=  1 and 1.5 < y < 2.5:
        return 10 # Square 10 
    elif  1 < x <=  2 and 1.5 < y < 2.5:
        return 8 # Square 8
    elif  2 < x <=  3 and 1.5 < y < 2.5:
        return 6 # Square 6
    elif  3 < x <=  4 and 1.5 < y < 2.5: 
        return 4 # Square 4
    elif  4 < x <= 5 and 1.5 < y < 2.5: 
        return 2 # Square 2
    # Bottom Four Squares 
    elif -1 < x <=  0 and 0.75 < y < 1.5:
        return 11 # Square 11
    elif  0 < x <=  1 and 0.75 < y < 1.5:
        return 12 # Square 12
    elif  -1 < x <= 0 and 0.00 < y < 0.75:
        return 13 # Square 13  
    elif  0 < x <= 1 and 0.00 < y < 0.75:
        return 14 # Square 14
    # Checking for Cone in outer Zones.
    elif -5 < x <= -1 and 0 < y < 1.5:
        return -1 # Zone Left of Garage.
    elif -8 < x <= -5 and 0 < y < 4:
        return -2 # Left Manuvering Zone
    elif -5 < x <= -1 and 2.5 < y < 4:
        return -3 # Far Left Zone.
    elif -1 < x <= 1 and 2.5 < y < 4:
        return -4 # Evil Zone thats directly ahead of starting,
    elif 1 < x <= 5 and 2.5 < y < 4:
        return -5 # Far Right Zone
    elif 5 < x <= 8 and 0 < y < 4:
        return -6 # Far Right Manuevering Zone
    elif 1 < x <= 5 and 0 < y < 1.5: 
        return -7 # Zone Right of Garage
    else:
        return 0 # Outside of playfield.
