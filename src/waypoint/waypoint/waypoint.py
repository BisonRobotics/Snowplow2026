import math

class Waypoint():
    def __init__(self, x: float, y: float, threshold: float = 1.0):
        self.x = x
        self.y = y
        self.threshold = threshold

    def within_range(self, x: float, y: float) -> bool:
        """Finds if the position is within the waypoints threshold distance"""
        return math.sqrt((self.x - x) ** 2 + (self.y - y) ** 2) <= self.threshold
    

def Cone(x: float, y: float, threshold=0.645) -> Waypoint:
    return Waypoint(x, y, threshold)

def Snow(x: float, y: float, threshold=0.12) -> Waypoint:
    return Waypoint(x, y, threshold)
