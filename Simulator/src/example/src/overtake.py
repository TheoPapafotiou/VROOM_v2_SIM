import time

class Overtake:

    """
    This class is used for the implementation of the overtake maneuver of the vehicle.
    """
    def __init__(self, get_perception):

        self.get_perc = get_perception
        self.speed = 0.0
        self.angle = 0.0
        
    def get_speed_angle(self):
        return self.speed, self.angle

    def maneuver(self):

        rayFront = self.get_perc()['RayFront']