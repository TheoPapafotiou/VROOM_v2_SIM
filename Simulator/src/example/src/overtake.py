import time

class Overtake:

    """
    This class is used for the implementation of the overtake maneuver of the vehicle.
    """
    def __init__(self, get_perception):

        self.get_perc = get_perception
        self.angle = 0.0
        self.dotted = False

        self.distance_threshold = 0.8

        self.laneKeepingFlag = False

        self.angle = 0

        self.distance_threshold_front = 0.6 # m
        self.distance_threshold_right = 0.4 # m
        self.yaw_margin = 3

        self.part = [False, False, False, False, False]
        self.vehicle_passed = True

        self.threshold = 16

        self.step_sub = 2
        self.yaw_diff = 0

        self.finished = False

    def check_dotted_line(self, graph, source, target):
        self.dotted = graph[source][target]["dotted"]

    def react_to_vehicle(self,  graph, source, target):
        
        self.check_dotted_line(graph, source, target)

        overtake_allowed = False

        distance_from_vehicle = self.get_perc()['RayFront']

        if distance_from_vehicle <= self.distance_threshold and self.dotted is True:
            overtake_allowed = True
            print("Distance = ", distance_from_vehicle)

        print("Overtake allowed = ", overtake_allowed)
        return overtake_allowed

    def get_angle(self):
        return self.angle

    def maneuver(self, yaw_init):
        
        while self.finished is False:

            yaw = self.get_perc()['Yaw']
            distance_from_vehicle_right = self.get_perc()['RayRight']
            lane_keeping_angle = self.get_perc()['LKAngle']

            self.yaw_diff = abs(yaw-yaw_init)

            print("Yaw_init = ", yaw_init, "\nYaw = ", yaw)
            print("diff = ", self.yaw_diff)

            if distance_from_vehicle_right > self.distance_threshold_right:
                self.vehicle_passed = True

            if yaw_init - self.yaw_margin <= yaw <= yaw_init + self.yaw_margin and not any(self.part[0:-1]): # Turn left
                
                for i in range(1, len(self.part)):
                    self.part[i] = False
                self.part[0] = True

            elif self.yaw_diff >= self.threshold and self.part[0] is True: # Turn right to correct
                for i in range(0, len(self.part)):
                    self.part[i] = False
                self.part[1] = True

            elif int(self.yaw_diff) in range (0,6) and self.part[1] is True: # Do Lane keeping
                for i in range(0, len(self.part)):
                    self.part[i] = False
                self.part[2] = True

            elif self.part[2] is True and self.vehicle_passed is True: # Turn right 
                for i in range(0, len(self.part)):
                    self.part[i] = False
                self.part[3] = True

            elif self.yaw_diff >= self.threshold and self.part[3] is True: #Turn left 
                for i in range(0, len(self.part)):
                    self.part[i] = False
                self.part[4] = True

            elif int(self.yaw_diff) in range (0,6) and self.part[4] is True: # Overtake is done. Lane keeping
                for i in range(0, len(self.part)):
                    self.part[i] = False
                self.laneKeepingFlag = True
                self.finished = True
                print("Overtake done!")

            ### Calculate the speed and angle
            if self.part[0] is True:
                print("===================Part 0==================")
                self.angle -= self.step_sub

            elif self.part[1] is True:
                print("===================Part 1==================")
                self.angle += 2*self.step_sub

            elif self.part[2] is True:
                print("===================Part 2==================")
                self.angle = lane_keeping_angle

            elif self.part[3] is True:
                print("===================Part 3==================")
                self.angle += self.step_sub

            elif self.part[4] is True:
                print("===================Part 4==================")
                self.angle -= 2*self.step_sub

            else:
                print("No part!")