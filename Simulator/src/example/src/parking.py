from statistics import mean
import time
from tracemalloc import start
import numpy as np
import math

class Parking:

    """
    This class is used for the implementation of the parking maneuver of the vehicle.
    
    Both vertical and horizontal parking maneuvers have been implemented.
    """
    
    def __init__(self, get_perception):

        self.get_perc = get_perception
        self.min_speed = 10
        self.speed = self.min_speed
        self.angle = 0.0

        self.roll_avg = 10
        self.car_pos = 2
        self.finished = False

        ### LENGTHS ###
        self.hor_spot_length = 70 #cm

        ### COUNTERS ###
        self.repetitions = 0
        self.counter_wait = 0
        self.counter_correct = 0
        self.counter_forward = 0

        ### Horizontal Parking Params ###
        self.theta_hor = 23
        self.omega_hor = 45

        ### Vertical Parking Params ###
        self.theta_ver = 23
        self.phi_ver = 21
        self.omega_ver = 28
        self.kappa_ver = 26#28
        self.correct_angle = -10
        
        ### Yaw margins ### 
        self.margin_L = 6
        self.margin_M = 3
        self.margin_S = 1

        ### Parking parts ###
        self.conditions = 9
        self.part = [False for i in range(self.conditions)]
        
        ### Flags ###
        self.correction = False
        self.prepared = False
        self.forward = False
        
    def check_start(self, type):

        self.finished = False

        if type == "H":
            checked = False
            rayFront_init = self.get_perc()['RayFront']
            ranges = []
            steps = 0
            start_time = time.time()
            time_threshold = self.hor_spot_length / self.min_speed

            while checked is False:
                self.angle = self.get_perc()['LKAngle']
                steps += 1
                ranges.append(self.get_perc()['RayFront'])
                total_avg = mean(ranges)
                roll_avg = mean(ranges[-self.roll_N:])

                if self.car_pos == -1:
                    if total_avg < (rayFront_init - 100) and (time.time() - start_time) < time_threshold:
                        self.car_pos = 1
                    elif (time.time() - start_time) >= time_threshold and self.car_pos != 1 and roll_avg < 400:
                        self.car_pos = 2
                    elif (time.time() - start_time) >= 2*time_threshold and roll_avg > 400:
                        self.car_pos = 0

                if self.car_pos == 1:
                    ...
                elif self.car_pos == 2:
                    checked = True
        else:
            checked = False
            while checked is False:
                ...
                checked = True

        return checked

    def get_speed_angle(self):
        return self.speed, self.angle

    def parking_horizontal(self, yaw_init):

        ### Main loop for maneuver ###

        while self.finished is False:
        
            yaw = self.get_perc()['Yaw']
            horizontal_line = self.get_perc()["HorLine"]
            print(yaw)
            print(horizontal_line)     

            if self.car_pos == 1:
                if (horizontal_line is True or self.counter_correct > 10) and self.part[3] is True:
                    self.correction = True
            else:
                if (horizontal_line is True or self.counter_correct > 10) and self.part[2] is True:
                    self.correction = True

            if self.counter_wait >= 100:
                self.prepared = True

            diff = abs(abs(yaw) - abs(yaw_init))
            

            if self.car_pos == 2 or self.car_pos == 0:

                ### Check the part of the parking procedure
                if (-self.margin_M <= diff <= self.margin_M and \
                    self.part[0] is False and self.part[1] is False and self.part[2] is False and \
                    self.part[3] is False and self.part[4] is False and self.part[5] is False and \
                    self.finished is False): #Turn right and backwards
                    
                    for i in range(self.conditions):
                        self.part[i] = False
                    self.part[0] = True

                elif (self.omega_hor - self.margin_M <= diff <=  self.omega_hor + self.margin_M and \
                    self.part[0] is True): #Turn left and backwards
                    
                    for i in range(self.conditions):
                        self.part[i] = False
                    self.part[1] = True

                elif (-self.margin_M <= diff <= self.margin_M and \
                    self.part[1] is True): #Correct parking
                    
                    for i in range(self.conditions):
                        self.part[i] = False
                    self.part[2] = True

                elif (self.part[2] is True and self.correction is True): #Wait a little
                    
                    for i in range(self.conditions):
                        self.part[i] = False
                    self.part[3] = True
                    self.correction = False
                    self.counter_correct = 0

                elif (self.part[3] is True and self.prepared is True): #Turn left and forward 
                    
                    for i in range(self.conditions):
                        self.part[i] = False
                    self.part[4] = True
                    self.prepared = False
                    self.counter_wait = 0
                    
                elif (self.omega_hor - self.margin_M <= diff <=  self.omega_hor + self.margin_M and \
                    self.part[4] is True): #Turn right until straight
                    
                    for i in range(self.conditions):
                        self.part[i] = False
                    self.part[5] = True

                elif (-self.margin_M <= diff <= self.margin_M and \
                    self.part[5] is True):
                    
                    for i in range(self.conditions):
                        self.part[i] = False
                    self.finished = True

                ### Calculate the speed and angle
                if self.part[0] is True:
                    print("Part 1: Turn right and backwards")
                    self.angle = self.theta_hor
                    self.speed = -self.min_speed

                elif self.part[1] is True:
                    print("Part 2: Turn left and backwards")
                    self.angle = -self.theta_hor
                    self.speed = -self.min_speed

                elif self.part[2] is True:
                    print("Part 3: Correct")
                    self.angle = 0.0
                    self.speed = self.min_speed
                    self.counter_correct += 1

                elif self.part[3] is True:
                    print("Part 4: Wait a little")
                    self.angle = 0.0
                    self.speed = 0.0
                    self.counter_wait += 1

                elif self.part[4] is True:
                    print("Part 5: Turn left and forward")
                    self.angle = -self.theta_hor
                    self.speed = self.min_speed
                
                elif self.part[5] is True:
                    print("Part 6: Turn right until yaw_init")
                    self.angle = self.theta_hor
                    self.speed = self.min_speed
                else:
                    print("No part")

            elif self.car_pos == 1:

                ### Check the part of the parking procedure
                if (-self.margin_M <= diff <= self.margin_M and \
                    self.part[0] is False and self.part[1] is False and self.part[2] is False and \
                    self.part[3] is False and self.part[4] is False and self.part[5] is False and \
                    self.part[6] is False and self.finished is False): #Turn right and forward
                    
                    for i in range(self.conditions):
                        self.part[i] = False
                    self.part[0] = True

                elif (self.omega_hor - self.margin_M <= diff <=  self.omega_hor + self.margin_M and \
                    self.part[0] is True): #Turn left and forward
                    
                    for i in range(self.conditions):
                        self.part[i] = False
                    self.part[1] = True

                elif (self.omega_hor/4 - self.margin_M <= diff <=  self.omega_hor/4 + self.margin_M and \
                    self.part[1] is True): #Turn right and backward
                    
                    for i in range(self.conditions):
                        self.part[i] = False
                    self.part[2] = True

                elif (-self.margin_M <= diff <= self.margin_M and \
                    self.part[2] is True): #Correct parking
                    
                    for i in range(self.conditions):
                        self.part[i] = False
                    self.part[3] = True

                elif (self.part[3] is True and self.correction is True): #Wait a little
                    
                    for i in range(self.conditions):
                        self.part[i] = False
                    self.part[4] = True
                    self.correction = False
                    self.counter_correct = 0

                elif (self.part[4] is True and self.prepared is True): #Turn left and forward 
                    
                    for i in range(self.conditions):
                        self.part[i] = False
                    self.part[5] = True
                    self.prepared = False
                    self.counter_wait = 0
                    
                elif (self.omega_hor - self.margin_M <= diff <=  self.omega_hor + self.margin_M and \
                    self.part[5] is True): #Turn right until straight
                    
                    for i in range(self.conditions):
                        self.part[i] = False
                    self.part[6] = True

                elif (-self.margin_M <= diff <= self.margin_M and \
                    self.part[6] is True):
                    
                    for i in range(self.conditions):
                        self.part[i] = False
                    self.finished = True

                ### Calculate the speed and angle
                if self.part[0] is True:
                    print("Part 1: Turn right and forward")
                    self.angle = self.theta_hor
                    self.speed = self.min_speed

                elif self.part[1] is True:
                    print("Part 2: Turn left and forward")
                    self.angle = -self.theta_hor
                    self.speed = self.min_speed

                elif self.part[2] is True:
                    print("Part 3: Turn right and backward")
                    self.angle = self.theta_hor
                    self.speed = -self.min_speed

                elif self.part[3] is True:
                    print("Part 4: Correct")
                    self.angle = 0.0
                    self.speed = -self.min_speed
                    self.counter_correct += 1

                elif self.part[4] is True:
                    print("Part 5: Wait a little")
                    self.angle = 0.0
                    self.speed = 0.0
                    self.counter_wait += 1

                elif self.part[5] is True:
                    print("Part 6: Turn left and forward")
                    self.angle = -self.theta_hor
                    self.speed = self.min_speed
                
                elif self.part[6] is True:
                    print("Part 7: Turn right until yaw_init")
                    self.angle = self.theta_hor
                    self.speed = self.min_speed
                else:
                    print("No part")

            time.sleep(0.05)
                        
        return 0

    def parking_vertical(self, yaw_init):

        while self.finished is False:

            yaw = self.get_perc()['Yaw']
            horizontal_line = self.get_perc()["HorLine"]
            print(yaw)
            print(horizontal_line)
            
            if self.car_pos == 1:
                if (horizontal_line is True or self.counter_correct == 3) and self.part[3] is True:
                    self.correction = True
            else:
                if (horizontal_line is True or self.counter_correct == 3) and self.part[2] is True:
                    self.correction = True

            if self.counter_wait == 100:
                self.prepared = True
                
            diff = abs(abs(yaw) - abs(yaw_init))

            if self.car_pos == 2 or self.car_pos == 0:
            
                ### Check the part of the parking procedure
                if (-self.margin_M <= diff <= self.margin_M and \
                    self.part[0] is False and self.part[1] is False and self.part[2] is False and \
                    self.part[3] is False and self.part[4] is False and self.part[5] is False and \
                    self.finished is False): #Turn left and forward
                    
                    for i in range(self.conditions):
                        self.part[i] = False
                    self.part[0] = True

                elif (self.kappa_ver - self.margin_M <= diff <= self.kappa_ver + self.margin_M and \
                    self.part[0] is True): #Turn right and backwards
                    
                    for i in range(self.conditions):
                        self.part[i] = False
                    self.part[1] = True

                elif (90 - self.margin_M <= diff <= 90 + self.margin_M and \
                    self.part[1] is True): #Correct parking
                    
                    for i in range(self.conditions):
                        self.part[i] = False
                    self.part[2] = True

                elif (self.part[2] is True and self.correction is True): #Wait a little
                    
                    for i in range(self.conditions):
                        self.part[i] = False
                    self.part[3] = True
                    self.correction = False
                    self.counter_correct = 0

                elif (self.part[3] is True and self.prepared is True): #Turn right and forward 
                    
                    for i in range(self.conditions):
                        self.part[i] = False
                    self.part[4] = True
                    self.prepared = False
                    self.counter_wait = 0
                    
                elif (self.omega_ver - self.margin_M <= diff <= self.omega_ver + self.margin_M and \
                    self.part[4] is True): #Turn more right until
                    
                    for i in range(self.conditions):
                        self.part[i] = False
                    self.part[5] = True

                elif (-self.margin_M <= diff <= self.margin_M and \
                    self.part[5] is True):
                    
                    for i in range(self.conditions):
                        self.part[i] = False
                    self.finished = True

                ### Calculate the speed and angle
                if self.part[0] is True:
                    print("Part 1: Turn left and forward")
                    self.angle = -self.theta_ver
                    self.speed = self.min_speed

                elif self.part[1] is True:
                    print("Part 2: Turn right and backwards")
                    self.angle = self.phi_ver
                    self.speed = -self.min_speed

                elif self.part[2] is True:
                    print("Part 3: Correct")
                    self.angle = self.correct_angle
                    self.speed = -self.min_speed
                    self.counter_correct += 1

                elif self.part[3] is True:
                    print("Part 4: Wait a little")
                    self.angle = 0
                    self.speed = 0
                    self.counter_wait += 1
                
                elif self.part[4] is True:
                    print("Part 5: Turn right and forward")
                    self.angle = self.theta_ver
                    self.speed = self.min_speed
                
                elif self.part[5] is True:
                    print("Part 6: Turn finally right")
                    self.angle = self.theta_ver
                    self.speed = self.min_speed
                else:
                    print("No part")
                        
            elif self.car_pos == 1:

                ### Check the part of the parking procedure
                if (-self.margin_M <= diff <= self.margin_M and \
                    self.part[0] is False and self.part[1] is False and self.part[2] is False and \
                    self.part[3] is False and self.part[4] is False and self.part[5] is False and \
                    self.part[6] is False and self.part[7] is False and self.finished is False): #Turn right and forward
                    
                    for i in range(self.conditions):
                        self.part[i] = False
                    self.part[0] = True

                elif (self.kappa_ver - self.margin_M <= diff <= self.kappa_ver + self.margin_M and \
                    self.part[0] is True): #Turn left and backwards
                    
                    for i in range(self.conditions):
                        self.part[i] = False
                    self.part[1] = True

                elif (1.5*self.kappa_ver - self.margin_M <= diff <= 1.5*self.kappa_ver + self.margin_M and \
                    self.part[1] is True): #Turn right and forward again
                    
                    for i in range(self.conditions):
                        self.part[i] = False
                    self.part[2] = True

                elif (90 - self.margin_M <= diff <= 90 + self.margin_M and \
                    self.part[2] is True): #Correct parking
                    
                    for i in range(self.conditions):
                        self.part[i] = False
                    self.part[3] = True

                elif (self.part[3] is True and self.correction is True): #Wait a little
                    
                    for i in range(self.conditions):
                        self.part[i] = False
                    self.part[4] = True
                    self.correction = False
                    self.counter_correct = 0

                elif (self.part[4] is True and self.prepared is True): #Turn right and backwards
                    
                    for i in range(self.conditions):
                        self.part[i] = False
                    self.part[5] = True
                    self.prepared = False
                    self.counter_wait = 0

                elif (1.5*self.kappa_ver - self.margin_M <= diff <= 1.5*self.kappa_ver + self.margin_M and \
                    self.part[5] is True): #Turn left and forward
                    
                    for i in range(self.conditions):
                        self.part[i] = False
                    self.part[6] = True

                elif (self.kappa_ver - self.margin_M <= diff <= self.kappa_ver + self.margin_M and \
                    self.part[6] is True): #Turn right and backwards
                    
                    for i in range(self.conditions):
                        self.part[i] = False
                    self.part[7] = True

                elif (-self.margin_M <= diff <= self.margin_M and \
                    self.part[7] is True):
                    
                    for i in range(self.conditions):
                        self.part[i] = False
                    self.finished = True

                ### Calculate the speed and angle
                if self.part[0] is True:
                    print("Part 1: Turn right and forward")
                    self.angle = self.theta_ver
                    self.speed = self.min_speed

                elif self.part[1] is True:
                    print("Part 2: Turn left and backward")
                    self.angle = -self.theta_ver
                    self.speed = -self.min_speed
                
                elif self.part[2] is True:
                    print("Part 3: Turn right and forward")
                    self.angle = self.theta_ver
                    self.speed = self.min_speed
                
                elif self.part[3] is True:
                    print("Part 4: Correct")
                    self.angle = -self.correct_angle
                    self.speed = self.min_speed
                    self.counter_correct += 1

                elif self.part[4] is True:
                    print("Part 5: Wait a little")
                    self.angle = 0
                    self.speed = 0
                    self.counter_wait += 1

                elif self.part[5] is True:
                    print("Part 6: Turn right and backwards")
                    self.angle = self.theta_ver
                    self.speed = -self.min_speed

                elif self.part[6] is True:
                    print("Part 7: Turn left and forward")
                    self.angle = -self.theta_ver
                    self.speed = self.min_speed
                
                elif self.part[7] is True:
                    print("Part 8: Turn finally right and backwards")
                    self.angle = self.theta_ver
                    self.speed = -self.min_speed
                
                else:
                    print("No part")


            time.sleep(0.05)

        return 0