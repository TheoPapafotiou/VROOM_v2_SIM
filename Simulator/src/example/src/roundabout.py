import time

import numpy as np


class Roundabout:

    def __init__(self, get_perception):
        self.get_perc = get_perception
        self.angle = 0.0
        self.finished = False
        self.case = -1
        self.yaw_margin = 5
        self.angle = 0.0
        self.up_angle_limit = 20
        self.lane_keeping_array = []

    def get_angle(self):
        return self.angle

    def roundabout_procedure(self):

        yaw_init = self.get_perc()['Yaw']
        self.angle = 0.0

        while self.finished is False:

            yaw = self.get_perc()['Yaw']
            diff = abs(abs(yaw) - abs(yaw_init))

            if self.get_perc()['HorLine']['Distance'] < 320 and self.case == -1 \
                                    and -self.yaw_margin < diff < self.yaw_margin:
                
                self.case = 0
            elif 30 - self.yaw_margin < diff < 30 + self.yaw_margin and self.case == 0:
                self.case = 1
                # maybe connect the lines when more lane keeping needed
            elif -self.yaw_margin < diff < self.yaw_margin and self.case == 1:
                self.case = 2
            elif 20 - self.yaw_margin < diff < 20 + self.yaw_margin and self.case == 2:
                self.case = 3
            elif -self.yaw_margin < diff < self.yaw_margin and self.case == 3:
                self.case = 4
                self.finished = True

            if self.case == 0:
                if self.angle >= self.up_angle_limit:
                    continue
                self.angle += 0.25
                print('Gradually increase angle')
            elif self.case == 1:
                self.angle = self.get_perc()['LKangle']
                self.lane_keeping_array.append(self.angle)
                print('Lane keeping boy')
            elif self.case == 2:
                self.angle = -self.up_angle_limit#np.average(self.lane_keeping_array)
                print('Stable angle --> ', self.angle) 
            elif self.case == 3:
                if self.angle >= self.up_angle_limit:
                    continue
                self.angle += 0.25
                print('Gradually increase angle')

            time.sleep(0.05)
