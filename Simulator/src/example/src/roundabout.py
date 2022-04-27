import math
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
        self.up_angle_limit = 23
        self.lane_keeping_array = []

        self.R = 1.6#1.4 # m
        self.dt = 0.1

        self.decrease_rate = 1.5
        self.increase_rate = 1.5

    def get_angle(self):
        return self.angle

    def roundabout_procedure(self, type):

        yaw_init = self.get_perc()['Yaw']
        self.angle = 0.0

        while self.finished is False:

            yaw = self.get_perc()['Yaw']
            loop_time = time.time()
            diff = abs(abs(yaw) - abs(yaw_init))

            if type == 'S':
                if self.get_perc()['HorLine']['Distance'] < 320 and self.case == -1 \
                                        and -self.yaw_margin < diff < self.yaw_margin:
                    self.case = 0
                elif 35 - self.yaw_margin < diff < 35 + self.yaw_margin and self.case == 0:
                    self.case = 1
                elif -self.yaw_margin < diff < self.yaw_margin and self.case == 1:
                    self.case = 2
                elif 35 - self.yaw_margin < diff < 35 + self.yaw_margin and self.case == 2:
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
                    if self.angle <= -self.up_angle_limit:
                        continue
                    self.angle -= self.decrease_rate
                    print('Gradually decrease angle')
                elif self.case == 2:
                    self.stable_omega(yaw, yaw_old, self.dt)
                    print('Keep stable omega')
                elif self.case == 3:
                    if self.angle >= self.up_angle_limit:
                        continue
                    self.angle += self.increase_rate
                    print('Gradually increase angle')

            elif type == 'R':
                if self.get_perc()['HorLine']['Distance'] < 320 and self.case == -1 \
                                        and -self.yaw_margin < diff < self.yaw_margin:
                    
                    self.case = 0
                elif 30 - self.yaw_margin < diff < 30 + self.yaw_margin and self.case == 0:
                    self.case = 1
                    # maybe connect the lines when more lane keeping needed
                elif 32 - self.yaw_margin < diff < 32 + self.yaw_margin and self.case == 1:
                    self.case = 2
                elif 90 - self.yaw_margin < diff < 90 + self.yaw_margin and self.case == 2:
                    self.case = 3
                    self.finished = True

                if self.case == 0:
                    if self.angle >= self.up_angle_limit:
                        continue
                    self.angle += 0.25
                    print('Gradually increase angle')
                elif self.case == 1:
                    self.angle = self.get_perc()['LKangle']
                    print('Lane keeping boy')
                elif self.case == 2:
                    if self.angle >= self.up_angle_limit:
                        continue
                    self.angle += 0.25
                    print('Gradually increase angle')

            elif type == 'L':
                if self.get_perc()['HorLine']['Distance'] < 320 and self.case == -1 \
                                        and -self.yaw_margin < diff < self.yaw_margin:
                    
                    self.case = 0
                elif 35 - self.yaw_margin < diff < 35 + self.yaw_margin and self.case == 0:
                    self.case = 1
                elif -self.yaw_margin < diff < self.yaw_margin and self.case == 1:
                    self.case = 2
                elif 90 - self.yaw_margin < diff < 90 + self.yaw_margin and self.case == 2:
                    self.case = 3
                elif 125 - self.yaw_margin < diff < 125 + self.yaw_margin and self.case == 3:
                    self.case = 4
                elif 90 - self.yaw_margin < diff < 90 + self.yaw_margin and self.case == 4:
                    self.case = 5
                    self.finished = True

                if self.case == 0:
                    if self.angle >= self.up_angle_limit:
                        continue
                    self.angle += 0.25
                    print('Gradually increase angle')
                elif self.case == 1:
                    if self.angle <= -self.up_angle_limit:
                        continue
                    self.angle -= self.decrease_rate
                    print('Gradually decrease angle')
                elif self.case == 2 or self.case == 3:
                    self.stable_omega(yaw, yaw_old, self.dt)
                    print('Keep stable omega')
                elif self.case == 4:
                    if self.angle >= self.up_angle_limit:
                        continue
                    self.angle += self.increase_rate
                    print('Gradually increase angle')

            yaw_old = yaw
            time.sleep(self.dt - (time.time() - loop_time))

    def stable_omega(self, yaw_cur, yaw_old, dt):
        omega_ct = (self.get_perc()['Speed']/100) * self.R

        dtheta = abs(np.deg2rad(yaw_cur - yaw_old))
        omega_cur = dtheta/dt
        flag = 0
        interval = 1

        if omega_cur > omega_ct:
            if yaw_cur >= yaw_old:
                flag = interval
            else:
                flag = -interval
        else:
            if yaw_cur >= yaw_old:
                flag = -interval
            else:
                flag = interval

        self.angle += flag
        if abs(self.angle) >= abs(self.up_angle_limit):
            if self.angle > 0:
                self.angle = self.up_angle_limit
            else:
                self.angle = -self.up_angle_limit
        print(self.angle)


