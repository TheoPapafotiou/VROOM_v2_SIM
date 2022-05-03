#!/usr/bin/env python3

import json
import traceback
import cv2
import time
import numpy as np
import math

from ranging_sensors    import RangeHandler
from std_msgs.msg       import String
from threading          import Thread
from camera             import CameraHandler
from gps                import GPSHandler
from imu                import IMUHandler
from LaneKeepingFinal   import LaneKeeping
from intersection       import Intersection
from roundabout         import Roundabout
import rospy

class AutonomousControlProcess():
    # ===================================== INIT==========================================
    def __init__(self):
        """Run on the PC. It is used to test the scripts created for each challenge.        
        """        
        self.speed = 0.0
        self.angle = 0.0 
        self.last_angle = 0.0
        self.reset = False

        rospy.init_node('EXAMPLEnode', anonymous=False)     
        self.publisher = rospy.Publisher('/automobile/command', String, queue_size=1)

        # cv_image param
        self.depth_cam = CameraHandler("depth")
        self.color_cam = CameraHandler("color")
       
        # range param
        self.rayFront = RangeHandler("front")
        self.rayRight = RangeHandler("right")
        self.rayFrontLeft = RangeHandler("front_left")
        self.rayFrontRight = RangeHandler("front_right")

        # pos param
        self.GPS = GPSHandler()

        # [roll, pitch, yaw] params
        self.IMU = IMUHandler()

        #for lane keeping
        # self.lane_frame = np.zeros((self.color_cam.cv_image.shape[1], self.color_cam.cv_image.shape[0], 3))
        self.width = 640
        self.height = 480        
        self.Lanekeep = LaneKeeping(self.color_cam.cv_image.shape[1], self.color_cam.cv_image.shape[0], version=1) 

        #for intersection 
        self.yaw_init = 0.0
        self.intersection_type = "None"
        self.intersection_running = False
        self.perception_dict = {}

    # ===================================== RUN ==========================================
    def run(self):
        """Apply initializing methods and start the threads. 
        """
        self.mainThread = Thread(name='TestFunction', target=self._test_function, daemon=True)
        self.mainThread.start()

        self.speedThread = Thread(name='SpeedFunction', target=self._command_speed, daemon=True)
        self.speedThread.start()

        self.angleThread = Thread(name='AngleFunction', target=self._command_angle, daemon=True)
        self.angleThread.start()

        rospy.spin()
        print("Threads closed")

    # ===================================== For Intersection ======================================
    def absolute_yaw_init(self, yaw_init):
        if -20 < yaw_init < 20:
            diff = 0.0 - yaw_init
            return 0.0, diff
        elif 70 < yaw_init < 110:
            diff = 90.0 - yaw_init
            return 90.0, diff
        elif -70 > yaw_init > -110:
            diff = -90.0 - yaw_init
            return -90.0, diff
        else:
            diff = 180.0 - yaw_init
            return 180, diff

    def _get_perception_results(self):

        return self.perception_dict

    # ===================================== TEST FUNCTION ====================================
    def _test_function(self):
        
        self.speed = 0
        lane_frame = self.color_cam.cv_image
        time.sleep(4)
        print('START!')
        self.angle = 0
        self.speed = 10
        #instead of Horizontal line for now

        dt = 0.1
        counter = 0
        last_time = time.time()

        try:
            while self.reset is False:
                
                loop_time = time.time()
                yaw = round(self.IMU.yaw, 2)

                lane_frame = self.color_cam.cv_image

                self.perception_dict['Yaw'] = yaw 
                self.perception_dict['Camera'] = lane_frame
                self.perception_dict['Speed'] = self.speed
                self.perception_dict['HorLine'] = {
                    'Distance' : float('inf')
                } 
                self.perception_dict['LKangle'], lk_frame1 = self.Lanekeep.lane_keeping_pipeline(lane_frame)

                self.angle = self.perception_dict['LKangle']
                # show_frame = np.concatenate([lane_frame, cv2.cvtColor(lk_frame2, cv2.COLOR_GRAY2RGB)], axis=1)

                if counter == 1:
                    x, y = self.calculate_position(self.GPS.pos[0], self.GPS.pos[1], yaw, self.speed/100, time.time() - last_time)
                elif counter > 1:
                    x, y = self.calculate_position(x, y, yaw, self.speed/100, time.time() - last_time)
                else:
                    x = self.GPS.pos[0]
                    y = self.GPS.pos[1]
                last_time = time.time()

                print(yaw)
                print(x, y, " VS ", self.GPS.pos[0], self.GPS.pos[1], " (init)")
                last_time = time.time()

                cv2.imshow("Preview", lk_frame1)
                cv2.waitKey(1)
                
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    self.reset = True
                
                counter += 1
                time.sleep(dt)

            self.speed = 0.0
        
        except Exception as e:
            print(e)
            traceback.print_exc()

    def calculate_position(self, x, y, yaw, speed, dt):
        
        yaw = math.radians(yaw)

        ux = speed*math.cos(yaw)
        uy = speed*math.sin(yaw)

        x = x + ux*dt        
        y = y - uy*dt
        return x, y
                 
    # ===================================== SEND COMMAND =================================
    def _command_speed(self):
        """
        Parameters
        ----------
        inP : Pipe
            Input pipe. 
        """

        while True:

            time.sleep(0.05)
            data = {}
            data['action'] = '1'
            data['speed'] = float(self.speed/100.0)        

            if data is not None:
        
                command = json.dumps(data)
                # print(command)
                self.publisher.publish(command)  


    def _command_angle(self):
        """Transmite the command to the remotecontrol receiver. 
        
        Parameters
        ----------
        inP : Pipe
            Input pipe. 
        """
        while True:

            time.sleep(0.05)
            data = {}
            if self.angle > 23.0:
                self.angle = 23.0
            elif self.angle < -23.0:
                self.angle = - 23.0

            data['action'] = '2'
            data['steerAngle'] = float(self.angle)

            if data is not None:
        
                command = json.dumps(data)
                # print(command)
                self.publisher.publish(command)  
            
if __name__ == '__main__':
    try:
        nod = AutonomousControlProcess()
        nod.run()
    except rospy.ROSInterruptException:
        pass
