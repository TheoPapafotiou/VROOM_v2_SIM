#!/usr/bin/env python3

import json
import cv2
import time
import numpy as np

from ranging_sensors    import RangeHandler
from std_msgs.msg       import String
from threading          import Thread
from camera             import CameraHandler
from gps                import GPSHandler
from imu                import IMUHandler
from LaneKeepingFinal   import LaneKeeping
from intersection       import Intersection
from intersection2      import Intersection2
import rospy

class AutonomousControlProcess():
    # ===================================== INIT==========================================
    def __init__(self):
        """Run on the PC. It is used to test the scripts created for each challenge.        
        """        
        self.speed = 0.0
        self.angle = 0.0 
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
        self.Lanekeep = LaneKeeping(self.color_cam.cv_image.shape[1], self.color_cam.cv_image.shape[0], version=2) 
        
        #for intersection 
        self.yaw_init = 0.0
        self.intersection_type = "None"
        self.intersection_running = False
        self.perception_dict = {}
        # self.intersection= Intersection(self.color_cam.cv_image.shape[1], self.color_cam.cv_image.shape[0],self._get_perception_results)
        self.intersection2= Intersection2(self.color_cam.cv_image.shape[1], self.color_cam.cv_image.shape[0],self._get_perception_results)

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

        self.intersectionThread = Thread(name='IntersectionFunction', target=self._run_intersection, daemon=True)

        rospy.spin()
        print("Threads closed")

    # ===================================== For Intersection ======================================
    def absolute_yaw_init(self, yaw_init):
        if -20 < yaw_init < 20:
            return 0.0
        elif 70 < yaw_init < 110:
            return 90.0
        elif -70 > yaw_init > -110:
            return -90.0
        else:
            return 180

    def _get_perception_results(self):

        return self.perception_dict


    def _run_intersection(self):

        if self.intersection_type == "R":
            yaw_init = self.absolute_yaw_init(self.IMU.yaw)
            self.intersection2.small_right_turn(yaw_init)

        # elif self.intersection_type == "L":
        #     self.yaw_init = self.absolute_yaw_init(self.IMU.yaw)
        #     self.intersection.intersection_left(self.color_cam.cv_image)
            
        # elif self.intersection_type == "S":
        #     self.yaw_init = self.absolute_yaw_init(self.IMU.yaw)
        #     self.intersection2.straight()

        print("Intersection finished!")
        self.intersection_running = False
        # self.speed=0

    # ===================================== TEST FUNCTION ====================================
    def _test_function(self):
        
        time.sleep(1)
        self.speed = 15
        self.angle = 0
        count=0

        try:
            while self.reset is False:
                print('NEW FRAME:')
                
                self.perception_dict['Yaw'] = self.IMU.yaw # MUST BE [-180, 180]
                self.perception_dict['Camera'] = self.color_cam.cv_image
                # print('IMU yaw',self.IMU.yaw)

                if self.intersection_running is False and count==0:  
                    self.intersection_type = "R"
                    self.intersectionThread.start()
                    self.intersection_running = True
                    self.start_yaw = self.perception_dict['Yaw']
                    self.cam_input = self.perception_dict['Camera']
                    count=1

                if self.intersection_running:
                    self.lane_frame=self.intersection2.cam_frame

                else:
                    # self.angle = self.Lanekeep.lane_keeping_pipeline(self.color_cam.cv_image)
                    self.lane_frame = self.color_cam.cv_image

                # cv2.imshow("Preview", self.lane_frame) 
                # cv2.waitKey(1)
                self.angle = self.Lanekeep.lane_keeping_pipeline(self.lane_frame)
                
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    self.reset = True
                
                time.sleep(0.1)

            self.speed = 0.0
        
        except Exception as e:
            print(e)
                 
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