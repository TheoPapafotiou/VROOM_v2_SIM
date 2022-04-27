#!/usr/bin/env python3

import json
import traceback
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
        self.round = Roundabout(self._get_perception_results)
        self.roundabout_running = False

        #for intersection 
        self.yaw_init = 0.0
        self.intersection_type = "None"
        self.intersection_running = False
        self.perception_dict = {}
        # self.intersection= Intersection(self.color_cam.cv_image.shape[1], self.color_cam.cv_image.shape[0],self._get_perception_results)

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

        self.roundThread = Thread(name='RoundaboutNavigation', target=self._round_nav, daemon=True)

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

    def _round_nav(self):

        self.round.roundabout_procedure(type='L')
        self.roundabout_running = False
        print('Roundabout finished!')

    # ===================================== TEST FUNCTION ====================================
    def _test_function(self):
        
        self.speed = 0
        time.sleep(4)
        print('START!')
        self.speed = 0
        self.angle = 0
        self.speed = 10
        counter = 0
        yaw_init = 0
        self.case = -1

        roundabout_detected = True
        roundabout_timer = time.time()
        #instead of Horizontal line for now

        try:
            while self.reset is False:
                
                yaw = round(self.IMU.yaw, 2)

                lane_frame = self.color_cam.cv_image

                self.perception_dict['Yaw'] = yaw 
                self.perception_dict['Camera'] = self.color_cam.cv_image
                self.perception_dict['Speed'] = self.speed
                self.perception_dict['HorLine'] = {
                    'Distance' : float('inf')
                } 
                self.perception_dict['LKangle'], lk_frame1 = self.Lanekeep.lane_keeping_pipeline(lane_frame)

                ### ROUNDABOUT

                if roundabout_detected == True and time.time() - roundabout_timer > 2:
                    
                    self.perception_dict['HorLine']['Distance'] = 300
                    self.roundThread.start()
                    roundabout_detected = False
                    self.roundabout_running = True

                if self.roundabout_running:
                    self.angle = self.round.get_angle()
                else:
                    self.angle = self.perception_dict['LKangle']

                    if self.round.finished:
                        print("Time to die [Roundabout]")
                        self.roundThread.join()
                        self.round.finished = False
                # show_frame = np.concatenate([lane_frame, cv2.cvtColor(lk_frame2, cv2.COLOR_GRAY2RGB)], axis=1)
                
                counter += 1
                cv2.imshow("Preview", lk_frame1)
                cv2.waitKey(1)
                
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    self.reset = True
                
                time.sleep(0.05)

            self.speed = 0.0
        
        except Exception as e:
            print(e)
            traceback.print_exc()


    def warp_image(self, frame, src_points):

        # Destination points for warping
        dst_points = np.float32([[0, 0], [self.width, 0], [0, self.height], [self.width, self.height]])
        
        # Warp frame
        self.warp_matrix = cv2.getPerspectiveTransform(src_points, dst_points)
        warped_frame = cv2.warpPerspective(frame, self.warp_matrix, (self.width, self.height))

        return warped_frame
                 
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
