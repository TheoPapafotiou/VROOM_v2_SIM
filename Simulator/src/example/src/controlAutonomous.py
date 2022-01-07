#!/usr/bin/env python3

import json
import cv2
import time

from ranging_sensors    import RangeHandler
from std_msgs.msg       import String
from threading          import Thread
from camera             import CameraHandler
from gps                import GPSHandler
from imu                import IMUHandler
from Center_Of_Roundbout    import CenterOfRoundabout
from LaneKeepingReloaded    import LaneKeepingReloaded

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
        self.lane_cam = CameraHandler("lane")

        # range param
        self.rayFront = RangeHandler("front")
        self.rayBack = RangeHandler("back")
        self.rayRight = RangeHandler("right")
        self.rayLeft = RangeHandler("left")
        self.rayFrontLeft = RangeHandler("front_left")
        self.rayFrontRight = RangeHandler("front_right")

        # pos param
        self.GPS = GPSHandler()

        # [roll, pitch, yaw] params
        self.IMU = IMUHandler()

        self.Round = CenterOfRoundabout(640, 480)
        self.Lane = LaneKeepingReloaded(640, 480)

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

    # ===================================== TEST FUNCTION ====================================
    def _test_function(self):

        self.speed = 10

        counter = 0

        try:
            imageLane = self.color_cam.cv_image
            imageRound = self.color_cam.cv_image
            starting_yaw = 0.0
            while self.reset is False:
                counter += 1

                cv2.imshow("Preview", imageRound) 
                #print(self.GPS.pos)
                #print(self.IMU.yaw)

                if counter == 1:
                    starting_yaw = self.IMU.yaw

                imageRound = self.Round.centerOfRoundabout(self.color_cam.cv_image, self.IMU.yaw, starting_yaw)
                if imageRound is not False:
                    self.angle, imageLane = self.Lane.lane_keeping_pipeline(imageRound)            
                else:
                    self.angle, imageRound = self.Lane.lane_keeping_pipeline(self.color_cam.cv_image)

                if cv2.waitKey(1) & 0xFF == ord('q'):
                    self.reset = True

                time.sleep(0.01)

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
                print(command)
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
                print(command)
                self.publisher.publish(command)  
            
if __name__ == '__main__':
    try:
        nod = AutonomousControlProcess()
        nod.run()
    except rospy.ROSInterruptException:
        pass