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
from pathlib import Path

from detectHorizontal   import DetectHorizontal
from parking            import Parking
from overtake           import Overtake

import rospy

class AutonomousControlProcess():
    # ===================================== INIT==========================================
    def __init__(self):
        """Run on the PC. It is used to test the scripts created for each challenge.        
        """        
        self.speed = 0.0
        self.angle = 0.0 
        self.reset = False
        self.perception_dict = {}

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

        # --- WRITE BELOW ---
        self.det = DetectHorizontal(mask_filename="src/example/src/default_mask_simulation2.json")
        
        self.park = Parking(self._get_perception_results)
        self.yaw_init_park = 0.0
        self.parking_running = False
        self.parking_type = "None"

        self.over = Overtake(self._get_perception_results)
        self.yaw_init_over = 0.0
        self.overtake_running = False
        # --- WRITE ABOVE ---

    # ===================================== RUN ==========================================
    def run(self):
        """Apply initializing methods and start the threads. 
        """
        self.parkThread = Thread(name='ParkFunction', target=self._run_parking, daemon=True)
        self.overtakeThread = Thread(name='OvertakeFunction', target=self._run_overtake, daemon=True)
        # self.lkThread = Thread(name='LaneKeeping', target=self._lane_keeping, daemon=True)

        self.mainThread = Thread(name='TestFunction', target=self._test_function, daemon=True)
        self.mainThread.start()

        self.speedThread = Thread(name='SpeedFunction', target=self._command_speed, daemon=True)
        self.speedThread.start()

        self.angleThread = Thread(name='AngleFunction', target=self._command_angle, daemon=True)
        self.angleThread.start()

        rospy.spin()
        print("Threads closed")

    # ===================================== TEST FUNCTION ====================================
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
    
    def _run_parking(self):

        if self.parking_type == "H":
            ### Check the initial conditions ###
            self.park.check_start("H")
            self.yaw_init_park = self.absolute_yaw_init(self.IMU.yaw)
            self.park.parking_horizontal(self.yaw_init_park)
        elif self.parking_type == "V":
            ### Check the initial conditions ###
            self.park.check_start("V")
            self.yaw_init_park = self.absolute_yaw_init(self.IMU.yaw)
            self.park.parking_vertical(self.yaw_init_park)

        print("Parking finished!")
        self.parking_running = False

    def _run_overtake(self):
        self.yaw_init_over = self.IMU.yaw
        self.over.maneuver(self.yaw_init_over)

        self.overtake_running = False
        print("Overtake finished!")
    
    def _test_function(self):

        self.speed = 10
        self.angle = 0

        counter = 0
        overtake_flag_to_start_the_procedure = False

        try:
            while self.reset is False:
                counter += 1

                self.perception_dict['RayFront'] = self.rayFront.range
                self.perception_dict['RayRight'] = self.rayRight.range
                self.perception_dict['Yaw']      = self.IMU.yaw # MUST BE [-180, 180]
                self.perception_dict['HorLine']  = False
                self.perception_dict['LKAngle']  = 0.0 # lane_keeping_angle

                vehicle_detected = True
                self.over.dotted = True
                graph = 1
                source = 1
                target = 1

                if vehicle_detected == True: 
                    overtake_flag_to_start_the_procedure = self.over.react_to_vehicle(graph, source, target)

                if overtake_flag_to_start_the_procedure and self.overtake_running is False:
                    self.overtakeThread.start()
                    self.overtake_running = True
                    overtake_flag_to_start_the_procedure = False

                # --- WRITE BELOW ---
                if self.overtake_running:
                    self.angle = self.over.get_angle()
                else:
                    self.angle = 0.0
                    self.speed = 15
                    if self.over.finished:
                        print("Time to die [Overtake]")
                        self.overtakeThread.join()
                        self.over.finished = False
                # --- WRITE ABOVE ---

                cv2.imshow("Preview", self.color_cam.cv_image) 
                # cv2.imwrite("Frame_"+str(counter)+".jpg", self.color_cam.cv_image)
                
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    self.reset = True

                time.sleep(0.05)

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