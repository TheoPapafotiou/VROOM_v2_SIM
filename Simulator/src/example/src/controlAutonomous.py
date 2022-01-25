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

        # --- WRITE BELOW ---
        print(Path("src/example/src/default_mask_real.json").absolute())
        self.det = DetectHorizontal(mask_filename="src/example/src/default_mask_simulation2.json")

        # --- WRITE ABOVE ---

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
        self.angle = 0

        counter = 0

        try:
            while self.reset is False:
                counter += 1

                # --- WRITE BELOW ---
                dict_hor = self.det.detection(self.color_cam.cv_image)
                print(dict_hor["avg_y"])
                # --- WRITE ABOVE ---

                cv2.imshow("Preview", self.color_cam.cv_image) 
                # cv2.imwrite("Frame_"+str(counter)+".jpg", self.color_cam.cv_image)
                
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    self.reset = True

                time.sleep(0.01)

            self.speed = 0.0
        
        except Exception as e:
            print(e)

    def display_lines_on_img(self, img, lines, thickness=10, wait=True):
        line_image = np.zeros_like(img)
        # if lines[0].size == 0:
        #     cv2.imshow("lines", img)
        #     if wait:
        #         cv2.waitKey()
        # else:
        for line in lines:
            # print(line)
            pass
        try:
            if len(lines) != 1:
                for line in lines:
                    # x1, y1, x2, y2 = line.reshape(4)
                    x1, y1, x2, y2 = line[0]
                    cv2.line(line_image, (int(x1), int(y1)), (int(x2), int(y2)), (255, 0, 0), thickness=thickness)
                    combined_image = cv2.addWeighted(img, 0.8, line_image, 1, 1)
                    cv2.imshow("lines", combined_image)
                    # if wait:
                    #     cv2.waitKey()
                    if wait:
                        cv2.waitKey()
            else:
                x1, y1, x2, y2 = lines[0]
                cv2.line(line_image, (int(x1), int(y1)), (int(x2), int(y2)), (255, 0, 0), thickness=thickness)
                combined_image = cv2.addWeighted(img, 0.8, line_image, 1, 1)
                cv2.imshow("lines", combined_image)
                if wait:
                    cv2.waitKey()

            return combined_image
            # if line.size == 0:
            #     raise
        except:
            cv2.imshow("lines", img)
            if wait:
                cv2.waitKey()
                 
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