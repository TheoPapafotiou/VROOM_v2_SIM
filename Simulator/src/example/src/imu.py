#!/usr/bin/env python3

import rospy
import math
from utils.msg import IMU

rad2deg = 57.29578

class IMUHandler:
    # ===================================== INIT==========================================
    def __init__(self):
        """
        Creates a bridge for converting the image from Gazebo image into OpenCV image
        """
        self.yaw = 0.0
        self.pitch = 0.0
        self.roll = 0.0
        # rospy.init_node('IMUnod', anonymous=True)
        self.imu_node = rospy.Subscriber("/automobile/IMU", IMU, self.callback)
        # rospy.spin()

    def callback(self, data):
        """
        :param data: sensor_msg array containing the image in the Gazebo format
        :return: nothing but sets [cv_image] to the usefull image that can be use in opencv (numpy array)
        """
        self.yaw = rad2deg * round(data.yaw, 4)
        self.roll = rad2deg * round(data.roll, 4)
        self.pitch = rad2deg * round(data.pitch, 4)
        print(self.yaw)
            
if __name__ == '__main__':
    try:
        nod = IMUHandler()
    except rospy.ROSInterruptException:
        pass