#!/usr/bin/env python3

import rospy
from utils.msg import localisation

class GPSHandler:
    # ===================================== INIT==========================================
    def __init__(self):
        """
        Creates a bridge for converting the image from Gazebo image into OpenCV image
        """
        self.pos = (0.0, 0.0)
        # rospy.init_node('GPSnod', anonymous=True)
        self.gps_node = rospy.Subscriber("/automobile/localisation", localisation, self.callback)
        # rospy.spin()

    def callback(self, data):
        """
        :param data: sensor_msg array containing the image in the Gazebo format
        :return: nothing but sets [cv_image] to the usefull image that can be use in opencv (numpy array)
        """
        self.pos = (round(data.posA, 3), round(data.posB, 3))
        # print(self.gps)
            
if __name__ == '__main__':
    try:
        nod = GPSHandler()
    except rospy.ROSInterruptException:
        pass