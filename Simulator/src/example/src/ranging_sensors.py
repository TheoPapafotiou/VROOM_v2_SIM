#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Range

class RangeHandler:
    # ===================================== INIT==========================================
    def __init__(self, pos):
        """
        Creates a bridge for converting the image from Gazebo image into OpenCV image
        """
        # rospy.init_node('RAYnod', anonymous=True)
        self.range_sub = rospy.Subscriber("/ir_" + pos, Range, self.callback)
        self.range = 0.0
        # rospy.spin()

    def callback(self, data):
        """
        :param data: sensor_msg array containing the image in the Gazebo format
        :return: nothing but sets [cv_image] to the useful image that can be use in opencv (numpy array)
        """
        self.range = round(data.range, 4)
    
            
if __name__ == '__main__':
    try:
        nod = RangeHandler("front")
    except rospy.ROSInterruptException:
        pass
