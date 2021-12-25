#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Range

class RangeHandler:
    # ===================================== INIT==========================================
    def __init__(self):
        """
        Creates a bridge for converting the image from Gazebo image into OpenCV image
        """
        rospy.init_node('RAYnod', anonymous=True)
        self.range_sub = rospy.Subscriber("/ir_front", Range, self.callback)
        rospy.spin()

    def callback(self, data):
        """
        :param data: sensor_msg array containing the image in the Gazebo format
        :return: nothing but sets [cv_image] to the useful image that can be use in opencv (numpy array)
        """
        print("Distance: ", round(data.range, 4))
    
            
if __name__ == '__main__':
    try:
        nod = RangeHandler()
    except rospy.ROSInterruptException:
        pass
