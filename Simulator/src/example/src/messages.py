import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from sensor_msgs.msg import Range
from cv_bridge import CvBridge, CvBridgeError

class CameraHandler():
    # ===================================== INIT==========================================
    def __init__(self):
        """
        Creates a bridge for converting the image from Gazebo image into OpenCV image
        """
        self.bridge = CvBridge()
        self.cv_image = np.zeros((640, 480))
        rospy.init_node('CAMnod', anonymous=True)
        self.image_sub = rospy.Subscriber("/automobile/depth_image_raw", Image, self.callback)
        rospy.spin()

    def callback(self, data):
        """
        :param data: sensor_msg array containing the image in the Gazebo format
        :return: nothing but sets [cv_image] to the usefull image that can be use in opencv (numpy array)
        """
        self.cv_image = self.bridge.imgmsg_to_cv2(data, desired_encoding='passthrough')
        cv2.imshow("Frame preview", self.cv_image)
        key = cv2.waitKey(1)
    
            
if __name__ == '__main__':
    try:
        nod = CameraHandler()
    except rospy.ROSInterruptException:
        pass