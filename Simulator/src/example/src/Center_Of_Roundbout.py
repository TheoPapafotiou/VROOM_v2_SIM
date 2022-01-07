import cv2
from matplotlib import image 
import matplotlib.pyplot as pl
import numpy as np 
import math
import sys
#from DetectHorizontal import DetectHorizontal

np.set_printoptions(threshold=sys.maxsize)

# Initialazing some colors
WHITE = (255, 255, 255)
RED = (255, 0, 0)
GREEN = (20, 255, 80)
BLUE = (0, 0, 255)
YELLOW = (255, 255, 0)


class CenterOfRoundabout():
    def __init__(self, width, height):
        self.image = np.zeros((width, height))
        self.image_copy = np.zeros((width, height))
        self.image_copy2 = np.zeros((width, height))
        self.width = width
        self.height = height
        self.thresh = None
        self.contours = None
        self.centers = None
        self.finalCenter = None
        self.closestCenter = None
        self.lastPoints = None
        self.yaw = 0.0
        self.starting_yaw = 0.0
        self.threshold = 14

        ## Line Params
        self.line_thickness = 40

    
    def preprocessing(self):
        """Saving an image copy and making the proper preprocessing"""
        # Saving a copy and converting it to RGB
        self.image_copy = self.image.copy()
        self.image_copy = cv2.cvtColor(self.image, cv2.COLOR_BGR2RGB)
        # BGR to GRAY
        
        self.image = cv2.cvtColor(self.image, cv2.COLOR_BGR2GRAY)
         # Converting image to binary a binary one 
        _, binary = cv2.threshold(self.image, 254, 255, cv2.THRESH_BINARY_INV)
        # Setting a threshold to detect only clear lines 
        ret, thresh = cv2.threshold(binary, 240, 255, 0)
        self.thresh = thresh

    
    def findAndDrawContours(self):
        """Finding all the kind of contrours """
        # find the contours from the thresholded image
        contours, hierarchy = cv2.findContours(self.thresh, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        # draw all contours
        self.image = cv2.cvtColor(self.image, cv2.COLOR_GRAY2BGR)
        self.image = cv2.drawContours(self.image, contours, -1, RED, 1)
        self.contours = contours


    def get_contour_centers(self): 
        """Finding all the possible centers of all possible contours"""
    
        # ((x, y), radius) = cv2.minEnclosingCircle(c)
        centers = np.zeros((len(self.contours), 2), dtype=np.int16)
        for i, c in enumerate(self.contours):
            M = cv2.moments(c)
            try:
                center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))
            except Exception as e:
                print(e)
                center = (int(M["m10"] / 0.0001), int(M["m01"] / 0.0001))
                pass
            centers[i] = center
        self.centers = centers
    

    def drawPoints(self):    
        """Drawing all the possible center with small dots"""
        print(len(self.centers))
        for center in self.centers:
            self.image = cv2.circle(self.image, tuple(center), radius=0, color=BLUE, thickness= 5)    
            

    def findTheCenter(self):
        """Finding the closest center to the left bottom point of the image"""
        self.finalCenter = self.centers[0]
      
        self.closestCenter = 0
        x = self.centers[0][0]
        y = self.centers[0][1]
        smallestDistance = math.sqrt(abs((self.height - y) ** 2 + (0 - x) ** 2))

        for index, center in enumerate(self.centers[1:]):
            x = center[0]
            y = center[1]

            distance = math.sqrt((self.height - y) ** 2 + (0 - x) ** 2)     
            
            if distance < smallestDistance:
                self.closestCenter = index + 1
                smallestDistance = distance

        self.image = cv2.circle(self.image, tuple(self.centers[self.closestCenter]), radius=0, color=YELLOW, thickness= 7)       
        print("index is {}".format(index))
        print("The length is {}".format(len(self.centers)))

    
    def findTheLine(self):
        """Find the point that we will connect our created line"""
        
        # Save the coordinates of the closest center
        x_closest = self.centers[self.closestCenter][0]
        y_closest = self.centers[self.closestCenter][1]         
                 
        for x in range(x_closest, self.image_copy.shape[1] - 1):
            value = [self.image_copy[y_closest][x]]
            if value[0][0] > 200 and value [0][1] > 200 and value [0][2] > 200:
                self.image = cv2.circle(self.image, (x, y_closest), radius=0, color=GREEN, thickness= 8)  
                cv2.line(self.image_copy, (0, self.height), (x, y_closest), WHITE, thickness=self.line_thickness) ### Here is hardcoded. Needs to be fixed
                cv2.line(self.image_copy2, (0, self.height), (x, y_closest), WHITE, thickness=self.line_thickness)
                cv2.line(self.image, (0, self.height), (x, y_closest), WHITE, thickness=self.line_thickness) ### Here is hardcoded. Needs to be fixed

                break

    
    def findTheAngle(self):
        """Finding the angle of our created line"""
        # Save the coordinates of the closest center
        x_closest = self.centers[self.closestCenter][0]
        y_closest = self.centers[self.closestCenter][1]
        y =  self.height - y_closest
        x = x_closest

        AB = y
        BC = x
        AC = math.sqrt(AB ** 2 + BC **2)
        print("The angle is {}".format(math.degrees(math.atan(BC / AB)))) 

    def checkStopFlag(self):
        """yaw > threshold = stop"""
        STOP = False
        if abs(self.yaw - self.starting_yaw) > self.threshold:
            STOP = True

        return STOP    


    def centerOfRoundabout(self, image, yaw, starting_yaw):
        self.image = image
        self.yaw = yaw
        self.starting_yaw = starting_yaw

        if not self.checkStopFlag():
            self.preprocessing()
            print(1)
            self.findAndDrawContours()
            print(2)
            self.get_contour_centers()
            print(3)
            self.drawPoints()
            print(4)
            self.findTheCenter()
            print(5)
            self.findTheLine()
            print(6)
            self.findTheAngle()
            print(7)

        else:
            return False

            # # Show the image. Print any button to quit
            # pl.imshow(self.image_copy2)
            # pl.show() 
            # cv2.imwrite("result3496.png", self.image_copy2 )       
            # pl.imshow(self.image_copy)
            # pl.show() 
            # cv2.imwrite("result3496c.png", self.image_copy )       

        return self.image

  
if __name__ == "__main__":
    image = CenterOfRoundabout(cv2.imread("frame3496.png"), 4)     
    image.centerOfRoundabout()