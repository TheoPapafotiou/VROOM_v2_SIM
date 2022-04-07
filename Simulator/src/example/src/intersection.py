###  This file is for Intersection navigation:
#      There are 4 main functions for each case and type of path in an intersection
#       for instance if you want to run the small right turn you can call the def : small_right_turn() 
#       it takes two arguments that they are given inside the logicprocess.py, where the function is called
#
##  ENTER intersection mode:
#   the check is done in the "logicprocess.py" and it is based on the flag-sign which represents the start of an intersection and in the horizontal line,
#   which is a common characteristic in the begging of all the intersections.
#   We have found a specific threshold distance and when the car is in this position then the intersection mode starts
#
##  EXIT intersection mode:
#   We initialize the parameter: self.finished = False 
#   The flags for exiting intersection mode are based on the crossing duration (in the straight) and in the yaw of the car (turns)
#   Straight: if the car reaches a specific time threshold (time difference) then the self.finished = True 
#   Turn: when the yaw of the car is a value among our max,min threshold values then the self.finished = True
#   When the self.finished = True then the car continues the lane keeping mode
#
##  SMALL RIGHT TURN:
#   The function firstly checks the yaw of the car in order to correct if it is necessary to succeed the task. Then, after it searchs for points in our region of interest
#   with the aid of the function thresh_callback(), it draws an arc as a new lane to navigate the car by running lane keeping mode  
#   To run the small right turn you have to go to the logicprocess.py file and change the parameter self.intersection_typ in order to execute the path you want 
#   for instance if you want to turn small right then : self.intersection_type = "R" 
#
##  STRAIGHT:
#   it is based on the yaw of the car with a check and correct mode. the angle is given trying to keep it equal to zero
#   To run the directions are similar to the small right turn -> self.intersection_type = "S"   
# 
##  BIG LEFT TURN: 



# TODO: A fix way to run the double lane road as a backup also in case something goes wrong with the other method and for the different types of turn or straight paths 

from audioop import cross
import numpy as np
import cv2 
import time


class Intersection:

    def __init__(self, width, height, get_perception):

        self.get_perc = get_perception 
        self.width = width
        self.height = height
        self.x_points = []
        self.y_points = []
        self.finished = False  
        self.reached = False
        self.lane_frame_int = np.zeros((self.height, self.width, 3), dtype=np.uint8)
        # self.img = np.zeros((self.height, self.width, 3), dtype=np.uint8)
      
        self.yaw_diff = 0
        self.increase_angle = False
        self.decrease_angle = False
        self.yaw_angle = False
        self.angle_step = 1
        self.angle = 0.0
        self.return_angle = False
        self.last_angle = 0.0
        self.modify_angle = False

        # image processing parameters
        self.ThresholdHigh = 150
        self.ThresholdLow = 350
        self.KSize = (5, 5)
        self.BorderType = 0

        # thresh_callback parameters
        self.xmax = 0
        self.wmax = 0
        self.ymax = 0
        self.hmax = 0

    def canny(self, image):
        gray = cv2.cvtColor(image, cv2.COLOR_RGB2GRAY)
        blur = cv2.GaussianBlur(gray, self.KSize, self.BorderType)
        canny = cv2.Canny(blur, self.ThresholdHigh, self.ThresholdLow)
        return canny

    def mask(self, image, case):
        if case == "Straight":
            polygons = np.array([
                [(int(self.width*0.35), int(self.height/3.5)), (int(self.width/3), int(self.height*0.45)), (int(self.width*0.5), int(self.height*0.45)), (int(self.width*0.5), int(self.height/3.5))]
                ])

        elif case == "SmallRight":  
            polygons = np.array([
                [(int(self.width/3), int(self.height*0.45)), (self.width, int(self.height*0.45)), (self.width, int(self.height*0.65)), (int(self.width/3), int(self.height*0.65))]
                ])

        elif case == "BigLeft":
            polygons = np.array([
                [(int(self.width/4), int(self.height*0.35)), (self.width*0.6, int(self.height*0.35)), (self.width*0.5, int(self.height*0.65)), (int(self.width/4), int(self.height*0.65))]
                ])
        mask = np.zeros_like(image)
        cv2.fillPoly(mask, np.int32([polygons]), 255)
        masked = cv2.bitwise_and(image, mask)

        return masked

    def thresh_callback(self, maskedimg):
        self.xmax = 0
        self.wmax = 0
        contours = cv2.findContours(maskedimg, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        contours = contours[0] if len(contours) == 2 else contours[1]

        for c in contours:
            x, y, w, h = cv2.boundingRect(c)

            if ((x+w) > self.width*0.7) and (w > 40):
                self.xmax = x 
                self.wmax = w 
                self.ymax = y
                self.hmax = h
                if (self.ymax + self.hmax) > self.height*0.65:
                    return self.xmax + self.wmax/2, self.ymax + self.hmax/2

                return self.xmax + self.wmax/2, self.ymax + self.hmax

            return self.width*2/3, self.height/2

    def convert_arc(self, point1, point2, sagitta):

        x1, y1 = point1
        x2, y2 = point2

        # find normal from midpoint, follow by length sagitta
        n = np.array([y2 - y1, x1 - x2])
        n_dist = np.sqrt(np.sum(n**2))

        if np.isclose(n_dist, 0):
            print('Error: The distance between point1 and point2 is too small.')

        n = n/n_dist
        x3, y3 = (np.array(point1) + np.array(point2))/2 + sagitta * n

        A = np.array([
            [x1**2 + y1**2, x1, y1, 1],
            [x2**2 + y2**2, x2, y2, 1],
            [x3**2 + y3**2, x3, y3, 1]])
        M11 = np.linalg.det(A[:, (1, 2, 3)])
        M12 = np.linalg.det(A[:, (0, 2, 3)])
        M13 = np.linalg.det(A[:, (0, 1, 3)])
        M14 = np.linalg.det(A[:, (0, 1, 2)])

        if np.isclose(M11, 0):
            print('Error: The third point is collinear.')

        cx = 0.5 * M12/M11
        cy = -0.5 * M13/M11
        radius = np.sqrt(cx**2 + cy**2 + M14/M11)

        point1_angle = 180*np.arctan2(y1 - cy, x1 - cx)/np.pi
        point2_angle = 180*np.arctan2(y2 - cy, x2 - cx)/np.pi

        return (cx, cy), radius, point1_angle, point2_angle

    def draw_ellipse(self, img, center, axes, angle, startAngle, endAngle):
        thickness = 14
        lineType = cv2.LINE_AA
        shift = 10
        color = (255, 255, 255)
        # uses the shift to accurately get sub-pixel resolution for arc
        center = (
            int(round(center[0] * 2**shift)),
            int(round(center[1] * 2**shift))
        )
        axes = (
            int(round(axes[0] * 2**shift)),
            int(round(axes[1] * 2**shift))
        )

        cv2.ellipse(img, center, axes, angle,
                    startAngle, endAngle, color,
                    thickness, lineType, shift)

    def cornerHarris(self, img):
        corners = cv2.goodFeaturesToTrack(img, 25, 0.01, 10)

        if corners is not None:
            corners = np.int0(corners)
            for i in corners:
                x, y = i.ravel()
                cv2.circle(img, (x, y), 3, 255, -1)
                
                self.x_points.append(x)
                self.y_points.append(y)

    def small_right_turn(self, yaw_init):
        
        lane_keeping_threshold = 22
        increasing_angle_threshold = [80, 100]
        while self.finished is False:
            
            absolute_yaw_diff = np.abs(np.abs(yaw_init) - np.abs(self.get_perc()['Yaw']))
            self.cam_frame = self.get_perc()['Camera']

            if (absolute_yaw_diff > lane_keeping_threshold) or (self.reached is True):
                if increasing_angle_threshold[0] < absolute_yaw_diff < increasing_angle_threshold[1]:
                    self.finished = True
                    self.increase_angle = False
                else:
                    print('====== GRADUALLY INCREASING ANGLE ======')
                    self.increase_angle = True
                    self.reached = True

            if not self.reached:
                cannyimg = self.canny(self.cam_frame)
                maskedimg = self.mask(cannyimg, case="SmallRight")
                
                point1 = (int(self.width/3), int(self.height))
                a, b = self.thresh_callback(maskedimg)
                point2 = (a, b)
                sagitta = 40
                center, radius, start_angle, end_angle = self.convert_arc(point1, point2, sagitta)
                
                axes = (radius, radius)
                self.lane_frame_int = np.zeros((self.height, self.width, 3), dtype=np.uint8)
                self.draw_ellipse(self.lane_frame_int, center, axes, 0, start_angle, end_angle)
             
            time.sleep(0.1)

    # def straight_yaw(self, yaw_init):

    #     start = time.time()
        
    #     self.yaw_angle = True

    #     while self.finished is False:
    #         speed = self.get_perc()['Speed']
    #         self.yaw_diff = np.abs(yaw_init) - np.abs(self.get_perc()['Yaw'])
    #         end = time.time()
    #         endFlag = 140/speed
            
    #         if end-start > endFlag:
    #             self.yaw_angle = False
    #             self.finished = True

    def straight_yaw(self, yaw_init, speed):

        start = time.time()

        self.return_angle = True

        crossing_distance = 140 # (cm)

        crossing_duration = crossing_distance/speed # (s)
        
        while self.finished is False:
            self.yaw_diff = yaw_init - self.get_perc()['Yaw']
            print("YAW DIFF = ", self.yaw_diff)
            print("YAW INIT = ", yaw_init)

            self.angle = self.yaw_diff
            self.angle = (self.angle + self.last_angle)/2
            self.last_angle = self.angle
            
            if time.time() - start > crossing_duration:
                self.return_angle = False
                self.finished = True

            time.sleep(0.1)

    def big_left_turn(self, yaw_init, init_diff):
        lane_keeping_threshold = 18
        decreasing_angle_threshold = [80, 90]
        correction = True
        self.yaw_diff = np.abs(yaw_init) - np.abs(self.get_perc()['Yaw'])

        while self.finished is False:

            absolute_yaw_diff = np.abs(np.abs(yaw_init) - np.abs(self.get_perc()['Yaw']))

            if (absolute_yaw_diff > lane_keeping_threshold) or (self.reached is True):
                if decreasing_angle_threshold[0] < absolute_yaw_diff < decreasing_angle_threshold[1]:
                    self.finished = True
                    self.decrease_angle = False
                else:
                    print('====== GRADUALLY DECREASING ANGLE ======')
                    self.angle_step = -abs(self.angle_step)
                    self.decrease_angle = True
                    self.reached = True

            if not self.reached:      
                if abs(absolute_yaw_diff) > 7.0 and correction is True:
                    # print("STRANGE MODE")
                    if init_diff > 0.0:
                        self.angle_step = -abs(self.angle_step)
                    else:
                        self.angle_step = abs(self.angle_step)
                    self.decrease_angle = True
                
                else:
                    correction = False
                    # print("STARTING LANE")
                    
                    self.decrease_angle = False
                    self.cam_frame = self.get_perc()['Camera']
                    cannyimg = self.canny(self.cam_frame)
                    maskedimg = self.mask(cannyimg, case="BigLeft")
                    self.cornerHarris(maskedimg)

                    point1 = (int(self.width), int(self.height*0.9))
                    a = min(self.x_points) 
                    index = self.x_points.index(a)
                    b = self.y_points[index]
                    point2 = (a+70, b)
                    sagitta = -30
                    center, radius, start_angle, end_angle = self.convert_arc(point1, point2, sagitta)
                    axes = (radius, radius)

                    self.lane_frame_int = np.zeros((self.height, self.width, 3), dtype=np.uint8)
                    self.draw_ellipse(self.lane_frame_int, center, axes, 0, start_angle, end_angle)
                    # self.draw_ellipse(self.img, center, axes, 0, start_angle, end_angle, 255)
                    
                    self.x_points = []
                    self.y_points = []
            
            time.sleep(0.1)
            
    def small_left_turn(self, yaw_init, speed):
        
        start = time.time()

        self.return_angle = True

        lane_keeping_threshold = 80

        crossing_distance = 160 # (cm)

        crossing_duration = crossing_distance/speed # (s)

        time.sleep(1.5)

        while self.finished is False:
    
            absolute_yaw_diff = np.abs(np.abs(yaw_init) - np.abs(self.get_perc()['Yaw']))

            if (absolute_yaw_diff < lane_keeping_threshold):
                
                self.angle -= 1
                print('====== GRADUALLY INCREASING ANGLE ======')
                print('angle: ', self.angle)

            else :
                print('========= GOING STRAIGHT ===============')
                self.angle = 0

            if time.time() - start > crossing_duration:

                self.return_angle = False
                self.finished = True

            time.sleep(0.1)

    def get_angle(self):
        return self.angle

    def straight_double(self,yaw_init,speed,init_diff):
        start = time.time()

        self.return_angle = True
        
        crossing_distance = 160

        straight_distance = 50
        
        diagonal_distance = 30 # TODO: measure with evesion

        crossing_duration = crossing_distance/speed

        straight_duration = straight_distance/speed

        diagonal_duration = diagonal_distance/speed

        correction = True

        while self.finished is False:
            absolute_yaw_diff = np.abs(np.abs(yaw_init) - np.abs(self.get_perc()['Yaw']))

            self.yaw_diff = yaw_init - self.get_perc()['Yaw']
            print("YAW DIFF = ", self.yaw_diff)
            print("YAW INIT = ", yaw_init)
           
            duration = time.time() - start
            
            if duration <= straight_duration:
                self.angle = self.yaw_diff
                self.angle = (self.angle + self.last_angle)/2
                self.last_angle = self.angle
            else:
                start2 = time.time()
                self.angle = -7 # TODO: test in simulation 
                while time.time() - start2 < diagonal_duration or correction:
                    self.angle = -7
                    correction = False
                if abs(absolute_yaw_diff) > 7.0 and correction is not True:
                    print("*******Trying to fix my yaw*******")
                    self.angle_step = -np.sign(init_diff) * abs(self.angle_step)
                    print('angle step:',self.angle_step)
                    self.modify_angle = True
                elif abs(absolute_yaw_diff) < 7.0 and correction is not True:
                    self.modify_angle = False
                    self.angle = 0
    
                # print("START CORRECTION")
                # self.yaw_diff = yaw_init - self.get_perc()['Yaw']
                # self.angle = self.yaw_diff
                # # self.angle = (self.angle + self.last_angle)/2
                # # self.last_angle = self.angle
            
            if time.time() -  start > crossing_duration:
                self.return_angle = False
                self.finished
            
            time.sleep(0.1)



## questions: in the straight we have not initiallize the last_angle how does it run? 