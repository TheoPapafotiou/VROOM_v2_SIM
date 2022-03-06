from mimetypes import init
import numpy as np
import cv2 
import time
from skimage.measure import ransac, LineModelND  
# get gia frame
    
class Intersection2:

    def __init__(self, width, height, get_perception):

        self.get_perc = get_perception 
        self.width = width
        self.height = height
        self.x_points = []
        self.y_points = []
        self.white = (255,255,255)
        self.counter2 = 0
        self.data = []
        self.finished = False  
        self.reached= False
        self.lane_frame_int = np.zeros((self.height, self.width, 3), dtype = np.uint8) 
        self.img = np.zeros((self.height, self.width, 3), dtype = np.uint8)  
        self.lastx = 0
        self.lasty = 0
        self.yaw_diff = 0
        self.increase_angle = False
        self.yaw_angle = False
        self.yaw_diff = 0
        #image processing parameters
        self.ThresholdHigh = 150
        self.ThresholdLow = 350
        self.KSize = (5, 5)
        self.BorderType = 0
        #thresh_callback parameters
        self.xmax = 0
        self.wmax = 0
        self.ymax = 0
        self.hmax = 0

        self.angle_step = 1
    def canny(self, image):
        gray = cv2.cvtColor(image, cv2.COLOR_RGB2GRAY)
        blur = cv2.GaussianBlur(gray, self.KSize, self.BorderType)
        canny = cv2.Canny(blur, self.ThresholdHigh, self.ThresholdLow)
        return canny

    def mask(self, image, case):
        # for the straight 
        if case == "Straight":
            polygons=np.array([
                [(int(self.width*0.35),int(self.height/3.5)),(int(self.width/3),int(self.height*0.45)),(int(self.width*0.5),int(self.height*0.45)),(int(self.width*0.5),int(self.height/3.5))]
                ])

        # for the small right turn 
        elif case == "SmallRight":  
            polygons=np.array([
                [(int(self.width/3), int(self.height*0.45)),(self.width,int(self.height*0.45)),(self.width,int(self.height*0.65)),(int(self.width/3),int(self.height*0.65))]
                ])
        #for the big left turn
        elif case == "BigLeft":
            polygons=np.array([
                [(int(self.width/4), int(self.height*0.35)),(self.width*0.6,int(self.height*0.35)),(self.width*0.5,int(self.height*0.65)),(int(self.width/4),int(self.height*0.65))]
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
            x,y,w,h = cv2.boundingRect(c)

            if ((x+w) > self.width*0.7) and (w > 40):
                # if (self.xmax + self.wmax) < (x + w):
                    # cv2.rectangle(self.cam_frame,(x,y),(x + w,y + h),(0,255,0),2)
                self.xmax = x 
                self.wmax = w 
                self.ymax = y
                self.hmax = h
                if (self.ymax + self.hmax) > self.height*0.65:
                    return self.xmax + self.wmax/2, self.ymax + self.hmax/2

                return self.xmax + self.wmax/2, self.ymax + self.hmax

            return self.width*2/3, self.height/2

    def convert_arc(self, point1, point2, sagitta):

        # extract point coordinates
        x1, y1 = point1
        x2, y2 = point2

        # find normal from midpoint, follow by length sagitta
        n = np.array([y2 - y1, x1 - x2])
        n_dist = np.sqrt(np.sum(n**2))

        if np.isclose(n_dist, 0):
            # catch error here, d(point1, point2) ~ 0
            print('Error: The distance between point1 and point2 is too small.')

        n = n/n_dist
        x3, y3 = (np.array(point1) + np.array(point2))/2 + sagitta * n

        # calculate the circle from three points
        A = np.array([
            [x1**2 + y1**2, x1, y1, 1],
            [x2**2 + y2**2, x2, y2, 1],
            [x3**2 + y3**2, x3, y3, 1]])
        M11 = np.linalg.det(A[:, (1, 2, 3)])
        M12 = np.linalg.det(A[:, (0, 2, 3)])
        M13 = np.linalg.det(A[:, (0, 1, 3)])
        M14 = np.linalg.det(A[:, (0, 1, 2)])

        if np.isclose(M11, 0):
            # catch error here, the points are collinear (sagitta ~ 0)
            print('Error: The third point is collinear.')

        cx = 0.5 * M12/M11
        cy = -0.5 * M13/M11
        radius = np.sqrt(cx**2 + cy**2 + M14/M11)

        # calculate angles of point1 and point2 from center of circle
        point1_angle = 180*np.arctan2(y1 - cy, x1 - cx)/np.pi
        point2_angle = 180*np.arctan2(y2 - cy, x2 - cx)/np.pi

        return (cx, cy), radius, point1_angle, point2_angle

    def draw_ellipse(self, img, center, axes, angle, startAngle, endAngle, color):
        thickness = 14
        lineType = cv2.LINE_AA
        shift = 10
        # uses the shift to accurately get sub-pixel resolution for arc
        center = (
            int(round(center[0] * 2**shift)),
            int(round(center[1] * 2**shift))
        )
        axes = (
            int(round(axes[0] * 2**shift)),
            int(round(axes[1] * 2**shift))
        )
        color = (255,255,255)

        cv2.ellipse(img, center, axes, angle,
                    startAngle, endAngle, color,
                    thickness, lineType, shift)


    def small_right_turn(self, yaw_init):
        
        lane_keeping_threshold = 22
        increasing_angle_threshold = [90-10, 90+10]
        while self.finished is False:
            
            absolute_yaw_diff = np.abs(np.abs(yaw_init) - np.abs(self.get_perc()['Yaw']))

            self.cam_frame=self.get_perc()['Camera']

            if  (absolute_yaw_diff > lane_keeping_threshold) or (self.reached is True ):
                if (increasing_angle_threshold[0] < absolute_yaw_diff < increasing_angle_threshold[1]):
                    self.finished = True
                    self.increase_angle = False
                else :
                    print('====== GRADUALLY INCREASING ANGLE ======')
                    self.increase_angle = True
                    self.reached = True

            if not self.reached:
                cannyimg = self.canny(self.cam_frame)
                maskedimg = self.mask(cannyimg, case="SmallRight")
                
                point1 = (int(self.width/3),int(self.height))
                a,b = self.thresh_callback(maskedimg)
                point2 = (a,b)
                sagitta = 40
                center, radius, start_angle, end_angle = self.convert_arc(point1, point2, sagitta)
                
                axes = (radius, radius)
                self.lane_frame_int = np.zeros((self.height, self.width, 3), dtype = np.uint8)  
                self.draw_ellipse(self.lane_frame_int, center, axes, 0, start_angle, end_angle, 255)
            
                # print('yaw init',yaw_init)
                # print('perc', self.get_perc()['Yaw'])
                # print('yaw_init-Yaw',x)
             
            time.sleep(0.1)

    def straight_yaw(self, yaw_init):
        startD = time.time()
        self.yaw_angle = True

        while self.finished is False:
            # startC=time.time()
            speed = self.get_perc()['Speed']
            self.yaw_diff = np.abs(yaw_init) - np.abs(self.get_perc()['Yaw'])
            endD = time.time()
            endFlag = 140/speed
            # endC=time.time()
            # print('duration', endC - startC)
            if endD-startD > endFlag:
                self.yaw_angle = False
                self.finished = True


    def big_left_turn(self, yaw_init, init_diff):
        lane_keeping_threshold = 20
        increasing_angle_threshold = [65, 95]

        correction = True
        self.yaw_diff = np.abs(yaw_init) - np.abs(self.get_perc()['Yaw'])

        while self.finished is False:
            
            absolute_yaw_diff = np.abs(np.abs(yaw_init) - np.abs(self.get_perc()['Yaw']))
            if (absolute_yaw_diff > lane_keeping_threshold) or (self.reached is True ):
                if (increasing_angle_threshold[0] < absolute_yaw_diff  < increasing_angle_threshold[1]):
                    self.finished = True
                    self.increase_angle = False
                else :
                    print('====== GRADUALLY INCREASING ANGLE ======')
                    self.angle_step = -abs(self.angle_step)
                    self.increase_angle = True
                    self.reached = True

            if not self.reached:      
                print('abs yaw', absolute_yaw_diff)
                if abs(absolute_yaw_diff) > 5.0 and correction is True:
                    print("STRANGE MODEEE")
                    if init_diff > 0.0:
                        self.angle_step = -abs(self.angle_step)
                    else:
                        self.angle_step = abs(self.angle_step)
                    self.increase_angle = True
                
                    
                else:
                    correction = False
                    print("STARTING LANE")
                    
                    self.increase_angle = False
                    self.cam_frame=self.get_perc()['Camera']
                    cannyimg = self.canny(self.cam_frame)
                    maskedimg = self.mask(cannyimg, case="BigLeft")
                    self.img=self.cornerHarris(maskedimg)
                    # print('x', self.x_points)
                    point1 = (int(self.width*0.8),int(self.height))                
                    
                    a = min(self.x_points) 
                    index = self.x_points.index(a)
                    b = self.y_points[index]
                    point2 = (a+70,b)
                    # must change the point because lane keeping cannot racognize or turn the car right .. POURQOI 
                    sagitta = -30
                    center, radius, start_angle, end_angle = self.convert_arc(point1, point2, sagitta)
                    
                    axes = (radius, radius)
                    self.lane_frame_int = np.zeros((self.height, self.width, 3), dtype = np.uint8)  
                    self.draw_ellipse(self.lane_frame_int, center, axes, 0, start_angle, end_angle, 255)
                    self.draw_ellipse(self.img, center, axes, 0, start_angle, end_angle, 255)
                    
                    self.x_points = []
                    self.y_points = []

            time.sleep(0.1)
            






    def ransac_method(self):
        ranges=int(len(self.x_points)/2)
        self.data = np.column_stack([self.x_points,self.y_points])

        model = LineModelND()
        min_sample = 4

        if len(self.data) > min_sample:
            model.estimate(self.data)
            model_robust, inliers = ransac(self.data, LineModelND, min_sample,
                                    residual_threshold=2, max_trials=500)
            inliers == False 
            line_x = self.x_points
            line_y_robust = list(model_robust.predict_y(line_x))

            return line_x, line_y_robust
        
        else:
            return self.x_points, self.y_points

    def cornerHarris(self,img):

        corners = cv2.goodFeaturesToTrack(img,25,0.01,10)

        if corners is not None:

            corners = np.int0(corners)

            for i in corners:
                
                x,y = i.ravel()
                cv2.circle(img,(x,y),3,255,-1)
                
                self.x_points.append(x)
                self.y_points.append(y)
        return img

    def straight(self):
        counter = 0
        counter3 = 0
        startD = time.time()
        self.counter2 = 0
        while self.finished is False:
            startC=time.time()
            counter += 1
            self.cam_frame = self.get_perc()['Camera']
            speed = self.get_perc()['Speed']
            
            cannyimg = self.canny(self.cam_frame)
            maskimg = self.mask(cannyimg, case="Straight")

            self.img=self.cornerHarris(maskimg)
            # if self.counter2 < 3:            
            #     self.x_points,self.y_points = self.ransac_method()
            #     self.counter2 += 1    
            # else:
            #     continue                        
        
            self.counter2 += 1
            if self.counter2 % 7 == 4:
                self.counter2 = 0
                self.x_points,self.y_points= self.ransac_method()                

            if self.y_points[-1] < self.height*0.6: 
                self.lane_frame_int = self.cam_frame
                cv2.line(self.lane_frame_int, (0, int(self.height)),(int(self.x_points[-1]), int(self.y_points[-1])), self.white, 14)
                # cv2.line(self.lane_frame_int, (self.width, int(self.height)),(int(self.width-self.x_points[-1]), int(self.y_points[-1])), self.white, 14)
                cv2.line(self.img, (int(self.width*0.1), int(self.height)),(int(self.x_points[counter]), int(self.y_points[counter])), self.white, 14)
                cv2.line(self.img, (int(self.width*0.9), int(self.height)),(int(self.width-self.x_points[-1]), int(self.y_points[-1])), self.white, 14)                
                counter3 = 2
                self.lastx = self.x_points[-1]
                self.lasty = self.y_points[-1]
            else: 
                print('hhhhhhhhhhhhhhhhhhhhhhhhh')
                self.lane_frame_int = self.cam_frame
                cv2.line(self.lane_frame_int, (int(self.width*0.1), int(self.height)),(int(self.x_points[-counter3]), int(self.y_points[-counter3])), self.white, 14)
                cv2.line(self.lane_frame_int, (int(self.width*0.9), int(self.height)),(int(self.width-self.x_points[-counter3]), int(self.y_points[-counter3])), self.white, 14)
                # cv2.line(self.img, (int(self.width*0.1), int(self.height)),(int(self.x_points[counter-counter3]), int(self.y_points[counter-counter3])), self.white, 14)
                # cv2.line(self.img, (int(self.width*0.9), int(self.height)),(int(self.width-self.x_points[counter-counter3]), int(self.y_points[counter-counter3])), self.white, 14)
                counter3 = counter3 + 1

            endD = time.time()
            endC=time.time()
            endFlag = 140/speed
            # print('duration', endC-startC)
            if endD-startD > endFlag:
                self.finished=True
