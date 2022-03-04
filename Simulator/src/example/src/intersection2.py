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
        self.right = []
        self.left = []
        self.finished = False
        self.x_points = []
        self.y_points = []
        self.white = (255,255,255)
        self.counter2 = 0
        self.data = []
        # self.frame = np.zeros((640, 480, 3))      
        self.finished = False  
        self.reached= False
        self.framecount=0
        self.lane_frame_int = np.zeros((self.width, self.height, 3), dtype = np.uint8)  
   
    def get_x_point(self):
        return self.x_points


    def get_y_point(self):
        return self.y_points


    def canny(self, image):
        gray = cv2.cvtColor(image, cv2.COLOR_RGB2GRAY)
        blur = cv2.GaussianBlur(gray, (5,5), 0)
        canny = cv2.Canny(blur, 150, 350)
        return canny

    def mask(self,image,case):
        # for the straight 
        if case == 1:
            polygons=np.array([
                [(int(self.width*0.4),int(self.height/3.5)),(int(self.width/3),int(self.height*0.45)),(int(self.width*0.5),int(self.height*0.45)),(int(self.width*0.5),int(self.height/3.5))] #(y,x)
                ])

        # for the small right turn 
        elif case == 2:  
            polygons=np.array([
                [(int(self.width/3), int(self.height*0.45)),(self.width,int(self.height*0.45)),(self.width,self.height*0.7),(int(self.width/3),self.height*0.7)] #(y,x)
                ])
        mask = np.zeros_like(image)
        cv2.fillPoly(mask, np.int32([polygons]), 255)
        masked = cv2.bitwise_and(image, mask)
        return masked

    def thresh_callback(self,maskedimg):
        xmax=0
        wmax=0
        ymax=0
        contours= cv2.findContours(maskedimg, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        contours=contours[0] if len(contours)==2 else contours[1]

        for c in contours:
            x,y,w,h = cv2.boundingRect(c)
            if w > 40: 
                if (xmax+wmax) < (x+w):
                    cv2.rectangle(self.cam_frame,(x,y),(x+w,y+h),(0,255,0),2)
                    xmax = x 
                    wmax = w 
                    ymax= y
                    hmax=h
                    if y > self.height*0.6:
                        self.reached= True
                    if (ymax+hmax) > self.height*0.65:
                        return xmax+wmax/2, ymax+hmax/2

                return xmax+wmax/2,ymax+hmax

    def make_lines(self,lines,newwidth,start):
        xl = []
        yl = []
        xr = []
        yr = []
        leftfit = []
        rightfit= []
        if lines is None:
            return np.array(leftfit), np.array(rightfit)
        for line in lines:
            x1, y1, x2, y2 = line[0]
            if (x1<(newwidth*2/3+start)) and (x2<(newwidth*4/5+start)):
                yl.append(y1)
                yl.append(y2)
                xl.append(x1)
                xl.append(x2)
            else:
                yr.append(y1)
                yr.append(y2)
                xr.append(x1)
                xr.append(x2)

        SlopeL, MiddleL = np.polyfit(xl, yl, 1)
        SlopeR, MiddleR = np.polyfit(xr, yr, 1)
        rightfit.append((SlopeR,MiddleR))
        leftfit.append((SlopeL,MiddleL))
    
        return leftfit,rightfit 

    def intersection(self,left,right):
        a1,b1=left[0]
        a2,b2=right[0]
        A = np.array([[-a1,1],[-a2, 1]])
        b = np.array([[b1], [b2]])
        xi, yi = np.linalg.solve(A, b)
        xi, yi = int(np.round(xi)), int(np.round(yi))
        # print('(xi,yi)',xi,yi)

        return xi, yi

    def convert_arc(self,pt1, pt2, sagitta):

        # extract point coordinates
        x1, y1 = pt1
        x2, y2 = pt2

        # find normal from midpoint, follow by length sagitta
        n = np.array([y2 - y1, x1 - x2])
        n_dist = np.sqrt(np.sum(n**2))

        if np.isclose(n_dist, 0):
            # catch error here, d(pt1, pt2) ~ 0
            print('Error: The distance between pt1 and pt2 is too small.')

        n = n/n_dist
        x3, y3 = (np.array(pt1) + np.array(pt2))/2 + sagitta * n

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

        # calculate angles of pt1 and pt2 from center of circle
        pt1_angle = 180*np.arctan2(y1 - cy, x1 - cx)/np.pi
        pt2_angle = 180*np.arctan2(y2 - cy, x2 - cx)/np.pi

        return (cx, cy), radius, pt1_angle, pt2_angle

    def draw_ellipse(self,
        img, center, axes, angle,
        startAngle, endAngle, color):
        thickness=14
        lineType=cv2.LINE_AA
        shift=10
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

        while self.finished is False:
            # if  np.abs(np.abs(yaw_init) - np.abs(self.get_perc()['Yaw'])) >18:
            #     self.same_frame=True
            #     continue
            if self.framecount%2==1:
                self.framecount+=1
                continue
            
            self.cam_frame=self.get_perc()['Camera']
            
            cannyimg=self.canny(self.cam_frame)
            maskedimg = self.mask(cannyimg,2)

            # self.frame = maskedimg
            
            # lines = cv2.HoughLinesP(maskedimg, rho=2, theta=np.pi/180, threshold=20, lines=np.array([]),
            #                             minLineLength=3, maxLineGap=40)
            # start,newwidth=self.thresh_callback(maskedimg)
            # left,right = self.make_lines(lines,newwidth,start)
            # a,b= self.intersection(left,right)          
            # pt1 = (int(self.height*0.34),int(self.width))

            
            #====== Test code===
            pt1=(int(self.width/4),int(self.height))
            a,b=self.thresh_callback(maskedimg)
            pt2 = (a,b)
            sagitta = 40
            center, radius, start_angle, end_angle = self.convert_arc(pt1, pt2, sagitta)
            
            axes = (radius, radius)
            self.draw_ellipse(self.cam_frame, center, axes, 0, start_angle, end_angle, 255)
           
            x=np.abs(np.abs(yaw_init) - np.abs(self.get_perc()['Yaw'])) 
            
            self.framecount+=1
            # print('yaw init',yaw_init)
            # print('perc', self.get_perc()['Yaw'])
            print('yaw_init-Yaw',x)
            cv2.imshow('t',self.cam_frame)
            cv2.waitKey(1)
            if  (np.abs(np.abs(yaw_init) - np.abs(self.get_perc()['Yaw']))>36) or (self.reached is True ):
                self.finished=True

            time.sleep(0.15)

    def ransac_method(self):
        self.data = np.column_stack([self.x_points,self.y_points])
        model = LineModelND()
        min_sample = 4
        if len(self.data) > min_sample:
            model.estimate(self.data)
            model_robust, inliers = ransac(self.data, LineModelND, min_sample,
                                    residual_threshold=2, max_trials=1000)
            outliers = inliers == False 
            line_x = self.x_points
            line_y = model.predict_y(line_x)
            line_y_robust = list(model_robust.predict_y(line_x))
            return line_x, line_y_robust
        else:
            return self.x_points,self.y_points

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

        while self.finished is False:
            self.cam_frame=self.get_perc()['Camera']
            speed = self.get_perc()['Speed']
            
            cannyimg = self.canny(self.cam_frame)
            img = self.mask(cannyimg,1)

            img = self.cornerHarris(img)
            # cv2.putText(img, 'SIMPLE HARRIS', (50,50), cv2.FONT_HERSHEY_SIMPLEX, 1,(255,255,255),2,cv2.LINE_4)

            if self.counter2 < 5:            
                self.x_points,self.y_points= self.ransac_method()
                self.counter2 += 1                            
            if self.counter2 >= 5:
                self.counter2 += 1
                if self.counter2 % 5 == 0:
                    self.x_points,self.y_points= self.ransac_method()
                # keep the points on the upper part of the img
                if self.y_points[counter] < self.height*0.6: 
                    cv2.line(self.lane_frame_int, (int(self.width*0.1), int(self.height)),(int(self.x_points[counter]), int(self.y_points[counter])), self.white, 14)
                    cv2.line(self.lane_frame_int, (int(self.width*0.9), int(self.height)),(int(self.width-self.x_points[counter]), int(self.y_points[counter])), self.white, 14)
                    cv2.line(img, (int(self.width*0.1), int(self.height)),(int(self.x_points[counter]), int(self.y_points[counter])), self.white, 14)
                    cv2.line(img, (int(self.width*0.9), int(self.height)),(int(self.width-self.x_points[counter]), int(self.y_points[counter])), self.white, 14)                
                    counter3 = 0
                else: 
                    counter3 = counter3 + 1
                    cv2.line(self.lane_frame_int, (int(self.width*0.1), int(self.height)),(int(self.x_points[counter]), int(self.y_points[counter])), self.white, 14)
                    cv2.line(self.lane_frame_int, (int(self.width*0.9), int(self.height)),(int(self.width-self.x_points[counter]), int(self.y_points[counter])), self.white, 14)
                    cv2.line(img, (int(self.width*0.1), int(self.height)),(int(self.x_points[counter-counter3]), int(self.y_points[counter-counter3])), self.white, 14)
                    cv2.line(img, (int(self.width*0.9), int(self.height)),(int(self.width-self.x_points[counter-counter3]), int(self.y_points[counter-counter3])), self.white, 14)
            endD = time.time()
            
            cv2.imshow('t',self.cam_frame)
            cv2.waitKey(1)
            
            cv2.imshow('harris',img)
            cv2.waitKey(1)
            
            endFlag = 130/speed

            if endD-startD > endFlag:
                self.finished=True
