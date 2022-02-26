import math
import time 
import cv2
import numpy as np

"""
This class implements the lane keeping algorithm by calculating angles from the detected lines.
Input: Camera Input
Output: Steering Angle
To-Do: wrap // why change lane // 
"""


class LaneKeeping:

    def __init__(self, width, height):
        self.angle = 0.0
        self.LastAngle = 0.0
        self.AngleBuffer = []
        self.MovingSize = 4
        self.height = height
        self.width = width
        self.YLimit = 4
        self.XLimit = 3
        self.RightBoundary = self.width * 3 / 5
        self.LeftBoundary = self.width * 2 / 5
        #image processing parameters
        self.ThresholdHigh = 150
        self.ThresholdLow = 350
        self.KSize = (5, 5)
        self.BorderType = 0
        #HoughLinesP() parameters
        self.rho = 2
        self.theta = np.pi / 180
        self.thershold = 20
        self.minLineLength = 3
        self.maxLineGap = 13
        # self.framecount = 1
        # ====TEST CODE====
        self.last_time = 0
        self.last_error = 0
        self.last_PD = 0
        self.Kp = 0.1  # .06
        self.Kd = 0.02 # .01

        # Rolling average parameters
        self.median_matrix = list()
        self.median_constant = 0
        self.median = 0.0


    def canny(self, image):
        gray = cv2.cvtColor(image, cv2.COLOR_RGB2GRAY)
        blur = cv2.GaussianBlur(gray, self.KSize, self.BorderType)
        canny = cv2.Canny(blur, self.ThresholdHigh, self.ThresholdLow)
        return canny

    def masked_region(self, image):  
        polygons = np.array([
             [(self.width, self.height * 4 / 5),(self.width * 5 / 6, self.height * 3 / 5), (self.width / 6, self.height * 3 / 5), 
             (0, self.height * 4 / 5)]
        ])
        mask = np.zeros_like(image)
        cv2.fillPoly(mask, np.int32([polygons]), 255)
        MaskedImage = cv2.bitwise_and(image, mask)
        return MaskedImage

    def warp(self, image, height, width):
        # Destination points for warping
        dstPoints = np.float32(
            [[self.width / 7, height * 55 / 100], [0, self.height * 4 / 5], [self.width, self.height * 4 / 5],
             [self.width * 6 / 7, self.height * 55 / 100]])
        srcPoints = np.float32(
            [[self.width / 6, self.height * 3 / 5], [0, self.height * 4 / 5], [self.width, self.height * 4 / 5],
             [self.width * 5 / 6, self.height * 55 / 100]])

        WarpMatrix = cv2.getPerspectiveTransform(srcPoints, dstPoints)
        WarpedFrame = cv2.warpPerspective(image, WarpMatrix, (width, height))
        return WarpedFrame

    def make_lines(self, lines):
        LeftFit = []
        RightFit = []
        xl = []
        yl = []
        xr = []
        yr = []
        if lines is None:
            return np.array(LeftFit), np.array(RightFit)
        for line in lines:
            x1, y1, x2, y2 = line[0]
            if (np.abs(y2 - y1) < self.YLimit) or (np.abs(x2 - x1 < self.XLimit)):
                continue
            elif (x1 < self.LeftBoundary):
                xl.append(x1)
                yl.append(y1)
                xl.append(x2)
                yl.append(y2)
                pts = np.array([[x1, y1], [x2, y2]], np.int32)
                cv2.polylines(self.frame, [pts], True, (0, 255, 0), 4)

            elif (x1 > self.RightBoundary):
                xr.append(x2)
                yr.append(y2)
                xr.append(x1)
                yr.append(y1)
                pts = np.array([[x1, y1], [x2, y2]], np.int32)
                cv2.polylines(self.frame, [pts], True, (255, 0, 0), 4)

        cv2.imshow('lines', self.frame)

        yr = np.array(yr)
        xr = np.array(xr)
        yl = np.array(yl)
        xl = np.array(xl)

        if (np.count_nonzero(xl) != 0) and (np.count_nonzero(yl) != 0):  
            SlopeL, MiddleL = np.polyfit(yl, xl, 1)
            LeftFit.append((SlopeL, MiddleL))
        if (np.count_nonzero(xr) != 0) and (np.count_nonzero(yr) != 0):
            SlopeR, MiddleR = np.polyfit(yr, xr, 1)
            RightFit.append((SlopeR, MiddleR))

        # print ('right y,x: ', yr, xr)
        # print ('left y,x: ', yl, xl)
        return np.array(LeftFit), np.array(RightFit)

    def lanes_pipeline(self, cap):
        start = time.time()
        self.frame = cap
        # self.framecount=self.framecount+1
        # if self.framecount%2==1:
        #     return self.angle

        CannyImage = self.canny(self.frame)
        # WrappedImage=self.warp(CannyImage,self.height,self.width)
        MaskedImage = self.masked_region(CannyImage)

        lines = cv2.HoughLinesP(MaskedImage, rho=self.rho, theta=self.theta, threshold=self.thershold, lines=np.array([]),
                                minLineLength=self.minLineLength, maxLineGap=self.maxLineGap)
        self.left, self.right = self.make_lines(lines)

        SteeringAngle = self.lane_keeping_pipeline()
        end = time.time()
        # print('run time: ', end - start)
        return SteeringAngle

    def get_poly_points(self, LeftFit, RightFit):

        PlotY = np.linspace(0, self.height - 1, self.height)
        al, bl = LeftFit[0] # <-- same value for both params!!
        ar, br = RightFit[0] # <-- same value for both params!!
        PlotXLeft = al * PlotY + bl
        PlotXRight = ar * PlotY + br

        return PlotXLeft.astype(np.int), PlotXRight.astype(np.int)

    def get_error(self, LeftX, RightX):

        factor = int(round(0.5 * self.height))
        weighted_mean = 0
        SampleRightX = RightX[factor:]
        SampleLeftX = LeftX[factor:]

        sample_x = np.array((SampleRightX + SampleLeftX) / 2.0)
        if len(sample_x) != 0:
            weighted_mean = self.weighted_average(sample_x)

        error = weighted_mean - int(self.width / 2.0)

        return error

    def weighted_average(self, NumList):

        # CHECK WEIGHTS, SHOULD BE FLIPPED?
        weights = [*range(0, len(NumList))]
        # mean = int(round(sum([NumList[i] * weights[i] for i in range(count)]) / sum(weights), 2))
        # mean = int(round(sum([NumList[i]*1 for i in range(count)])/count,2))
        return np.average(NumList, weights=weights)

    def lane_keeping_pipeline(self):

        if np.count_nonzero(self.left) != 0 and np.count_nonzero(self.right) != 0:
            print("BOTH LANES")

            LeftX, RightX = self.get_poly_points(self.left, self.right)
            error = self.get_error(LeftX, RightX)
            self.angle = 90 - math.degrees(math.atan2(self.height, error))

        elif np.count_nonzero(self.left) != 0 and np.count_nonzero(self.right) == 0:
            print("LEFT LANE")

            a, b = self.left[0]
            x1 = a * self.height + b
            x2 = b

            dx = x2 - x1
            dy = self.height
            self.angle = - 45 + 90 - math.degrees(math.atan2(dy, dx))

        elif np.count_nonzero(self.left) == 0 and np.count_nonzero(self.right) != 0:
            print("RIGHT LANE")

            a, b = self.right[0]
            x1 = a * self.height + b
            x2 = b

            dx = x2 - x1
            dy = self.height
            self.angle = 45 + 90 - math.degrees(math.atan2(dy, dx))

        else:
            print("No lanes found")

        # ===========TEST CODE=========
        now = time.time()
        dt = now - self.last_time

        error1 = self.angle

        derivative = self.Kd * (error1 - self.last_error) / dt
        proportional = self.Kp * error1
        PD = int(self.angle + derivative + proportional)

        self.last_error = error1
        self.last_time = time.time()

        # self.median_matrix.insert(0, PD)
        # if len(self.median_matrix) == self.median_constant:
        #     self.median = np.average(self.median_matrix)
        #     PD = self.median
        #     self.median = 0.0
        #     self.median_matrix.pop()

        print('derivative',derivative)
        print('proportional',proportional)
        print('PD',PD)
        # ==========END OF TEST CODE=========
        
        if np.abs(self.LastAngle - self.angle) < 0.5:
            self.angle = self.LastAngle
        else:
            if self.angle > 0:
                self.angle = min(23, self.angle)
            else:
                self.angle = max(-23, self.angle)

            weights = [*range(1, self.MovingSize + 1)]

            if len(self.AngleBuffer) >= self.MovingSize:
                self.AngleBuffer.pop(0)
                self.AngleBuffer.append(self.angle)
            else:
                self.AngleBuffer.append(self.angle)
                weights = [*range(1, len(self.AngleBuffer) + 1)]

            self.angle = np.average(self.AngleBuffer, weights=weights)
            self.LastAngle = self.angle

        print('angle ', self.angle, '\n')

        return self.angle
