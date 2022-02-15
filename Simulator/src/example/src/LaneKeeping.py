import math
import datetime 
import cv2
import numpy as np

"""
This class implements the lane keeping algorithm by calculating angles from the detected lines.
Input: Camera Input
Output: Steering Angle
To-Do: wrap // why change lane // 
"""


class LaneKeeping:

    def __init__(self):
        self.angle = 0.0
        self.last_angle = 0.0
        self.angle_buffer = []
        self.moving_size = 4
        # self.framecount = 1

    def canny(self, image):
        gray = cv2.cvtColor(image, cv2.COLOR_RGB2GRAY)
        blur = cv2.GaussianBlur(gray, (5, 5), 0)
        canny = cv2.Canny(blur, 150, 350)
        return canny

    def masked_region(self, image):  
        polygons = np.array([
            [(self.width / 6, self.height * 3 / 5), (0, self.height * 4 / 5), (self.width, self.height * 4 / 5),
             (self.width * 5 / 6, self.height * 3 / 5)]
        ])
        mask = np.zeros_like(image)
        cv2.fillPoly(mask, np.int32([polygons]), 255)
        masked_image = cv2.bitwise_and(image, mask)
        return masked_image

    def warp(self, image, height, width):
        # Destination points for warping
        dst_points = np.float32(
            [[self.width / 7, height * 55 / 100], [0, self.height * 4 / 5], [self.width, self.height * 4 / 5],
             [self.width * 6 / 7, self.height * 55 / 100]])
        src_points = np.float32(
            [[self.width / 6, self.height * 3 / 5], [0, self.height * 4 / 5], [self.width, self.height * 4 / 5],
             [self.width * 5 / 6, self.height * 55 / 100]])

        warp_matrix = cv2.getPerspectiveTransform(src_points, dst_points)
        warped_frame = cv2.warpPerspective(image, warp_matrix, (width, height))
        return warped_frame

    def make_lines(self, lines):
        left_fit = []
        right_fit = []
        xl = []
        yl = []
        xr = []
        yr = []
        if lines is None:
            return np.array(left_fit), np.array(right_fit)
        for line in lines:
            x1, y1, x2, y2 = line[0]
            if (np.abs(y2 - y1) < 4) or (np.abs(x2 - x1 < 3)):
                continue
            elif (x1 < self.width * 2 / 5):
                xl.append(x1)
                yl.append(y1)
                xl.append(x2)
                yl.append(y2)
                pts = np.array([[x1, y1], [x2, y2]], np.int32)
                cv2.polylines(self.frame, [pts], True, (0, 255, 0), 4)

            elif (x1 > self.width * 3 / 5):
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
            slope_l, middle_l = np.polyfit(yl, xl, 1)
            left_fit.append((slope_l, middle_l))
        if (np.count_nonzero(xr) != 0) and (np.count_nonzero(yr) != 0):
            slope_r, middle_r = np.polyfit(yr, xr, 1)
            right_fit.append((slope_r, middle_r))

        # print ('right y,x: ', yr, xr)
        # print ('left y,x: ', yl, xl)
        return np.array(left_fit), np.array(right_fit)

    def lanes_pipeline(self, cap):
        start = datetime.datetime.now()
        self.frame = cap
        # self.framecount=self.framecount+1
        # if self.framecount%2==1:
        #     return self.angle

        self.height = self.frame.shape[0]
        self.width = self.frame.shape[1]
        canny_image = self.canny(self.frame)
        # wrapped_image=self.warp(canny_image,self.height,self.width)
        masked_image = self.masked_region(canny_image)

        lines = cv2.HoughLinesP(masked_image, rho=2, theta=np.pi / 180, threshold=20, lines=np.array([]),
                                minLineLength=3, maxLineGap=13)
        self.left, self.right = self.make_lines(lines)

        steering_angle = self.lane_keeping_pipeline()
        end = datetime.datetime.now()
        print('run time: ', end - start)
        return steering_angle

    def get_poly_points(self, left_fit, right_fit):

        plot_y = np.linspace(0, self.height - 1, self.height)
        al, bl = left_fit[0]
        ar, br = right_fit[0]
        plot_xleft = al * plot_y + bl
        plot_xright = ar * plot_y + br

        return plot_xleft.astype(np.int), plot_xright.astype(np.int)

    def get_error(self, left_x, right_x):

        factor = int(round(0.5 * self.height))
        weighted_mean = 0
        sample_right_x = right_x[factor:]
        sample_left_x = left_x[factor:]

        sample_x = np.array((sample_right_x + sample_left_x) / 2.0)
        if len(sample_x) != 0:
            weighted_mean = self.weighted_average(sample_x)

        error = weighted_mean - int(self.width / 2.0)

        return error

    def weighted_average(self, num_list):

        # CHECK WEIGHTS, SHOULD BE FLIPPED?
        weights = [*range(0, len(num_list))]
        # mean = int(round(sum([num_list[i] * weights[i] for i in range(count)]) / sum(weights), 2))
        # mean = int(round(sum([num_list[i]*1 for i in range(count)])/count,2))
        return np.average(num_list, weights=weights)

    def lane_keeping_pipeline(self):

        if np.count_nonzero(self.left) != 0 and np.count_nonzero(self.right) != 0:
            print("BOTH LANES")

            left_x, right_x = self.get_poly_points(self.left, self.right)
            error = self.get_error(left_x, right_x)
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

        if np.abs(self.last_angle - self.angle) < 0.5:
            self.angle = self.last_angle
        else:
            if self.angle > 0:
                self.angle = min(23, self.angle)
            else:
                self.angle = max(-23, self.angle)

            weights = [*range(1, self.moving_size + 1)]

            if len(self.angle_buffer) >= self.moving_size:
                self.angle_buffer.pop(0)
                self.angle_buffer.append(self.angle)
            else:
                self.angle_buffer.append(self.angle)
                weights = [*range(1, len(self.angle_buffer) + 1)]

            self.angle = np.average(self.angle_buffer, weights=weights)

        print('angle ', self.angle, '\n')

        return self.angle
