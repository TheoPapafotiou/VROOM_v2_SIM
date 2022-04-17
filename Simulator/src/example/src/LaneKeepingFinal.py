import traceback
from turtle import color
import cv2
import numpy as np
import math
import time

"""
This class implements the lane keeping algorithm by calculating angles from the detected lines slopes.
"""

class LaneKeeping:

    def __init__(self, width, height, max_angle):

        self.width = width
        self.height = height
        self.max_angle = max_angle
        wT, hT1, hT2 =  0.1 * self.width, 0.7 * self.height, 0.6 * self.height
        wB, hB1, hB2 = 0.0 * self.width, 0.9 * self.height, 0.8 * self.height
        self.src_points = {
            '0' : np.float32([[wT, hT1], [width - wT, hT1], [wB, hB1], [width - wB, hB1]]),
            '1' : np.float32([[wT, hT2], [width - wT, hT2], [wB, hB2], [width - wB, hB2]])
        }
        self.warp_matrix = None
        self.angle = 0.0
        self.last_angle = 0.0

        self.last_time = 0
        self.Kp = 0#0.1  # .06
        self.Kd = 0#0.02# .01

        # Rolling average parameters
        self.median_matrix = list()
        self.median_constant = 4

        # Canny Image parameters
        self.cannyHigh = 200
        self.cannyLow = 100

        #HoughLines parameters 
        self.rho = 1  # distance precision in pixel, i.e. 1 pixel
        self.phi = np.pi / 180  # angular precision in radian, i.e. 1 degree
        self.threshold = 60  # minimal of votes

        # Lanes Slope limits 
        self.YLimit = 4
        self.XLimit = 4
        self.MidBoundary = self.width * 0.5
        self.lines_threshold = 5

        # Adaptive Threshold parameters
        self.adaptiveBlockSize = 39 # odd number!
        self.adaptiveC = -20

        # Lanes parameters
        self.mid_factor = 0.5
        self.windows_number = 12                                            # Number of sliding windows (to scan the whole image)
        self.window_width = 40                                              # Width of the windows (maybe dynamic??)
        self.window_height = int(self.height / float(self.windows_number))  # Window Height
        self.minpix = 10                                                    # Min number of pixels needed to recenter the window
        self.min_lane_pts = 1000                                             # Min number of eligible pixels needed to encountered as a lane line
        self.edge_case_thres = self.width // 5
        margin_x = 200
        self.setBestRight = self.width/2 + margin_x
        self.setBestLeft = self.width/2 - margin_x
        
    def warp_image(self, frame, src_points):
        """Warps the given frame projecting the given source points to the distance points
        The function takes the src_points in the order of ([LeftTop], [RightTop], [LeftBottom], [RightBottom]) 
        and projects them to the corners of the image. 
        Parameters
        ----------
        frame : array
            The frame to be warped
        src_points : array
            The source points we want to project
        Returns
        -------
        array
            The warped frame
        """

        # Destination points for warping
        dst_points = np.float32([[0, 0], [self.width, 0], [0, self.height], [self.width, self.height]])
        
        # Warp frame
        self.warp_matrix = cv2.getPerspectiveTransform(src_points, dst_points)
        warped_frame = cv2.warpPerspective(frame, self.warp_matrix, (self.width, self.height))

        return warped_frame

    def calculate_slope(self, lines, frame):
        out = np.dstack((frame, frame, frame)) * 255
        LeftFit = []
        RightFit = []
        xl = []
        yl = []
        xr = []
        yr = []
        xm = []
        ym = []

        if lines is None:
            return None, None, None

        total_lines = len(lines) 
        for line in lines:
            x1, y1, x2, y2 = line[0]
            if (np.abs(y2 - y1) < self.YLimit) or (np.abs(x2 - x1 < self.XLimit)):
                continue
            elif (x1 < self.MidBoundary and x2 < self.MidBoundary):
                xl.append(x1)
                yl.append(y1)
                xl.append(x2)
                yl.append(y2)
                pts = np.array([[x1, y1], [x2, y2]], np.int32)
                cv2.polylines(out, [pts], True, (0, 255, 0), 4)

            elif (x1 > self.MidBoundary and x2 > self.MidBoundary):
                xr.append(x2)
                yr.append(y2)
                xr.append(x1)
                yr.append(y1)
                pts = np.array([[x1, y1], [x2, y2]], np.int32)
                cv2.polylines(out, [pts], True, (255, 0, 0), 4)

            else:
                xm.append(x2)
                ym.append(y2)
                xm.append(x1)
                ym.append(y1)
                pts = np.array([[x1, y1], [x2, y2]], np.int32)
                cv2.polylines(out, [pts], True, (255, 0, 255), 4)

        yr = np.array(yr)
        xr = np.array(xr)
        yl = np.array(yl)
        xl = np.array(xl)
        ym = np.array(ym)
        xm = np.array(xm)

        cv2.imshow('Slope_img', out)

        if len(xm) > total_lines / 3:
            xl = []
            yl = []
            xr = []
            xl = []

        SlopeL = None
        SlopeR = None
        SlopeM = None
        if (np.count_nonzero(xl) != 0) and (np.count_nonzero(yl) != 0):  
            SlopeL, MiddleL = np.polyfit(yl, xl, 1)
            LeftFit.append((SlopeL, MiddleL))
        if (np.count_nonzero(xr) != 0) and (np.count_nonzero(yr) != 0):
            SlopeR, MiddleR = np.polyfit(yr, xr, 1)
            RightFit.append((SlopeR, MiddleR))
        if (np.count_nonzero(xm) != 0) and (np.count_nonzero(ym) != 0):
            SlopeM, MiddleM = np.polyfit(ym, xm, 1)
            RightFit.append((SlopeM, MiddleM))

        return SlopeL, SlopeR, SlopeM

    def extract_factors(self, slopeL, slopeR, slopeM):
        if slopeM is not None and slopeM >= 0.6:
            self.mid_factor = 0.3
        elif slopeM is not None and slopeM <= -0.5:
            self.mid_factor = 1 - 0.3
        else:
            self.mid_factor = 0.5

    def polyfit_sliding_window(self, frame):
        """Performs the lane detection on the given frame
        1) The function takes the frame and calculates the sum of intensities from each column (histogram).
        2) It detects and keeps all non-zero values of the frame.
        3) Then, it sets the areas of interest where each type of line (left, right) is detected. (!maybe dynamic!)
        4) From each area it detects the corresponding maximum value of the area's histogram (part of the initial histogram) (!maybe not just nonzeros!)
        5) Then the function "scans" the image from bottom to top using windows centered initially on the aforementioned maximum value index (for both right and left line). 
            i) It gathers the indexes of the non-zero values within each window.
            ii) The center of each window is moved, in ever iteration, to the position where the mean value of those gathered non-zero values is located.
        6) It searches for edge cases when one line is perceived as both left and right at the same time and keeps the one with the more non-zero values.
        7) Finally, it fits a 2nd order polynomial for each lane line pixels 
        Parameters
        ----------
        frame : array
            The frame on which the lanes are detected
        Returns
        -------
        array
            The left polynomial fit
        array 
            The right polynomial fit
        array
            The output frame with the drawn lines
        """

        ## === Sum Intensities of each column on the frame ===
        out = np.dstack((frame, frame, frame)) * 255

        histogram = np.sum(frame, axis=0)

        if histogram.max() == 0:
            return None, None, out

        ## === Find all pixels that are lane lines on the picture ===
        nonzero = frame.nonzero()
        nonzerox = np.array(nonzero[1])
        nonzeroy = np.array(nonzero[0])

        ## === Set areas of interest ===
        midpoint = int(self.width * self.mid_factor)

        hist_left = histogram[:midpoint]

        hist_right_pre = histogram[midpoint:]
        hist_right = hist_right_pre[::-1]

        if not np.any(hist_left) and not np.any(hist_right):
            print('Empty left and right hist')
            return None, None, out

        ## === Detect maximum value of each area ===
        max_left = np.argmax(hist_left)
        max_right = np.argmax(hist_right)
        
        leftx_base = max_left
        rightx_base = midpoint + len(hist_right) - max_right - 1

        # Current position, updated for each window
        leftx_current = leftx_base
        rightx_current = rightx_base

        ## === Scan the frame from bottom to top ===
        left_lane_inds = []
        right_lane_inds = []

        for window in range(self.windows_number):
            win_y_low = self.height - (1 + window) * self.window_height
            win_y_high = self.height - window * self.window_height

            ## ====== LEFT ======
            win_xleft_low = leftx_current - self.window_width/2
            win_xleft_high = leftx_current + self.window_width/2

            good_left_inds = ((nonzeroy >= win_y_low) & (nonzeroy <= win_y_high)
                                & (nonzerox >= win_xleft_low) & (nonzerox <= win_xleft_high)).nonzero()[0]
            left_lane_inds.append(good_left_inds)

            ## ====== RIGHT ======
            win_xright_low = rightx_current - self.window_width/2
            win_xright_high = rightx_current + self.window_width/2

            # Draw windows for visualisation
            # cv2.rectangle(out, (win_xleft_low, win_y_low), (win_xleft_high, win_y_high), (0, 0, 255), 2)
            # cv2.rectangle(out, (win_xright_low, win_y_low), (win_xright_high, win_y_high), (0, 255, 0), 2)

            good_right_inds = ((nonzeroy >= win_y_low) & (nonzeroy <= win_y_high)
                                & (nonzerox >= win_xright_low) & (nonzerox <= win_xright_high)).nonzero()[0]
            right_lane_inds.append(good_right_inds)
            
            if len(good_left_inds) > self.minpix:
                leftx_current = int(np.mean(nonzerox[good_left_inds]))

            if len(good_right_inds) > self.minpix:
                rightx_current = int(np.mean(nonzerox[good_right_inds]))

        ## === Edge case when one line is perceived as both left and right lane ===
        if abs(rightx_base - leftx_base) < self.edge_case_thres:
            if len(left_lane_inds) > len(right_lane_inds):
                right_lane_inds = []
            elif len(left_lane_inds) < len(right_lane_inds):
                left_lane_inds = []
            else:
                right_lane_inds = []
                left_lane_inds = []

        ## === Extract pixel positions for the left and right lane lines ===
        if len(left_lane_inds) > 0:
            left_lane_inds = np.concatenate(left_lane_inds)
        if len(right_lane_inds) > 0:
            right_lane_inds = np.concatenate(right_lane_inds)

        leftx = nonzerox[left_lane_inds]
        lefty = nonzeroy[left_lane_inds]
        rightx = nonzerox[right_lane_inds]
        righty = nonzeroy[right_lane_inds]

        ## === Fit a 2nd order polynomial for each lane line pixels ===
        left_fit, right_fit = None, None

        if len(leftx) >= self.min_lane_pts:  # and histogram[leftx_base] != 0:
            left_fit = np.polyfit(lefty, leftx, 2)

        if len(rightx) >= self.min_lane_pts:  # and histogram[rightx_base] != 0:
            right_fit = np.polyfit(righty, rightx, 2)

        # Color the detected pixels for each lane line
        out[nonzeroy[left_lane_inds], nonzerox[left_lane_inds]] = [255, 0, 0]
        out[nonzeroy[right_lane_inds], nonzerox[right_lane_inds]] = [255, 0, 255]

        return left_fit, right_fit, out

    def get_poly_points(self, left_fit, right_fit, polynomial=2):
        """Warps the given frame projecting the given source points to the distance points
        The function takes the src_points in the order of ([LeftTop], [RightTop], [LeftBottom], [RightBottom]) 
        and projects them to the corners of the image. 
        Parameters
        ----------
        frame : array
            The frame to be warped
        src_points : array
            The source points we want to project
        Returns
        -------
        array
            The warped frame
        """
        plot_y = np.linspace(0, self.height - 1, self.height)

        if left_fit is not None and right_fit is not None:
            plot_xleft = left_fit[0] * plot_y ** 2 + left_fit[1] * plot_y + left_fit[2]
            plot_xright = right_fit[0] * plot_y ** 2 + right_fit[1] * plot_y + right_fit[2]
            
            plot_yright = plot_y
            plot_yleft = plot_y

            return plot_xleft.astype(np.int), plot_yleft.astype(np.int), \
                        plot_xright.astype(np.int), plot_yright.astype(np.int)

    def get_error(self, left_x, right_x):

        factor = int(round(0.5 * self.height))

        if left_x is not None and right_x is not None:
            sample_right_x = right_x[factor:]
            sample_left_x = left_x[factor:]

            sample_x = np.array((sample_right_x + sample_left_x) / 2.0)
            if len(sample_x) != 0:
                weighted_mean = self.weighted_average(sample_x)

            center = int(self.width / 2.0)
            error = weighted_mean - center

        elif left_x is None:
            x1 = right_x[0] * self.height ** 2 + right_x[1] * self.height + right_x[2]
            x2 = right_x[2]
            error_init = x2 - x1

            diff = (x1+x2)/2 - self.setBestRight
            error = x2 - x1 + diff

            print('Before Right: ', 90 - math.degrees(math.atan2(self.height, error_init)))
            print('After Right: ', 90 - math.degrees(math.atan2(self.height, error)), 'Diff: ', diff)

        elif right_x is None:
            x1 = left_x[0] * self.height ** 2 + left_x[1] * self.height + left_x[2]
            x2 = left_x[2]
            error_init = x2 - x1
            
            diff = (x1+x2)/2 - self.setBestLeft
            error = x2 - x1 + diff
            
            print('Before Left: ', 90 - math.degrees(math.atan2(self.height, error_init)))
            print('After Left: ', 90 - math.degrees(math.atan2(self.height, error)), 'Diff: ', diff)

        return error

    def weighted_average(self, num_list):

        weights = [*range(0, len(num_list))]
        mean = np.average(num_list, weights=weights)

        return mean

    def image_preprocessing(self, frame, src_points):
        
        frame = self.warp_image(frame, src_points)
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        thresholded = cv2.adaptiveThreshold(gray, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, \
                                    cv2.THRESH_BINARY, self.adaptiveBlockSize, self.adaptiveC)          
        edged = cv2.Canny(thresholded, self.cannyLow, self.cannyHigh)

        return gray, edged, thresholded

    def lane_detection(self, thresholded):
    
        left, right, out = self.polyfit_sliding_window(thresholded)
        return left, right, out

    def angle_calculation(self, left, right):

        if left is not None and right is not None:
            # print("BOTH LANES")

            left_x, left_y, right_x, right_y = self.get_poly_points(left, right)

            error = self.get_error(left_x, right_x)
            angle = 90 - math.degrees(math.atan2(self.height, error))

        elif right is None and left is not None:
            # print("LEFT LANE")

            error = self.get_error(left, None)
            angle = 90 - math.degrees(math.atan2(self.height, error))

        elif left is None and right is not None:
            # print("RIGHT LANE")

            error = self.get_error(None, right)
            angle = 90 - math.degrees(math.atan2(self.height, error))

        else:
            angle = self.angle
            error = 0
            print("No lanes found")

        return angle, error

    def fix_angle(self):

        dt = time.time() - self.last_time

        diff = self.angle - self.last_angle

        derivative = self.Kd * (diff / dt)
        proportional = self.Kp * self.angle
        PD = int(self.angle + derivative + proportional)

        self.median_matrix.insert(0, PD)
        if len(self.median_matrix) == self.median_constant:
            median = np.average(self.median_matrix)
            PD = median
            self.median_matrix.pop()

        self.angle = PD
        self.last_angle = self.angle
        self.last_time = time.time()
        
    def lane_keeping_pipeline(self, frame):

        try:
            angles = [0 for i in range(2)]
            errors = [0 for i in range(2)]

            self.mid_factor = 0.5

            start = time.time()
            for repeat in range(2):

                gray, edged_pre, thresholded = self.image_preprocessing(frame, self.src_points[str(repeat)])

                ## === Calculate approximate slope ===
                lines = cv2.HoughLinesP(edged_pre, self.rho, self.phi, self.threshold,
                                np.array([]), minLineLength=8, maxLineGap=4)

                left_slope, right_slope, medium_slope = self.calculate_slope(lines, edged_pre)
                self.extract_factors(left_slope, right_slope, medium_slope)

                ## === Angle calculation ===
                left, right, out = self.lane_detection(thresholded)#(edged_pre)#
                angles[repeat], errors[repeat] = self.angle_calculation(left, right)

            self.angle = np.average(angles, weights=[3, 1])
            
            # print(self.angle)
            self.fix_angle()

            output_image = out
            cv2.line(output_image, (int(self.mid_factor * self.width), 0), (int(self.mid_factor * self.width), self.height), color=(255, 255, 255), thickness=2)
            cv2.line(output_image, (int(self.setBestLeft), 0), (int(self.setBestLeft), self.height), color=(255, 100, 100), thickness=2)
            cv2.line(output_image, (int(self.setBestRight), 0), (int(self.setBestRight), self.height), color=(100, 100, 255), thickness=2)
            cv2.circle(output_image, (int(errors[1]), self.height//2), radius=5, color=(0, 255, 0), thickness=-1)

            return self.angle, output_image
        except Exception as e:
            print(e)
            traceback.print_exc()