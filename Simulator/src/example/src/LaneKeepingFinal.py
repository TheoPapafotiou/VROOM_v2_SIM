import traceback
import cv2
import numpy as np
import math
import time

"""
This class implements the lane keeping algorithm by calculating angles from the detected lines slopes.
"""

class LaneKeeping:

    def __init__(self, width, height, version):

        self.version = version
        self.width = width
        self.height = height
        wT, hT1, hT2 =  0.1 * self.width, 0.7 * self.height, 0.5 * self.height
        wB, hB1, hB2 = 0.0 * self.width, 0.9 * self.height, 0.6 * self.height
        self.src_points = {
            '0' : np.float32([[wT, hT1], [width - wT, hT1], [wB, hB1], [width - wB, hB1]]),
            '1' : np.float32([[wT, hT2], [width - wT, hT2], [wB, hB2], [width - wB, hB2]])
        }
        self.warp_matrix = None
        self.angle = 0.0
        self.last_angle = 0.0

        self.last_time = 0
        self.last_PD = 0
        self.Kp = 0#0.1  # .06
        self.Kd = 0#0.02# .01

        # Rolling average parameters
        self.median_matrix = list()
        self.median_constant = 4

        #HoughLines parameters 
        self.rho = 1  # distance precision in pixel, i.e. 1 pixel
        self.phi = np.pi / 180  # angular precision in radian, i.e. 1 degree
        self.threshold = 60  # minimal of votes

        # Blur Image parameters
        self.KSize = (5, 5)
        self.BorderType = 0

        # Canny Image parameters
        self.cannyHigh = 200
        self.cannyLow = 100

        # Adaptive Threshold parameters
        self.adaptiveBlockSize = 49 # odd number!
        self.adaptiveC = -20

        # Lanes limits 
        self.YLimit = 4
        self.XLimit = 3
        self.RightBoundary = self.width * 3 / 5
        self.LeftBoundary = self.width * 2 / 5

        # Lanes parameters
        self.left_factor = 1
        self.right_factor = 1
        self.windows_number = 12                                            # Number of sliding windows (to scan the whole image)
        self.window_width = 60                                              # Width of the windows (maybe dynamic??)
        self.window_height = int(self.height / float(self.windows_number))  # Window Height
        self.minpix = 10                                                    # Min number of pixels needed to recenter the window
        self.min_lane_pts = 500                                             # Min number of eligible pixels needed to encountered as a lane line
        self.edge_case_thres = self.width // 5
        margin_x = 150
        self.setBestRight = self.width/2 + margin_x
        self.setBestLeft = self.width/2 - margin_x

        self.case = -1
        
    def warp_image(self, frame, src_points):

        # Destination points for warping
        dst_points = np.float32([[0, 0], [self.width, 0], [0, self.height], [self.width, self.height]])
        
        # Warp frame
        self.warp_matrix = cv2.getPerspectiveTransform(src_points, dst_points)
        warped_frame = cv2.warpPerspective(frame, self.warp_matrix, (self.width, self.height))

        return warped_frame

    def make_lanes(self, lines, frame):
        out = np.dstack((frame, frame, frame)) * 255
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
                cv2.polylines(out, [pts], True, (0, 255, 0), 4)

            elif (x1 > self.RightBoundary):
                xr.append(x2)
                yr.append(y2)
                xr.append(x1)
                yr.append(y1)
                pts = np.array([[x1, y1], [x2, y2]], np.int32)
                cv2.polylines(out, [pts], True, (255, 0, 0), 4)

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

        return np.array(LeftFit), np.array(RightFit)

    def polyfit_sliding_window(self, frame):

        ## === Compute peaks in the two frame halves, to get an idea of lane start positions ===
        histogram = None

        # cutoffs = [int(self.height / 2.0), 0]
        cutoffs = [0, 0]
        out = np.dstack((frame, frame, frame)) * 255

        for cutoff in cutoffs:
            histogram = np.sum(frame[cutoff:, :], axis=0) #!!! maybe in upper half only the second time !!!

            if histogram.max() > 0:
                break

        if histogram.max() == 0:
            return None, None, out

        ## === Calculate peaks of histogram ===
        midpoint = np.int(self.width / 2.0)

        left_mid = int(midpoint * self.left_factor)
        hist_left = histogram[:left_mid]
        max_left = np.argmax(hist_left)

        right_mid = int(midpoint * self.right_factor)
        hist_right = histogram[right_mid:]
        hist_right = hist_right[::-1]
        max_right = np.argmax(hist_right)

        leftx_base = max_left
        rightx_base = len(hist_right) - max_right - 1 + right_mid

        # Find all pixels that are lane lines on the picture
        nonzero = frame.nonzero()
        nonzerox = np.array(nonzero[1])
        nonzeroy = np.array(nonzero[0])

        # Current position, updated for each window
        leftx_current = leftx_base
        rightx_current = rightx_base

        # Lists for indices of each lane
        left_lane_inds = []
        right_lane_inds = []

        for window in range(self.windows_number):
            # Find window boundaries in x and y axis
            win_y_low = self.height - (1 + window) * self.window_height
            win_y_high = self.height - window * self.window_height

            ## ====== LEFT ======
            win_xleft_low = leftx_current - self.window_width/2
            win_xleft_high = leftx_current + self.window_width/2

            # Draw windows for visualisation
            # print((win_xleft_low, win_y_low), (win_xleft_high, win_y_high))
            # cv2.rectangle(out, (win_xleft_low, win_y_low), (win_xleft_high, win_y_high), (0, 0, 255), 2)

            good_left_inds = ((nonzeroy >= win_y_low) & (nonzeroy <= win_y_high)
                                & (nonzerox >= win_xleft_low) & (nonzerox <= win_xleft_high)).nonzero()[0]
            left_lane_inds.append(good_left_inds)

            ## ====== RIGHT ======
            win_xright_low = rightx_current - self.window_width/2
            win_xright_high = rightx_current + self.window_width/2

            # cv2.rectangle(out, (win_xright_low, win_y_low), (win_xright_high, win_y_high), (0, 255, 0), 2)

            # Identify the nonzero pixels in x and y within the window
            good_right_inds = ((nonzeroy >= win_y_low) & (nonzeroy <= win_y_high)
                                & (nonzerox >= win_xright_low) & (nonzerox <= win_xright_high)).nonzero()[0]
            right_lane_inds.append(good_right_inds)
            
            # If you found > minpix pixels, recenter next window on their mean position
            if len(good_left_inds) > self.minpix:
                leftx_current = int(np.mean(nonzerox[good_left_inds]))

            if len(good_right_inds) > self.minpix:
                rightx_current = int(np.mean(nonzerox[good_right_inds]))

        # Edge case for when one line is perceived as both left and right lane
        if abs(rightx_base - leftx_base) < self.edge_case_thres:
            if len(left_lane_inds) > len(right_lane_inds):
                right_lane_inds = []
            elif len(left_lane_inds) < len(right_lane_inds):
                left_lane_inds = []
            else:
                right_lane_inds = []
                left_lane_inds = []

        if len(left_lane_inds) > 0:
            left_lane_inds = np.concatenate(left_lane_inds)
        if len(right_lane_inds) > 0:
            right_lane_inds = np.concatenate(right_lane_inds)

        # Extract pixel positions for the left and right lane lines
        leftx = nonzerox[left_lane_inds]
        lefty = nonzeroy[left_lane_inds]
        rightx = nonzerox[right_lane_inds]
        righty = nonzeroy[right_lane_inds]

        left_fit, right_fit = None, None

        # Fit a 2nd order polynomial for each lane line pixels
        if len(leftx) >= self.min_lane_pts:  # and histogram[leftx_base] != 0:
            left_fit = np.polyfit(lefty, leftx, 2)

        if len(rightx) >= self.min_lane_pts:  # and histogram[rightx_base] != 0:
            right_fit = np.polyfit(righty, rightx, 2)

        # Color the detected pixels for each lane line
        out[nonzeroy[left_lane_inds], nonzerox[left_lane_inds]] = [255, 0, 0]
        out[nonzeroy[right_lane_inds], nonzerox[right_lane_inds]] = [255, 10, 255]

        return left_fit, right_fit, out

    def get_poly_points(self, left_fit, right_fit, polynomial=2):

        plot_y = np.linspace(0, self.height - 1, self.height)

        if left_fit is not None and right_fit is not None:
            if polynomial == 2:
                plot_xleft = left_fit[0] * plot_y ** 2 + left_fit[1] * plot_y + left_fit[2]
                plot_xright = right_fit[0] * plot_y ** 2 + right_fit[1] * plot_y + right_fit[2]
            else:
                plot_xleft = left_fit[0][0] * plot_y + left_fit[0][1]
                plot_xright = right_fit[0][0] * plot_y + right_fit[0][1]
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

    def plot_points(self, left_x, left_y, right_x, right_y, frame):

        out = frame * 0

        for i in range(len(left_x)):
            cv2.circle(out, (left_x[i], left_y[i]), 10, color=(255, 255, 255), thickness=-1)
            cv2.circle(out, (right_x[i], right_y[i]), 10, color=(255, 255, 255), thickness=-1)

        return out

    def image_preprocessing(self, frame, src_points, threshold=1, warp=1):
        if warp:
            frame = self.warp_image(frame, src_points)
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        if threshold:
            thresholded = cv2.adaptiveThreshold(gray, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, \
                                        cv2.THRESH_BINARY, self.adaptiveBlockSize, self.adaptiveC)          
            edged = cv2.Canny(thresholded, self.cannyLow, self.cannyHigh)

            return edged, thresholded
        else:
            blured = cv2.GaussianBlur(gray, self.KSize, self.BorderType)
            edged = cv2.Canny(blured, self.cannyLow, self.cannyHigh)

            return edged, None

    def lane_detection(self, edged, thresholded):
        lines = cv2.HoughLinesP(edged, self.rho, self.phi, self.threshold,
                                np.array([]), minLineLength=8, maxLineGap=4)
    
        if thresholded is not None:
            left, right, out = self.polyfit_sliding_window(thresholded)
            return left, right, out
        else:
            left, right = self.make_lanes(lines, edged)
            return left, right, edged
        

    def angle_calculation(self, left, right):

        if left is not None and right is not None:
            # print("BOTH LANES")

            left_x, left_y, right_x, right_y = self.get_poly_points(left, right, polynomial=2)

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
            print("No lanes found")

        return angle

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
            for repeat in range(1):

                edged, thresholded = self.image_preprocessing(frame, self.src_points[str(repeat)], threshold=1, warp=1)
                left, right, edged = self.lane_detection(edged, thresholded)
                angles[repeat] = self.angle_calculation(left, right)

            self.angle = np.average(angles, weights=[1, 0])
            # print(self.angle)
            print()
            self.fix_angle()

            return self.angle, edged
        except Exception as e:
            print(e)
            traceback.print_exc()