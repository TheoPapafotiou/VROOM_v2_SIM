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
        wT, hT =  0.1 * self.width, 0.6 * self.height
        wB, hB = 0.0 * self.width, 0.7 * self.height
        self.src_points = np.float32([[wT, hT], [width - wT, hT], [wB, hB], [width - wB, hB]])
        self.warp_matrix = None
        self.angle = 0.0
        self.last_angle = 0.0

        self.last_time = 0
        self.last_PD = 0
        self.Kp = 0.1  # .06
        self.Kd = 0.02# .01

        # Rolling average parameters
        self.median_matrix = list()
        self.median_constant = 0
        self.median = 0.0

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
        self.adaptiveBlockSize = 39 # odd number!
        self.adaptiveC = -20

        # Lanes limits 
        self.YLimit = 4
        self.XLimit = 3
        self.RightBoundary = self.width * 3 / 5
        self.LeftBoundary = self.width * 2 / 5

        # Lanes parameters
        self.left_factor = 1
        self.right_factor = 1
        self.margin = 20            # Width of the windows +/- margin
        self.windows_number = 12    # Number of sliding windows (to scan the whole image)
        self.minpix = 10            # Min number of pixels needed to recenter the window
        self.min_lane_pts = 500     # Min number of eligible pixels needed to encountered as a lane line
        self.edge_case_thres = self.width // 5
        
    def warp_image(self, frame):

        # Destination points for warping
        dst_points = np.float32([[0, 0], [self.width, 0], [0, self.height], [self.width, self.height]])
        
        # Warp frame
        self.warp_matrix = cv2.getPerspectiveTransform(self.src_points, dst_points)
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

        cutoffs = [int(self.height / 2.0), 0]
        out = np.dstack((frame, frame, frame)) * 255

        for cutoff in cutoffs:
            histogram = np.sum(frame[cutoff:, :], axis=0)

            if histogram.max() > 0:
                break

        if histogram.max() == 0:
            return np.array([None, None])

        ## === Calculate peaks of histogram ===
        midpoint = np.int(self.width / 2.0)

        left_mid = int(midpoint * self.left_factor)
        max_left = np.argmax(histogram[:left_mid])

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

        window_height = int(self.height / float(self.windows_number)) # Window Height

        for window in range(self.windows_number):
            # Find window boundaries in x and y axis
            win_y_low = self.height - (1 + window) * window_height
            win_y_high = self.height - window * window_height

            ## ====== LEFT ======
            win_xleft_low = leftx_current - self.margin
            win_xleft_high = leftx_current + self.margin

            # Draw windows for visualisation
            cv2.rectangle(out, (win_xleft_low, win_y_low), (win_xleft_high, win_y_high), \
                            (0, 0, 255), 2)

            good_left_inds = ((nonzeroy >= win_y_low) & (nonzeroy <= win_y_high)
                                & (nonzerox >= win_xleft_low) & (nonzerox <= win_xleft_high)).nonzero()[0]
            left_lane_inds.append(good_left_inds)

            ## ====== RIGHT ======
            win_xright_low = rightx_current - self.margin
            win_xright_high = rightx_current + self.margin

            cv2.rectangle(out, (win_xright_low, win_y_low), (win_xright_high, win_y_high), \
                            (0, 255, 0), 2)

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

        return np.array([left_fit, right_fit])

    def get_poly_points(self, left_fit, right_fit, polynomial=2):

        plot_y = np.linspace(0, self.height - 1, self.height)

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

        sample_right_x = right_x[factor:]
        sample_left_x = left_x[factor:]

        sample_x = np.array((sample_right_x + sample_left_x) / 2.0)
        if len(sample_x) != 0:
            weighted_mean = self.weighted_average(sample_x)

        center = int(self.width / 2.0)
        error = weighted_mean - center
        setpoint = weighted_mean

        return error, setpoint

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

    def image_preprocessing(self, frame, threshold=1, warp=1):
        if warp:
            frame = self.warp_image(frame)
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

    def masked_region(self, image):  
        polygons = np.array([
             [(self.width, self.height * 4 / 5),(self.width * 5 / 6, self.height * 3 / 5), (self.width / 6, self.height * 3 / 5), 
             (0, self.height * 4 / 5)]
        ])
        mask = np.zeros_like(image)
        cv2.fillPoly(mask, np.int32([polygons]), 255)
        MaskedImage = cv2.bitwise_and(image, mask)
        return MaskedImage

    def lane_detection(self, edged, thresholded):
        lines = cv2.HoughLinesP(edged, self.rho, self.phi, self.threshold,
                                np.array([]), minLineLength=8, maxLineGap=4)
    
        if thresholded is not None:
            left, right = self.polyfit_sliding_window(thresholded)
        else:
            left, right = self.make_lanes(lines, edged)

        return left, right, edged

    def angle_calculation(self, left, right):

        if self.version == 1:
            if left is not None and right is not None:
                print("BOTH LANES")

                left_x, left_y, right_x, right_y = self.get_poly_points(left, right, polynomial=2)

                error, setpoint = self.get_error(left_x, right_x)

                self.angle = 90 - math.degrees(math.atan2(self.height, error))

            elif right is None and left is not None:
                print("LEFT LANE")

                x1 = left[0] * self.height ** 2 + left[1] * self.height + left[2]
                x2 = left[2]

                dx = x2 - x1
                dy = self.height

                self.angle = 90 - math.degrees(math.atan2(dy, dx))

            elif left is None and right is not None:
                print("RIGHT LANE")
                x1 = right[0] * self.height ** 2 + right[1] * self.height + right[2]
                x2 = right[2]

                dx = x2 - x1
                dy = self.height

                self.angle = 90 - math.degrees(math.atan2(dy, dx))

            else:
                print("No lanes found")

        elif self.version == 2:
            if np.count_nonzero(left) != 0 and np.count_nonzero(right) != 0:
                # print("BOTH LANES")

                left_x, left_y, right_x, right_y = self.get_poly_points(left, right, polynomial=1)
                error, setpoint = self.get_error(left_x, right_x)
                
                self.angle = 90 - math.degrees(math.atan2(self.height, error))

            elif np.count_nonzero(left) != 0 and np.count_nonzero(right) == 0:
                # print("LEFT LANE")

                a, b = left[0]
                x1 = a * self.height + b
                x2 = b

                dx = x2 - x1
                dy = self.height
                
                self.angle = - 45 + 90 - math.degrees(math.atan2(dy, dx))

            elif np.count_nonzero(left) == 0 and np.count_nonzero(right) != 0:
                # print("RIGHT LANE")

                a, b = right[0]
                x1 = a * self.height + b
                x2 = b

                dx = x2 - x1
                dy = self.height
                
                self.angle = 45 + 90 - math.degrees(math.atan2(dy, dx))

            else:
                print("No lanes found")

    def fix_angle(self):

        dt = time.time() - self.last_time

        diff = self.angle - self.last_angle

        derivative = self.Kd * (diff / dt)
        proportional = self.Kp * self.angle
        PD = int(self.angle + derivative + proportional)

        self.median_matrix.insert(0, PD)
        if len(self.median_matrix) == self.median_constant:
            self.median = np.average(self.median_matrix)
            PD = self.median
            self.median = 0.0
            self.median_matrix.pop()

        self.angle = PD
        if self.angle > 0:
            self.angle = min(23, self.angle)
        else:
            self.angle = max(-23, self.angle)

        self.last_angle = self.angle
        self.last_time = time.time()
        
    def lane_keeping_pipeline(self, frame):

        if self.version == 1:
            edged, thresholded = self.image_preprocessing(frame, threshold=1, warp=1)
            left, right, edged = self.lane_detection(edged, thresholded)

        elif self.version == 2:
            edged, thresholded = self.image_preprocessing(frame, threshold=0, warp=0)
            masked = self.masked_region(edged)
            left, right, edged = self.lane_detection(masked, None)

        self.angle_calculation(left, right)
        self.fix_angle()

        cv2.imshow('edged', edged)
        cv2.waitKey(1)

        return self.angle