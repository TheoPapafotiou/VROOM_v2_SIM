import cv2
import numpy as np
import math
import time

from itertools import compress
time
"""
This class implements the lane keeping algorithm by calculating angles from the detected lines slopes.
"""


class LaneKeepingReloaded:
	
	lane_width_px = 478

	def __init__(self, width, height):
		self.width = width
		self.height = height
		# 	wT, hT, wB,hB = 0.2*self.width, 0.55*self.height, 0.1*self.width, 0.7*self.height #0.05*self.width
		wT, hT, wB, hB = 0.1 * self.width, 0.6 * self.height, 0.0 * self.width, 0.7 * self.height
		# wT, hT, wB,hB = 70, 350, 0, 443
		self.src_points = np.float32([[wT, hT], [width - wT, hT], [wB, hB], [width - wB, hB]])
		self.warp_matrix = None
		self.inv_warp_matrix = None
		self.angle = 0.0

		self.last_time = 0
		self.last_error = 0
		self.last_PD = 0
		self.Kp = 0.1  # .06
		self.Kd = 0.02# .01
		self.cache = [None, None]

		# Rolling average parameters
		self.median_matrix = list()
		self.median_constant = 0
		self.median = 0.0

	def warp_image(self, frame):

		# Destination points for warping
		dst_points = np.float32([[0, 0], [self.width, 0], [0, self.height], [self.width, self.height]])

		self.warp_matrix = cv2.getPerspectiveTransform(self.src_points, dst_points)
		self.inv_warp_matrix = cv2.getPerspectiveTransform(dst_points, self.src_points)

		# Warp frame
		warped_frame = cv2.warpPerspective(frame, self.warp_matrix, (self.width, self.height))

		return warped_frame

	def calculate_lane_in_pixels(self, frame):

		count = 0
		start_count = False

		for j in range(1, self.height):

			if frame[0, j] == 0 and frame[0, j - 1] == 255:
				start_count = True

			if start_count and frame[0, j] == 0:
				count += 1

			if start_count and frame[0, j] == 255 and frame[0, j + 1] == 255:
				break

	def polyfit_sliding_window(self, frame):

		# Check if frame is black
		#         if frame.max() <= 0:
		#             return np.array([[0,0,0],[0,0,0]])

		# Compute peaks in the two frame halves, to get an idea of lane start positions
		histogram = None

		cutoffs = [int(self.height / 2.0), 0]
		out = np.dstack((frame, frame, frame)) * 255
		for cutoff in cutoffs:
			histogram = np.sum(frame[cutoff:, :], axis=0)

			if histogram.max() > 0:
				break

		if histogram.max() == 0:
			# print('Unable to detect lane lines in this frame. Trying another frame!')
			return np.array([None, None, out])

		# Calculate peaks of histogram

		midpoint = np.int(self.width / 2.0)
		left_factor = 1
		right_factor = 1
		# print(self.angle)
		# if self.angle > 15:
		#    left_factor = 1.5
		#   right_factor = 1.5
		# if self.angle < -15:
		#   left_factor = 0.5
		#    right_factor = 0.5
		max_left = np.argmax(histogram[:int(midpoint * left_factor)])

		# print("Left base: ", left_factor)
		right_mid = int(midpoint * right_factor)
		b = histogram[int(right_mid):]

		b = b[::-1]
		max_right = np.argmax(b)

		#         rightx_base = np.argmax(b) + int(1.5*midpoint)

		leftx_base = max_left
		rightx_base = len(b) - max_right - 1 + right_mid
		# Width of the windows +/- margin
		margin = 20  # πλάτος παραθύρου

		# Number of sliding windows
		windows_number = 12  # πόσα παραθυράκια θα κάνει για να σκανάρει όλη την εικόνα

		# Min number of pixels needed to recenter the window
		minpix = 10

		# Window Height
		window_height = int(self.height / float(windows_number))

		# Min number of eligible pixels needed to fit a 2nd order polynomial as a lane line
		min_lane_pts = 500

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

		# print("DIFF: ", rightx_base - leftx_base,  " THRESH: " ,self.width//5 )

		for window in range(windows_number):
			# Find window boundaries in x and y axis
			win_y_low = self.height - (1 + window) * window_height
			win_y_high = self.height - window * window_height

			# LEFT

			win_xleft_low = leftx_current - margin
			win_xleft_high = leftx_current + margin

			# Draw windows for visualisation
			cv2.rectangle(out, (win_xleft_low, win_y_low), (win_xleft_high, win_y_high), \
						  (0, 0, 255), 2)

			good_left_inds = ((nonzeroy >= win_y_low) & (nonzeroy <= win_y_high)
							  & (nonzerox >= win_xleft_low) & (nonzerox <= win_xleft_high)).nonzero()[0]
			left_lane_inds.append(good_left_inds)

			# RIGHT
			win_xright_low = rightx_current - margin
			win_xright_high = rightx_current + margin

			cv2.rectangle(out, (win_xright_low, win_y_low), (win_xright_high, win_y_high), \
						  (0, 255, 0), 2)

			# Identify the nonzero pixels in x and y within the window
			good_right_inds = ((nonzeroy >= win_y_low) & (nonzeroy <= win_y_high)
							   & (nonzerox >= win_xright_low) & (nonzerox <= win_xright_high)).nonzero()[0]

			right_lane_inds.append(good_right_inds)
			# If you found > minpix pixels, recenter next window on their mean position
			if len(good_left_inds) > minpix:
				leftx_current = int(np.mean(nonzerox[good_left_inds]))

			if len(good_right_inds) > minpix:
				rightx_current = int(np.mean(nonzerox[good_right_inds]))

		# Edge case for when one line is perceived as both left and right lane
		if abs(rightx_base - leftx_base) < (self.width // 5):
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

		#         print("@"*20)
		#         print("histogram[leftx_base] VS histogram[rightx_base]: ", histogram[leftx_base], histogram[rightx_base])
		# print("$"*20)
		# print("Leftx VS Min_lane_pts: ", len(leftx), min_lane_pts)
		# Sanity check; Fit a 2nd order polynomial for each lane line pixels
		if len(leftx) >= min_lane_pts:  # and histogram[leftx_base] != 0:
			left_fit = np.polyfit(lefty, leftx, 2)

		# print("^"*20)
		# print("Rightx VS Min_lane_pts: ", len(rightx), min_lane_pts)
		# print("^"*20)
		if len(rightx) >= min_lane_pts:  # and histogram[rightx_base] != 0:
			right_fit = np.polyfit(righty, rightx, 2)
		# print("Left fit VS Right fit: ", left_fit is not None, right_fit is not None)

		# Color the detected pixels for each lane line
		out[nonzeroy[left_lane_inds], nonzerox[left_lane_inds]] = [255, 0, 0]
		out[nonzeroy[right_lane_inds], nonzerox[right_lane_inds]] = [255, 10, 255]

		return np.array([left_fit, right_fit, out])

	def get_poly_points(self, left_fit, right_fit):

		# TODO: CHECK EDGE CASES

		ysize, xsize = self.height, self.width

		# Get the points for the entire height of the image
		plot_y = np.linspace(0, ysize - 1, ysize)
		# print(len(plot_y))
		plot_xleft = left_fit[0] * plot_y ** 2 + left_fit[1] * plot_y + left_fit[2]
		plot_xright = right_fit[0] * plot_y ** 2 + right_fit[1] * plot_y + right_fit[2]

		# But keep only those points that lie within the image
		# plot_xleft = plot_xleft[(plot_xleft >= 0) & (plot_xleft <= xsize - 1)]
		# plot_xright = plot_xright[(plot_xright >= 0) & (plot_xright <= xsize - 1)]
		# plot_yleft = np.linspace(ysize - len(plot_xleft), ysize - 1, len(plot_xleft))
		# plot_yright = np.linspace(ysize - len(plot_xright), ysize - 1, len(plot_xright))

		plot_yright = plot_y
		plot_yleft = plot_y

		return plot_xleft.astype(np.int), plot_yleft.astype(np.int), plot_xright.astype(np.int), plot_yright.astype(
			np.int)

	def get_error(self, left_x, left_y, right_x, right_y):

		num_lines = 20

		line_height = int(self.height / float(num_lines))
		# CHECK ORDER FOR WEIGHT AVERAGE
		lines = np.flip(np.array([int(self.height - 1 - i * line_height) for i in range(num_lines)]))
		# print(lines)
		factor = int(round(0.5 * self.height))

		sample_right_x = right_x[factor:]
		sample_left_x = left_x[factor:]

		sample_x = np.array((sample_right_x + sample_left_x) / 2.0)
		# print(len(sample_x))
		if len(sample_x) != 0:
			weighted_mean = self.weighted_average(sample_x)

		error = weighted_mean - int(self.width / 2.0)
		print("Center: ", int(self.width / 2.0), " Mean: ", weighted_mean)
		setpoint = weighted_mean

		return error, setpoint

	def weighted_average(self, num_list):

		mean = 0
		count = len(num_list)
		# CHECK WEIGHTS, SHOULD BE FLIPPED?
		weights = [*range(0, count)]
		# print(weights)
		#mean = int(round(sum([num_list[i] * weights[i] for i in range(count)]) / sum(weights), 2))
		mean = np.average(num_list, weights=weights)
		# mean = int(round(sum([num_list[i]*1 for i in range(count)])/count,2))
		# weights[i]
		return mean

	def plot_points(self, left_x, left_y, right_x, right_y, frame):

		out = frame * 0

		for i in range(len(left_x)):
			cv2.circle(out, (left_x[i], left_y[i]), 10, color=(255, 255, 255), thickness=-1)
			cv2.circle(out, (right_x[i], right_y[i]), 10, color=(255, 255, 255), thickness=-1)

		# cv2.circle(out, (int((left_x[i] + right_x[i]) / 2.0), int((left_y[i] + right_y[i]) / 2.0)), 10, color=(255,255,255), thickness=-1)

		return out

	def lane_keeping_pipeline(self, frame):
		start = time.time()
		warped = self.warp_image(frame)
		#         print("Warp: ", time.time() - start)
		start = time.time()
		gray = cv2.cvtColor(warped, cv2.COLOR_BGR2GRAY)
		#         print("CVTColor: ", time.time() - start)
		start = time.time()
		thresh = cv2.adaptiveThreshold(gray, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, \
									   cv2.THRESH_BINARY, (39), -20)  # -56
		edged = cv2.Canny(gray, 100, 200)
		#         ret, thresh = cv2.threshold(gray,180,255,cv2.THRESH_BINARY)
		# print("Threshold: ", time.time() - start)
		# ========================TEST CODE=======        rho = 1  # distance precision in pixel, i.e. 1 pixel
		rho = 1  # distance precision in pixel, i.e. 1 pixel
		angle = np.pi / 180  # angular precision in radian, i.e. 1 degree
		min_threshold = 60  # minimal of votes
		lines = cv2.HoughLinesP(edged, rho, angle, min_threshold,
								np.array([]), minLineLength=8, maxLineGap=4)
		theta = 0
		if (lines is not None):
			for x in range(0, len(lines)):
				for x1, y1, x2, y2 in lines[x]:
					cv2.line(warped, (x1, y1), (x2, y2), (0, 255, 0), 2)
					theta = theta + (math.atan2((y2 - y1), (x2 - x1)))
			theta *= -1
		# print("$"*50)
		# print(theta)
		# print("$"*50)
		# ========================END TEST CODE=================
		both_lanes = False
		poly_image = thresh
		start = time.time()
		left, right, out = self.polyfit_sliding_window(thresh)
		# print("Polyfit: ", time.time() - start)

# 		if left is None and self.cache[0] is not None:
# 			left = self.cache[0]
# 		if right is None and self.cache[1] is not None:
# 			right = self.cache[1]
		
		if left is not None and right is not None:
			print("BOTH LANES")

			self.cache[0] = left
			self.cache[1] = right
			# start = time.time()
			left_x, left_y, right_x, right_y = self.get_poly_points(left, right)
			#             print("Poly points: ", time.time() - start)

			# start = time.time()
			poly_image = self.plot_points(left_x, left_y, right_x, right_y, warped)
			#             print("Plot points: ", time.time() - start)

			# start = time.time()
			error, setpoint = self.get_error(left_x, left_y, right_x, right_y)
			# print("GetError: ", error)

			nose2wheel = self.height  # 160#self.height//10

			self.angle = 90 - math.degrees(math.atan2(nose2wheel, error))

			both_lanes = True
# 			if theta * self.angle <= 0 and abs(theta - self.angle) > 5:
# 				self.angle += theta // 2
		# print("Coords", nose2wheel, error)
		# print("Angle: ", self.angle)

		elif right is None and left is not None:
			print("LEFT LANE")
			self.cache[0] = left

			x1 = left[0] * self.height ** 2 + left[1] * self.height + left[2]
			x2 = left[2]

			dx = x2 - x1
			dy = self.height # self.height

			# tan = float(dy) / float(dx)

			self.angle = 90 - math.degrees(math.atan2(dy, dx))
# 			if theta * self.angle <= 0 and abs(theta - self.angle) > 5:
# 				self.angle = theta
		# angle = 20

		elif left is None and right is not None:
			print("RIGHT LANE")
			self.cache[1] = right
			x1 = right[0] * self.height ** 2 + right[1] * self.height + right[2]
			x2 = right[2]

			dx = x2 - x1
			dy = self.height  # self.height//8

			# tan = float(dy) / float(dx)

			self.angle = 90 - math.degrees(math.atan2(dy, dx))
# 			if theta * self.angle <= 0 and abs(theta - self.angle) > 5:
# 				self.angle = theta
		# print("SINGLE LINE ANGLE: " , angle)
		else:
			print("No lanes found")
		white_angle = self.angle

		# ===========TEST CODE=========
		# self.angle += (-1 * (theta))//2
		now = time.time()
		dt = now - self.last_time

		deviation = self.angle

		error = deviation

		derivative = self.Kd * (error - self.last_error) / dt
		proportional = self.Kp * error
		PD = int(self.angle + derivative + proportional)

		self.last_error = error
		self.last_time = time.time()

		# if (PD - self.last_PD) > 10:
		# 	PD = (PD + self.last_PD)/2

		self.median_matrix.insert(0, PD)
		if len(self.median_matrix) == self.median_constant:
			# for i in range(0, self.median_constant):
			#     self.median += self.median_matrix[i]
			# self.median /= self.median_constant
			self.median = np.average(self.median_matrix)
			PD = self.median
			self.median = 0.0
			self.median_matrix.pop()

		# PD /= 2

		self.angle = PD
		self.last_PD = PD

		# font
		font = cv2.FONT_HERSHEY_SIMPLEX

		# org
		org = (50, 50)

		# fontScale
		fontScale = 1

		# Blue color in BGR
		color = (255, 0, 0)

		# Line thickness of 2 px
		thickness = 1

		# Using cv2.putText() method
		image = cv2.putText(warped, 'Theta: ' + str(int(theta)), org, font,
							fontScale, color, thickness, cv2.LINE_AA)

		image = cv2.putText(warped, 'Angle: ' + str(int(self.angle)), (50, 70), font,
							fontScale, color, thickness, cv2.LINE_AA)

		image = cv2.putText(warped, 'Derivative: ' + str(int(derivative)), (50, 90), font, fontScale, color, thickness,
							cv2.LINE_AA)
		cv2.putText(warped, 'Proportional: ' + str(int(proportional)), (50, 120), font, fontScale, color, thickness,
					cv2.LINE_AA)
		cv2.putText(warped, 'CurrentVSfinal: ' + str(int(PD)) + ' ' + str(int(self.angle)), (0, 200), font, fontScale,
					color, thickness, cv2.LINE_AA)

		# =========END TEST CODE======
		if self.angle > 0:
			self.angle = min(23, self.angle)
		else:
			self.angle = max(-23, self.angle)
		
		return self.angle, warped


# image = LaneKeepingReloaded(width=1194, height=781)
# frame7878 = cv2.imread("result3496c.png")
# print("the type is:", type(frame7878))
# cv2.imshow("", frame7878)
# k = cv2.waitKey(0)
# if k == 27:         # wait for ESC key to exit
#     cv2.destroyAllWindows()
# cv2.waitKey(0)
# print("image shape is", frame7878.shape)
# lines = image.lane_keeping_pipeline(frame7878)
# cv2.imshow("fe,", lines[1])
# k = cv2.waitKey(0)
# if k == 27:         # wait for ESC key to exit
#     cv2.destroyAllWindows()
# cv2.waitKey(0)
# numpy_vertical_concat = np.concatenate((frame7878, lines[1]), axis=0)
# numpy_horizontal_concat = np.concatenate((frame7878, lines[1]), axis=1)

# cv2.imshow("frame.png", lines[1])
# k = cv2.waitKey(0)
# if k == 27:         # wait for ESC key to exit
#     cv2.destroyAllWindows()
# cv2.waitKey(0)