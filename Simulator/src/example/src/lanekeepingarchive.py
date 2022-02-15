import cv2
import numpy as np
import math

"""
This class implements the lane keeping algorithm by calculating angles from the detected lines.
Input: Camera Input
Output: Steering Angle
To-Do: wrap // why change lane // 
"""

class LaneKeeping:

    def __init__(self):
        
        self.angle = 0.0
        self.last_angle=0.0
        self.angle_buffer=[]
        self.count=0
        self.xl=[]
        self.yl=[]
        self.xr=[]
        self.yr=[]
        self.framecount=1

    def canny(self, image):
        gray=cv2.cvtColor(image,cv2.COLOR_RGB2GRAY)
        blur=cv2.GaussianBlur(gray,(5,5),0)
        canny=cv2.Canny(blur,150,350)     #100,300
        return canny

    def masked_region(self,image):
        polygons=np.array([
            [(self.width/6,self.height*3/5),(0,self.height*4/5),(self.width,self.height*4/5),(self.width*5/6,self.height*3/5)] #(y,x)*5/6
            ])
        mask=np.zeros_like(image)
        cv2.fillPoly(mask,np.int32([polygons]),255)
        masked_image=cv2.bitwise_and(image,mask) 
        return masked_image

    def masked_region1(self,image):
        polygons=np.array([
            [(0,0),(0,self.height/3),(self.width,self.height/3),(self.width,0)] #(y,x)
            ])
        mask=np.zeros_like(image)
        cv2.fillPoly(mask,np.int32([polygons]),255)
        masked_image=cv2.bitwise_and(image,mask) 
        return masked_image

    def warp(self,image,height,width):
        # Destination points for warping
        dst_points = np.float32([[self.width/7, height*55/100],[0, self.height*4/5],[self.width, self.height*4/5],[self.width*6/7, self.height*55/100]])
        src_points = np.float32([[self.width/6,self.height*3/5],[0,self.height*4/5],[self.width,self.height*4/5],[self.width*5/6,self.height*55/100]])

        warp_matrix = cv2.getPerspectiveTransform(src_points, dst_points)
        warped_frame = cv2.warpPerspective(image,warp_matrix,(width,height))
        return warped_frame

    def make_lines(self,lines):
        left_fit=[]     
        right_fit=[]
        self.xl=[]
        self.yl=[]
        self.xr=[]
        self.yr=[]
        if lines is None:
                return np.array(left_fit), np.array(right_fit)

        #Loop through the lines
        for line in lines:
            x1,y1,x2,y2=line[0]
            if (np.abs(y2-y1) < 2) or (np.abs(x2-x1 < 3)):
                #
                continue
            elif  (x1 < self.width*2/5):
            # (((y2 - y1) // (x2 - x1))<0) and
                self.xl.append(x1)
                self.yl.append(y1)
                self.xl.append(x2)
                self.yl.append(y2)
                pts = np.array([[x1, y1 ], [x2 , y2]], np.int32)
                cv2.polylines(self.frame, [pts], True, (0,255,0),4)
                # cv2.line(self.frame,(x1,y1),(x2,y2),(0,255,0),4)

            elif  (x1 > self.width*3/5):
            #  (((y2 - y1) // (x2 - x1))>0) and
                self.xr.append(x2)
                self.yr.append(y2)
                self.xr.append(x1)
                self.yr.append(y1)
                
                pts = np.array([[x1, y1 ], [x2 , y2]], np.int32)
                cv2.polylines(self.frame, [pts], True, (255,0,0),4)

        cv2.imshow('lines',self.frame)

        self.yr=np.array(self.yr)
        self.xr=np.array(self.xr)
        self.yl=np.array(self.yl)
        self.xl=np.array(self.xl)

        if (np.count_nonzero(self.xl)!=0) and (np.count_nonzero(self.yl)!=0):         
            slope_l,middle_l=np.polyfit((self.yl),(self.xl),1)
            left_fit.append((slope_l,middle_l))
        if (np.count_nonzero(self.xr)!=0) and (np.count_nonzero(self.yr)!=0):
            slope_r,middle_r=np.polyfit((self.yr),(self.xr),1)
            right_fit.append((slope_r,middle_r))

        # print ('right y,x: ', self.yr, self.xr)
        # print ('left y,x: ', self.yl, self.xl)
        return np.array(left_fit), np.array(right_fit)

    def lanes_pipeline(self,cap):

        self.frame=cap
        # self.framecount=self.framecount+1
        # if self.framecount%2==1:
        #     return self.angle

        self.height=self.frame.shape[0]
        self.width=self.frame.shape[1]
        
        canny_image=self.canny(self.frame)
        # wrapped_image=self.warp(canny_image,self.height,self.width)

        masked_image=self.masked_region(canny_image)
        # number_of_white_pix = np.sum(masked_image > 250)
   
        # print("num of pixels: ",number_of_white_pix)

        # if number_of_white_pix > 3000:
        #     self.angle=0
        #     self.count=1
        #     return self.angle  
        # elif (number_of_white_pix > 2400) and (self.count==1):
        #     masked_image=self.masked_region1(canny_image)
        # else:
        #     self.count=0
# rho=2
        lines=cv2.HoughLinesP(masked_image,rho=1,theta=np.pi/180,threshold=30,lines=np.array([]),minLineLength=3,maxLineGap=7)
        self.left, self.right=self.make_lines(lines)

        steering_angle=self.lane_keeping_pipeline()

        return steering_angle

    # def get_poly_points(self, line_fit, id):		#left_fit=[a,b,c] from y=ax^2+bx+c
        
    #     a,b,c=line_fit[0]       
    #     if id==1:
    #         plot_x=np.linspace(0,self.width/2,num=10)
    #         # plot_x = np.linspace(sol1.real, self.width*2/3,num=10)
    #         plot_y = a*(plot_x**2) + b*(plot_x) + c
    #     elif id==2:
    #         plot_x=np.linspace(self.width/2,self.width,num=10)
    #         # plot_x = np.linspace(sol2.real, self.width*2/3,num=10)
    #         plot_y = a*(plot_x**2) + b*(plot_x) + c

    #     # print('linespace x,y', plot_x,plot_y)
    #     return plot_y.astype(np.int),plot_x.astype(np.int)

    # def get_error(self, left_x, right_x): #the x calculated from get_poly_points

    #     factor = int(round(0.5 * self.height)) #240*0.5= 120

    #     sample_right_x = right_x[(right_x>= factor)]  #all after factor(120) => right_x[121,122,..240]
    #     sample_left_x = left_x[left_x>=factor] 	#all after factor => left_x[121,122,...240]  => the bottom of the self.frame
    #     sample_x = np.array((right_x + left_x) / 2.0)

    #     if len(sample_x) != 0:
    #         weighted_mean = self.weighted_average(sample_x)

    #     error = weighted_mean - int(self.width / 2.0)
    #     error1=np.average(sample_x)
    #     return error

    # def weighted_average(self, num_list):

    #     mean = 0
    #     count = len(num_list)
    #     # CHECK WEIGHTS, SHOULD BE FLIPPED?
    #     weights = [*range(0, count)]
    #     mean = np.average(num_list, weights=weights)

    #     return mean

    # def lane_keeping_pipeline(self):
        
    #     left, right=self.left, self.right
    #     print('left:',left)
    #     print('right:',right)
        
    #     if np.count_nonzero(left) !=0  and np.count_nonzero(right) !=0 :   
    #         print("BOTH LANES")
    #         al,bl=left[0]
    #         ar,br=right[0]
         
    #         if  len(self.yr) >= len(self.yl):
    #             print("RIGHT LANE")
               
    #             # a,b,c=right[0]
    #             # y1=a*(self.width**2)+ b*self.width+c
    #             # y2=c
    #             # dy=y2-y1
    #             # dx=self.width
    #             # self.angle = 90 - math.degrees(math.atan2(dy, dx))

    #             a,b=right[0]
    #             x1=a*self.height+ b
    #             x2=b
    #             dx=x2-x1
    #             dy=self.height
    #             self.angle = 90 - math.degrees(math.atan2(dy, dx))

    #             # # x1=ar*((self.width/2)**2)+ br*(self.width/2)+cr
    #             # # x2=cr
    #             # # dx=x2-x1
    #             # # dy=self.height
    #             # # self.angle = 90 - math.degrees(math.atan2(dy, dx))

    #             # right_y,right_x = self.get_poly_points(right,2)
    #             # left_x = np.linspace(0, self.width/3, num=len(right_x))
    #             # error= self.get_error(left_x,right_x)
    #             # self.angle = 90 - math.degrees(math.atan2(self.height,error))

    #         # (second_der_l > 0.001)
    #         elif len(self.yr) < len(self.yl):
    #             # and (second_der_l > -0.003)
    #             print("LEFT LANE")
              
    #             x1=al*self.height+ bl
    #             x2=bl
    #             dx=x2-x1
    #             dy=self.height
    #             self.angle = 90 - math.degrees(math.atan2(dy, dx))
                
    #             # left_y,left_x = self.get_poly_points(left,1)
    #             # right_x = np.linspace(self.width*2/3, self.width, num=len(left_x))
    #             # error= self.get_error(left_x,right_x)
    #             # x=math.degrees(math.atan2(self.height,error))
    #             # self.angle = 90 - x

    #         else:
    #             left_y,left_x = self.get_poly_points(left,1)
    #             right_y,right_x = self.get_poly_points(right,2)
    #             error= self.get_error(left_y,right_y)
    #             x=math.degrees(math.atan2(error,self.width/2))
    #             self.angle = x-25

    #     if np.count_nonzero(right)==0  and np.count_nonzero(left) !=0 :
    #         print("LEFT LANE")     
    #         # a,b=left[0]
    #         # x=math.degrees(math.atan(a))
    #         # self.angle=-47-x
    #         # print('x, angle ',x,self.angle)
    #         a,b=left[0]

    #         x1=a*self.height+ b
    #         x2=b
    #         dx=x2-x1
    #         dy=self.height
    #         self.angle = 90 - math.degrees(math.atan2(dy, dx))
    #         # left_y,left_x = self.get_poly_points(left,1)
    #         # right_x = np.linspace(self.width*2/3, self.width, num=len(left_x))
    #         # error= self.get_error(left_x,right_x)
    #         # x=math.degrees(math.atan2(self.height,error))
    #         # self.angle = 90 - x

    #     elif np.count_nonzero(left)==0  and np.count_nonzero(right) !=0 :
    #         print("RIGHT LANE")

    #         a,b=right[0]
    #         x1=a*self.height+ b
    #         x2=b
    #         dx=x2-x1
    #         dy=self.height
    #         self.angle = 90 - math.degrees(math.atan2(dy, dx))
    #         # a,b=right[0]
    #         # x=math.degrees(math.atan(a))
    #         # self.angle=47-x
    #         # print('x, angle ',x,self.angle)

    #         # x1=a*self.height**2+ b*self.height+c
    #         # x2=c
    #         # dx=x2-x1
    #         # dy=self.height
    #         # self.angle = 90 - math.degrees(math.atan2(dy, dx))

    #         # right_y,right_x = self.get_poly_points(right,2)
    #         # left_x = np.linspace(self.width/3, self.width, num=len(right_x))
    #         # error= self.get_error(left_x,right_x)
            
    #         # self.angle = 90 - math.degrees(math.atan2(self.height,error))


    def get_poly_points(self, left_fit, right_fit):

        ysize, xsize = self.height, self.width

        # Get the points for the entire height of the image
        plot_y = np.linspace(0, ysize - 1, ysize)
        al,bl=left_fit[0]
        ar,br=right_fit[0]
        plot_xleft = al * plot_y + bl 
        plot_xright = ar * plot_y  + br

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

    def lane_keeping_pipeline(self):

        if np.count_nonzero(self.left) !=0  and np.count_nonzero(self.right) !=0 :
            print("BOTH LANES")

            left_x, left_y, right_x, right_y = self.get_poly_points(self.left, self.right)

            error, setpoint = self.get_error(left_x, left_y, right_x, right_y)  

            self.angle = 90 - math.degrees(math.atan2(self.height, error))

        elif np.count_nonzero(self.left)!=0  and np.count_nonzero(self.right) ==0 :
            print("LEFT LANE")
            a,b=self.left[0]
            x1 = a*self.height + b
            x2 = b

            dx = x2 - x1
            dy = self.height 

            self.angle = 90 - math.degrees(math.atan2(dy, dx))

        elif np.count_nonzero(self.left)==0  and np.count_nonzero(self.right) !=0 :
            print("RIGHT LANE")
            a,b=self.right[0]
            x1 = a*self.height + b 
            x2 = b

            dx = x2 - x1
            dy = self.height  

            self.angle = 90 - math.degrees(math.atan2(dy, dx))
        
        else:
            print("No lanes found")

        if np.abs(self.last_angle - self.angle) < 0.5:
            self.angle = self.last_angle
        else :
            if self.angle>0:
                self.angle = min(23, self.angle)
            else:
                self.angle = max(-23, self.angle)

            moving_size=4
            weights = [*range(1, moving_size + 1)]
        
            if len(self.angle_buffer) >= moving_size:
                self.angle_buffer.pop(0)
                self.angle_buffer.append(self.angle)  
            else:
                self.angle_buffer.append(self.angle)
                weights = [*range(1, len(self.angle_buffer)+1)]
            
            self.angle = np.average(self.angle_buffer, weights=weights)

        print('angle ', self.angle, '\n')

        return self.angle

if __name__ == "__main__":
    lk= LaneKeeping()
    lk.lanes_pipeline()