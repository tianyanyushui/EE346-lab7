#!/usr/bin/env python

import rospy, cv2, cv_bridge, numpy, math
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
import cv2.aruco as aruco
import time

class Follower:

        def __init__(self):
                self.time0=time.time();
                self.corner=0;
                self.tem=cv2.imread("cor.png")
                self.check='true'
                self.close=0
                self.bridge = cv_bridge.CvBridge()

                self.image_sub = rospy.Subscriber('raspicam_node/image',
                        Image, self.image_callback)


                self.cmd_vel_pub = rospy.Publisher('cmd_vel',
                        Twist, queue_size=10)

                self.twist = Twist()
                
                self.Nfound = True#find marker


                self.onlyRight = True
                self.onlyLeft = False

                if self.onlyRight:
                        pts_src = numpy.array([[27, 239], [283, 239], [42, 224],[268, 224]])
                        pts_dst = numpy.array([[-20, 239],[160, 239],[-20, 210],[160,210]])

		#elif self.onlyLeft:
                        #pts_src = numpy.array([[27, 239], [283, 239], [42, 224],[268, 224]])
                        #pts_dst = numpy.array([[160, 239],[340, 239],[160, 210],[340,210]])		

                else:
                        pts_src = numpy.array([[27, 239], [283, 239], [42, 224],[268, 224]])
                        pts_dst = numpy.array([[45, 239],[275, 239],[45, 210],[275,210]])
                
                self.homography, self.status = cv2.findHomography(pts_src, pts_dst)

        def image_callback(self, msg):    

                # find marker
                aruco_dict = aruco.Dictionary_get(aruco.DICT_6X6_250)
                parameters = aruco.DetectorParameters_create()
                font = cv2.FONT_HERSHEY_SIMPLEX  # font for displaying text (below)
  
                image = self.bridge.imgmsg_to_cv2(msg,desired_encoding='bgr8')
                h, w, d = image.shape
                bev = cv2.warpPerspective(image, self.homography, (w,h)) 
                gray = cv2.cvtColor(bev, cv2.COLOR_RGB2GRAY)
                gray[gray<10] = 200     
                #flag, gray_thresh = cv2.threshold(gray,110,255,cv2.THRESH_BINARY)
		flag, gray_thresh = cv2.threshold(gray,0,255,cv2.THRESH_BINARY+cv2.THRESH_OTSU)
                gray_mask = 255-gray_thresh

		hsv = cv2.cvtColor(bev, cv2.COLOR_RGB2HSV)

		lower_yellow = numpy.array([0, 0, 0])
		upper_yellow = numpy.array([85, 255, 255])
		hsv_mask1 = cv2.inRange(hsv, lower_yellow, upper_yellow)

		lower_line = numpy.array([0, 0, 0])
		upper_line = numpy.array([5, 5, 220])
		hsv_mask2 = cv2.inRange(hsv, lower_line, upper_line)

		gray_mask[hsv_mask1 == 0] = 0
		#gray_mask[hsv_mask2 == 255] = 0

                search_top = 40
                search_bot = 200
                search_left = 40  #40 for right; 60 for left
                search_right = 260  #260 for right; 280 for left
                gray_mask[0:search_top, 0:w] = 0
                gray_mask[0:h, 0:search_left] = 0
                gray_mask[0:h, search_right:w] = 0
                #gray_mask[search_bot:h, 0:w] = 0

                RGB=cv2.cvtColor(gray_mask,cv2.COLOR_GRAY2BGR)
                result=cv2.matchTemplate(RGB,self.tem, cv2.TM_SQDIFF_NORMED,-1)
                min_val,max_val,min_loc,max_loc=cv2.minMaxLoc(result)
                print(min_val)

                if min_val<0.25 and self.check=='true':
                    self.corner=self.corner+1;
                    self.check='false'
                    self.close=time.time()
                print(self.check,self.corner)

                curv = self.cal_curvature(gray)
                curv = max(0, numpy.mean(curv) - 0.15075)

                M = cv2.moments(gray_mask)
                time2=time.time()
                time_change=time2-self.time0
                self.twist.linear.x=0.2
                if (time2-self.close)>=4 and self.close!=0:
                    self.check='true'
                    self.close=0
                
                if time_change>3:
                    self.twist.linear.x = 0.2
                    self.twist.angular.z = -2

                if M['m00'] > 0 and time_change > 4 and self.corner<2: #4 for right;
                #if M['m00'] > 0 and time_change > 4 and self.corner<2:
                    cx = int(M['m10']/M['m00'])
                    cy = int(M['m01']/M['m00'])

                    fpt_x = cx  #(cx1 + cx2)/2
                    fpt_y = cy  #(cy1 + cy2)/2

                    cv2.circle(bev, (fpt_x, fpt_y), 10, (128,128,128), -1)

                    err = w/2 - fpt_x
                    theta = math.atan2(err, fpt_y) 
                    

                    self.twist.angular.z =   3 * theta + 0.5 * (err*90.0/160)/15
                    #if err != 0 =:
                    if self.corner<2:
                            self.twist.linear.x = 0.2
                            #self.twist.linear.x = 0.2
                    else:
                            self.twist.linear.x = 0.1
                    	    #print(4*theta,  (err*90.0/160)/15, self.twist.linear.x)

                    self.cmd_vel_pub.publish(self.twist)

               
                cv2.imshow("camera view", image)
                cv2.imshow("bird eye view",bev)
                cv2.imshow("gray mask",gray_mask)
                cv2.waitKey(1)

        def cal_curvature(self, img):
                
                x , y = numpy.gradient(img)
                xx, xy = numpy.gradient(x)
                yx, yy = numpy.gradient(y)
                Iup =  (1+x*x)*yy - 2*x*y*xy + (1+y*y)*xx
                Idown = 2*numpy.power((1 + x*x + y*y),1.5)
                curv  = Iup/Idown
                curv = abs(curv)
                return curv
 



rospy.init_node('lane_follower')
follower = Follower()
rospy.spin()
