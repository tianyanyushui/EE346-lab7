#!/usr/bin/env python

import rospy, cv2, cv_bridge, numpy, math
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
import cv2.aruco as aruco
import time


class Follower:

        def __init__(self):
                self.ARUCO_DICT = {
                         "DICT_4X4_50": cv2.aruco.DICT_4X4_50,
                        "DICT_4X4_100": cv2.aruco.DICT_4X4_100,
                        "DICT_4X4_250": cv2.aruco.DICT_4X4_250,
                         "DICT_4X4_1000": cv2.aruco.DICT_4X4_1000,
                         "DICT_5X5_50": cv2.aruco.DICT_5X5_50,
                         "DICT_5X5_100": cv2.aruco.DICT_5X5_100,
                         "DICT_5X5_250": cv2.aruco.DICT_5X5_250,
                        "DICT_5X5_1000": cv2.aruco.DICT_5X5_1000,
                         "DICT_6X6_50": cv2.aruco.DICT_6X6_50,
                        "DICT_6X6_100": cv2.aruco.DICT_6X6_100,
                        "DICT_6X6_250": cv2.aruco.DICT_6X6_250,
                        "DICT_6X6_1000": cv2.aruco.DICT_6X6_1000,
                         "DICT_7X7_50": cv2.aruco.DICT_7X7_50,
                         "DICT_7X7_100": cv2.aruco.DICT_7X7_100,
                         "DICT_7X7_250": cv2.aruco.DICT_7X7_250,
                         "DICT_7X7_1000": cv2.aruco.DICT_7X7_1000,
                        "DultiNavICT_ARUCO_ORIGINAL": cv2.aruco.DICT_ARUCO_ORIGINAL
                }

                self.time0 = time.time()
                self.time_end = 0
                self.time1=0
                self.corner=0
                self.tem=cv2.imread("cor.png")
                self.check = 'true'
                self.close=0
                self.left='true'
                self.lefton=True
                self.right='true'
                self.reset='true'
                self.readyStop = False
                self.bridge = cv_bridge.CvBridge()

                self.image_sub = rospy.Subscriber('raspicam_node/image',
                        Image, self.image_callback)


                self.cmd_vel_pub = rospy.Publisher('cmd_vel',
                        Twist, queue_size=10)

                self.twist = Twist()

                self.onlyRight = True
                self.onlyLeft = False
        
                if self.onlyRight:
                        pts_src = numpy.array([[27, 239], [283, 239], [42, 224],[268, 224]])
                        pts_dst = numpy.array([[-20, 239],[160, 239],[-20, 205],[160,205]])

		#elif self.onlyLeft:
                        #pts_src = numpy.array([[27, 239], [283, 239], [42, 224],[268, 224]])
                        #pts_dst = numpy.array([[160, 239],[340, 239],[160, 210],[340,210]])		

                else:
                        pts_src = numpy.array([[27, 239], [283, 239], [42, 224],[268, 224]])
                        pts_dst = numpy.array([[45, 239],[275, 239],[45, 210],[275,210]])
                
                self.homography, self.status = cv2.findHomography(pts_src, pts_dst)

        def image_callback(self, msg):  
                this_aruco_dictionary = cv2.aruco.Dictionary_get(self.ARUCO_DICT["DICT_6X6_250"])
                this_aruco_parameters = cv2.aruco.DetectorParameters_create()
  

                image = self.bridge.imgmsg_to_cv2(msg,desired_encoding='bgr8')
                h, w, d = image.shape
                bev = cv2.warpPerspective(image, self.homography, (w,h)) 
                gray = cv2.cvtColor(bev, cv2.COLOR_RGB2GRAY)
                gray[gray<10] = 255     
                flag, gray_thresh = cv2.threshold(gray,130,255,cv2.THRESH_BINARY)
		#flag, gray_thresh = cv2.threshold(gray,0,255,cv2.THRESH_BINARY+cv2.THRESH_OTSU)
                gray_mask = 255-gray_thresh
                gray_mask[gray<10] = 0
                hsv = cv2.cvtColor(bev, cv2.COLOR_RGB2HSV)
                lower_yellow = numpy.array([0, 0, 0])
                upper_yellow = numpy.array([180, 255, 70])
                hsv_mask1 = cv2.inRange(hsv, lower_yellow, upper_yellow)

                lower_line = numpy.array([0, 0, 0])
                upper_line = numpy.array([5, 5, 220])
                hsv_mask2 = cv2.inRange(hsv, lower_line, upper_line)

		#gray_mask[(hsv_mask1 == 255 and gray_mask == 255)] = 255
		#gray_mask[hsv_mask2 == 255] = 0

                search_top = 40
                search_bot = 200
                search_left = 40  #40 for right; 60 for left
                search_right = 260  #260 for right; 280 for left
                gray_mask[0:search_top, 0:w] = 0
                gray_mask[0:h, 0:search_left] = 0
                gray_mask[0:h, search_right:w] = 0
                #gray_mask[search_bot:h, 0:w] = 0

                time2 = time.time()
		#print(time_change)
                time_change = time2 - self.time0
		#if time_change >= 5.5:
                RGB=cv2.cvtColor(gray_mask,cv2.COLOR_GRAY2BGR)
                result=cv2.matchTemplate(RGB,self.tem, cv2.TM_SQDIFF_NORMED,-1)
                min_val,max_val,min_loc,max_loc=cv2.minMaxLoc(result)
		#print(min_val)

                M = cv2.moments(gray_mask)
                
                
                if time_change<=4:
                    self.twist.linear.x=0.2

                if (time2-self.close)>=4 and self.close!=0:
                    self.check='true'
                    self.close=0
                
                if time_change > 4 and time_change < 5.4:
                    self.twist.angular.z = -1
                    self.twist.linear.x=0.0
                    #self.cmd_vel_pub.publish(self.twist)
                     

                if min_val < 0.52 and self.check == 'true':
                    self.corner=self.corner+1;
                    self.check = 'false'
                    self.close=time.time()
                print(min_val, self.check,self.corner)

                if M['m00'] > 0 and time_change >= 5.5: #4 for right;
                    if self.reset=='true':  
                        self.time0=0
                        self.reset='false'
                #if M['m00'] > 0 and time_change > 4 and self.corner<2:
                    cx = int(M['m10']/M['m00'])
                    cy = int(M['m01']/M['m00'])

                    fpt_x = cx  #(cx1 + cx2)/2
                    fpt_y = cy  #(cy1 + cy2)/2

                    cv2.circle(bev, (fpt_x, fpt_y), 10, (128,128,128), -1)

                    err = w/2 - fpt_x
                    theta = math.atan2(err, fpt_y) 

                    
                    if self.corner == 2 and self.lefton:
                        self.twist.linear.x = 0.2
                        self.twist.angular.z = 0.3
                        self.cmd_vel_pub.publish(self.twist)
                        time.sleep(6.75)
                        if self.left=='true':
                            self.time1=time.time()
                            self.left='false'
                        self.lefton=False 
                    	#self.twist.angular.z = -1.5
                    
                    elif self.corner >= 4:
                        if self.readyStop == False:
                                self.time_end = time.time()
                                self.readyStop = True
                        time4 = time.time()
                        dur = time4 - self.time_end
                        if dur <= 3.1:  
                        	self.twist.linear.x = 0.02
                                self.twist.angular.z = -1.05
                                self.cmd_vel_pub.publish(self.twist)
                                time.sleep(1.5)
		        
                        else:
				self.twist.linear.x = 2.0
                        	self.twist.angular.z = 0.0
                        	self.cmd_vel_pub.publish(self.twist)
                		(corners, ids, rejected) = cv2.aruco.detectMarkers(image, this_aruco_dictionary, parameters=this_aruco_parameters)
				if len(corners) > 0:
					while True:
						print("Ready to Stop!")
						self.twist.linear.x = 2.0
						self.cmd_vel_pub.publish(self.twist)
						time.sleep(4.15)
						self.twist.linear.x = 0.0
						self.cmd_vel_pub.publish(self.twist)
						print("Stop!")
						time.sleep(1000)

                    else:
                            self.twist.linear.x = 0.2
                            self.twist.angular.z =   0.5 * (err*90.0/160)/15 + 2 * theta   
                            self.cmd_vel_pub.publish(self.twist) 
                    	    #print(4*theta,  (err*90.0/160)/15, self.twist.linear.x)

                    
                    print(self.twist.linear.x,self.twist.angular.z)      
	
		#self.twist.linear.x = 0.0
                #self.twist.angular.z = 0.0
		#print(self.corner, self.check)
		#print(self.twist.linear.x,self.twist.angular.z)
		self.cmd_vel_pub.publish(self.twist)
               
                cv2.imshow("camera view", image)
                cv2.imshow("bird eye view",bev)
                cv2.imshow("gray mask",gray_mask)
		cv2.imshow("hsv mask",hsv_mask1)
		cv2.imwrite("bev2.png",bev)
                cv2.waitKey(1)


rospy.init_node('lane_follower')
follower = Follower()
rospy.spin()
