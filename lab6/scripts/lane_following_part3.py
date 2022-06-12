#!/usr/bin/env python

from __future__ import print_function
from distutils.log import INFO
import rospy, cv2, cv_bridge, numpy, time, math
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist

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
                        "DICT_ARUCO_ORIGINAL": cv2.aruco.DICT_ARUCO_ORIGINAL
                }

                self.D = numpy.array([0.0, 0.0, 0.0, 0.0, 0.0])

                self.K = numpy.array([[-3.313061098815712, 0.0, 160.5], [0.0, -3.313061098815712, 120.5],[0.0, 0.0, 1.0]])
                
             
                self.time0=time.time()
                self.stoptime=0 
                self.stop = 'false'
                self.stop_check='true'
                self.tem=cv2.imread("cor.png")
                self.check='true'
                self.corner=0

                self.bridge = cv_bridge.CvBridge()

                self.image_sub = rospy.Subscriber('raspicam_node/image',
                        Image, self.image_callback)

                self.cmd_vel_pub = rospy.Publisher('cmd_vel',
                        Twist, queue_size=10)

                self.twist = Twist()

                pts_src = numpy.array([[27, 239], [283, 239], [42, 224],[268, 224]])
                pts_dst = numpy.array([[-20, 239],[160, 239],[-20, 210],[160,210]]) 
               
                self.homography, self.status = cv2.findHomography(pts_src, pts_dst)

        def image_callback(self, msg):

                this_aruco_dictionary = cv2.aruco.Dictionary_get(self.ARUCO_DICT["DICT_6X6_250"])
                this_aruco_parameters = cv2.aruco.DetectorParameters_create()

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

                (corners, ids, rejected) = cv2.aruco.detectMarkers(image, this_aruco_dictionary, parameters=this_aruco_parameters)

                
               
                print(min_val)

                
              


                if min_val < 0.36 and self.check == 'true':
                    self.corner=self.corner+1;
                    self.check = 'false'
                    self.close=time.time()
                print(min_val, self.check,self.corner)
                
                if self.corner>=1 and self.stop=='false':
                    self.on='false'
                    if self.stop_check=='true':
                        self.stoptime=time.time()
                        self.stop_check='false'
                    else: 
                        time2=time.time()
                        on=time2-self.stoptime 

                    if on<=2: 
                        self.twist.linear.x = 0.2
                        self.twist.angular.z = 0.0
                        self.cmd_vel_pub.publish(self.twist)
                    elif on> 2 and on<3.5:
                        self.twist.linear.x = 0.2
                        self.twist.angular.z = -2
                        self.cmd_vel_pub.publish(self.twist)
                    else:
                        self.stop_count=0
                        self.corner=0  
                        self.twist.linear.x = 0.2
                        self.twist.angular.z =0.0
                        self.cmd_vel_pub.publish(self.twist)
                        self.stop=true
                        self.stop_check='true'
                        self.stop='true'
                print(self.stop)


                if len(corners) > 0 and self.stop=='true':
                    if self.stop_check=='true':
                        self.stoptime=time.time()
                        self.stop_check='false'
                    else:
                        time2=time.time()
                        on=time2-self.stoptime 
                    if on<=2:
                        self.twist.linear.x = 0.2
                        self.twist.angular.z =0.0
                        self.cmd_vel_pub.publish(self.twist)   
                    else:
                        self.twist.linear.x = 0.0
                        self.twist.angular.z =0.0
                        self.cmd_vel_pub.publish(self.twist)               

               

                cv2.imshow("camera view", image)
                cv2.imshow("gray mask",gray_mask)
                cv2.waitKey(1)

        

rospy.init_node('lane_follower')
follower = Follower()
rospy.spin()
