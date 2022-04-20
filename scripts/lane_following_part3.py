#!/usr/bin/env python

import rospy, cv2, cv_bridge, numpy
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
import cv2.aruco as aruco
import time

class Follower:

        def __init__(self):

                self.bridge = cv_bridge.CvBridge()

                self.image_sub = rospy.Subscriber('camera/image',
                        Image, self.image_callback)

                self.cmd_vel_pub = rospy.Publisher('cmd_vel',
                        Twist, queue_size=10)

                self.twist = Twist()
                self.Nfound = True

        def image_callback(self, msg):
                self.Nfound = True
                aruco_dict = aruco.Dictionary_get(aruco.DICT_6X6_250)
                parameters = aruco.DetectorParameters_create()
                
                font = cv2.FONT_HERSHEY_SIMPLEX  # font for displaying text (below)  

                image = self.bridge.imgmsg_to_cv2(msg,desired_encoding='bgr8')
                hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

                lower_yellow = numpy.array([ 26, 43, 46])
                upper_yellow = numpy.array([34, 255, 255])

                lower_white = numpy.array([0, 0, 221])
                upper_white = numpy.array([180, 30, 255])
                
                mask1 = cv2.inRange(hsv, lower_yellow, upper_yellow)
                mask2 = cv2.inRange(hsv, lower_white, upper_white)

                h, w, d = image.shape
                homograph=numpy.array([[-2.96594441e-01, -1.21859995e+00, 2.35260957e+02],
                [ 1.22776799e-02, -1.48865241e+00, 2.33413917e+02],
                [ 6.72480233e-05, -6.79414470e-03, 1.00000000e+00]])
                result=cv2.warpPerspective(image,homograph,(w,h));
                
                
                search_top = 2*h/3
                mask1[0:search_top, 0:w] = 0
                mask2[0:search_top, 0:w] = 0

                M1 = cv2.moments(mask1)
                M2 = cv2.moments(mask2)

                if M1['m00'] > 0:
                    cx1 = int(M1['m10']/M1['m00'])
                    cy1 = int(M1['m01']/M1['m00'])

                    cx2 = int(M2['m10']/M2['m00'])
                    cy2 = int(M2['m01']/M2['m00'])

                    fpt_x = (cx1 + cx2)/2
                    fpt_y = (cy1 + cy2)/2 + 2*h/3

                    cv2.circle(image, (cx1, cy1), 10, (0,255,255), -1)
                    cv2.circle(image, (cx2, cy2), 10, (255,255,255), -1)
                    cv2.circle(image, (fpt_x, fpt_y), 10, (128,128,128), -1)

                    err = w/2 - fpt_x

                    self.twist.linear.x = 0.3
                    self.twist.angular.z = (err*90.0/160)/15
                    self.cmd_vel_pub.publish(self.twist)


                # detect and stop
                gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
                
                mtx = numpy.array([[265, 0., 160],
                                   [0., 265, 120],
                                   [0., 0., 1.]])
                dist = numpy.array([[0., 0., 0., 0., 0.]]) 
                
                corners, ids, rejectedImgPoints = aruco.detectMarkers(gray, aruco_dict, parameters=parameters)
                if ids is not None and self.Nfound:
                    rvec, tvec, _ = aruco.estimatePoseSingleMarkers(corners, 0.015, mtx, dist)
                    (rvec - tvec).any() 
                    for i in range(rvec.shape[0]):
                        aruco.drawAxis(image, mtx, dist, rvec[i, :, :], tvec[i, :, :], 0.03)
                        aruco.drawDetectedMarkers(image, corners)
                        ###### DRAW ID #####
                        cv2.putText(image, "Id: " + str(ids), (0, 64), font, 1, (0, 255, 0), 2, cv2.LINE_AA)
                    distance = tvec[0][0][2]  
                    cv2.putText(image, "distance: " + str(distance), (0, 128), font, 1, (0, 255, 0), 2, cv2.LINE_AA)
                    if distance<0.1 :
                        self.twist.angular.z = 0.0
                        self.twist.linear.x = 0.0
                        self.cmd_vel_pub.publish(self.twist)
                        time.sleep(10)
                        self.Nfound = False
                else:
                    ##### DRAW "NO IDS" #####
                    cv2.putText(image, "No Ids", (0, 64), font, 1, (0, 255, 0), 2, cv2.LINE_AA)
                    

                cv2.imshow("window1", image)
                cv2.imshow('window2',result);
                cv2.waitKey(1)

rospy.init_node('lane_follower')
follower = Follower()
rospy.spin()
