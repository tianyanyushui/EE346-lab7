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
                self.check='true'
                self.close=0
                self.tem=cv2.imread("corner.png")
                self.bridge = cv_bridge.CvBridge()

                self.Nfound = True

                self.image_sub = rospy.Subscriber('raspicam_node/image',
                        Image, self.image_callback)


                self.cmd_vel_pub = rospy.Publisher('cmd_vel',
                        Twist, queue_size=10)

                self.twist = Twist()

                self.onlyRight = True

                if self.onlyRight:
                        pts_src = numpy.array([[27, 239], [283, 239], [42, 224],[268, 224]])
                        pts_dst = numpy.array([[-20, 239],[160, 239],[-20, 220],[160,220]])

                else:
                        pts_src = numpy.array([[27, 239], [283, 239], [42, 224],[268, 224]])
                        pts_dst = numpy.array([[45, 239],[275, 239],[45, 220],[275,220]])
                
                self.homography, self.status = cv2.findHomography(pts_src, pts_dst)

        def image_callback(self, msg):

                aruco_dict = aruco.Dictionary_get(cv2.aruco.DICT_4X4_250)
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

                search_top = 160
                search_bot = 200
                search_left = 40
                search_right = 188
                gray_mask[0:search_top, 0:w] = 0
                gray_mask[0:h, 0:search_left] = 0
                gray_mask[0:h, search_right:w] = 0
                #gray_mask[search_bot:h, 0:w] = 0
                
                RGB=cv2.cvtColor(gray_mask,cv2.COLOR_GRAY2BGR)
                result=cv2.matchTemplate(RGB,self.tem, cv2.TM_SQDIFF_NORMED,-1)
                min_val,max_val,min_loc,max_loc=cv2.minMaxLoc(result)
                #print(min_val)
                if min_val<0.25 and self.check=='true':
                    self.corner=self.corner+1;
                    self.check='false'
                    self.close=time.time()
                #print(self.check,self.corner)

                curv = self.cal_curvature(gray)
                curv = max(0, numpy.mean(curv) - 0.15075)

                M = cv2.moments(gray_mask)
                time2=time.time()
                if (time2-self.close)>=4 and self.close!=0:
                    self.check='true'
                    self.close=0
                change=time2-self.time0
                #print(change)
                if M['m00'] > 0 and change>1:
                
                    cx = int(M['m10']/M['m00'])
                    cy = int(M['m01']/M['m00'])

                    fpt_x = cx  #(cx1 + cx2)/2
                    fpt_y = cy  #(cy1 + cy2)/2

                    cv2.circle(bev, (fpt_x, fpt_y), 10, (128,128,128), -1)

                    err = w/2 - fpt_x
                    theta = math.atan2(err, fpt_y) 
                    corners=cv2.cornerHarris(gray_mask,2,3,0.05) 
                    #print(corners) 

                    self.twist.angular.z =0
                    #self.twist.angular.z =   2 * theta + 0.5 * (err*90.0/160)/15
                    if err != 0:
                            #self.twist.linear.x = 0.2 - 0.5*curv
                            self.twist.linear.x = 0.0
                    else:
                            self.twist.linear.x = 0.0
                   
                    
                    self.cmd_vel_pub.publish(self.twist)
                    
                else:
                        #self.twist.angular.z = 0.1
                        self.twist.linear.x = 0.0
                        self.cmd_vel_pub.publish(self.twist)

                # detect and stop
                gray1 = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
                
                mtx = numpy.array([[265, 0., 160],
                                   [0., 265, 120],
                                   [0., 0., 1.]])
                dist = numpy.array([[0., 0., 0., 0., 0.]]) 
                
                corners, ids, rejectedImgPoints = aruco.detectMarkers(gray1, aruco_dict, parameters=parameters)
                
                if ids is not None and self.Nfound:
                    rvec, tvec, _ = aruco.estimatePoseSingleMarkers(corners, 0.015, mtx, dist)
                    
                    # from camera coeficcients
                    (rvec - tvec).any()  # get rid of that nasty numpy value array error
                    
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
                        cv2.imshow('BEV_mask1',BEV_mask1)
                        cv2.imshow('BEV_mask2',BEV_mask2)
                        cv2.imshow('window', image)
                        cv2.imshow('BEV',BEV)
                        cv2.waitKey(1)
                        time.sleep(4)
                        self.Nfound = False
                else:
                    ##### DRAW "NO IDS" #####
                    cv2.putText(image, "No Ids", (0, 64), font, 1, (0, 255, 0), 2, cv2.LINE_AA)


                #print(self.twist.angular.z, self.twist.linear.x)   
                #print(M['m00'],err,fpt_x,fpt_y,theta,cv2.THRESH_OTSU,cv2.THRESH_BINARY)
                cv2.imshow("camera view", image)
                cv2.imshow("bird eye view",bev)
                cv2.imshow("gray mask",gray_mask)
                #cv2.imshow("corners",corners)
		cv2.imwrite("camera view.png",image)
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
