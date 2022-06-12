import rospy  
import actionlib  
import collections
from actionlib_msgs.msg import *  
from geometry_msgs.msg import Pose, PoseWithCovarianceStamped, Point, Quaternion, Twist  
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal    
from math import pow, sqrt
import cv2.aruco as aruco
import rospy, cv2, cv_bridge, numpy, math
from sensor_msgs.msg import Image
from sound_play.msg import SoundRequest
from sound_play.libsoundplay import SoundClient
import time

  
class RobotRun():  
    def __init__(self):  
        rospy.init_node('RobotRun', anonymous=True)  
        rospy.on_shutdown(self.shutdown)

        # Important parameter -- judge navigation or line following
        # 0 -- navigation; 1 -- line following
        self.switch = 0

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
	self.markID = -1
	self.soundhandle = SoundClient() 
  
        self.rest_time = rospy.get_param("~rest_time", 0.1)
        self.rest_time_mid = rospy.get_param("~rest_time_mid", 0.05)
  
        self.fake_test = rospy.get_param("~fake_test", True)  
  
        # Goal state return values  
        self.goal_states = ['PENDING', 'ACTIVE', 'SUCCEEDED',  
                       'ABORTED', 'REJECTED', 'RECALLING',   
                       'RECALLED','LOST']
	
	# Subscribe to the camera node
	self.bridge = cv_bridge.CvBridge()
	self.image_sub = rospy.Subscriber('raspicam_node/image', Image, self.image_callback)
        pts_src = numpy.array([[27, 239], [283, 239], [42, 224],[268, 224]])
	pts_dst = numpy.array([[-20, 239],[160, 239],[-20, 30],[160, 30]])
        # pts_dst = numpy.array([[-20, 239],[160, 239],[-20, 205],[160,205]])
        self.homography, self.status = cv2.findHomography(pts_src, pts_dst)
       
	# Publisher to manually control the robot (e.g. to stop it)  
        self.cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=10) 

        # Subscribe to the move_base action server
        self.move_base = actionlib.SimpleActionClient("move_base", MoveBaseAction)  
        rospy.loginfo("Waiting for move_base action server...")  
  
        # Wait 60 seconds for the action server to become available  
        self.move_base.wait_for_server(rospy.Duration(60))  
        rospy.loginfo("Connected to move base server") 

	self.soundhandle = SoundClient() 

        # A variable to hold the initial pose of the robot to be set by the user in RViz  
        initial_pose = PoseWithCovarianceStamped()

        # Variables to run the robot
        self.start_time = rospy.Time.now().to_sec()
        running_time = 0

        # Get the initial pose from the user  
        rospy.loginfo("Click on the map in RViz to set the intial pose...")  
        rospy.wait_for_message('initialpose', PoseWithCovarianceStamped)  
        self.last_location = Pose()  
        rospy.Subscriber('initialpose', PoseWithCovarianceStamped, self.update_initial_pose)
        
        # Make sure the initial pose is correct  
        keyinput = int(input("Input 0 to continue,or reset the initialpose!\n"))
        while keyinput != 0:
            rospy.loginfo("Click on the map in RViz to set the intial pose...")  
            rospy.wait_for_message('initialpose', PoseWithCovarianceStamped)  
            rospy.Subscriber('initialpose', PoseWithCovarianceStamped, self.update_initial_pose)
            keyinput = int(input("Input 0 to continue,or reget the initialpose!")) 
        while initial_pose.header.stamp == "":  
            rospy.sleep(1)
          
        rospy.loginfo("Starting running")  
        
        #Go to start point
        rospy.loginfo("Updating current pose.")
        initial_pose.header.stamp = ""
        self.goal = MoveBaseGoal()  
        self.goal.target_pose.pose = Pose(Point(3.278465271, -1.01158380508, 0.000), Quaternion(0.000, 0.000, 1, 0.01)) # start-point
        self.goal.target_pose.header.frame_id = 'map'  
        self.goal.target_pose.header.stamp = rospy.Time.now()
        rospy.loginfo("Going to: " + "point-start")
	self.move_base.send_goal(self.goal)
        finished_within_time = self.move_base.wait_for_result(rospy.Duration(240))
        if not finished_within_time:  
                self.move_base.cancel_goal()  
                rospy.loginfo("Timed out achieving goal")  
        else:  
                state = self.move_base.get_state()  
                if state == GoalStatus.SUCCEEDED:  
                    rospy.loginfo("Ready to start!")  
                    self.switch = 1
                else:  
                    rospy.loginfo("Goal failed with error code: " + str(self.goal_states[state]))
        
        #Pause navigation
        initial_pose.header.stamp = ""
        #self.move_base.cancel_goal()

        # Start line following
        self.twist = Twist()
        self.corner = 0
        self.tem1 = cv2.imread("corner1.png")
	#self.tem2 = cv2.imread("corner2.png")
	#self.tem3 = cv2.imread("corner3.png")
        self.check = True
        self.close = 0
	self.lefton = True
        self.alreadyTurn = False
	self.visitP2 = False
	self.back = False
	self.time_change = 0.0

        self.lf_start_time = rospy.Time.now().to_sec()
        self.lf_end_time = 0
        

    def image_callback(self, msg): 
        
        image = self.bridge.imgmsg_to_cv2(msg,desired_encoding='bgr8')

        h, w, d = image.shape
        bev = cv2.warpPerspective(image, self.homography, (w,h)) 
        gray = cv2.cvtColor(bev, cv2.COLOR_RGB2GRAY)
        # gray[gray<10] = 255     
	flag, gray_thresh = cv2.threshold(gray,125,255,cv2.THRESH_BINARY)
        # flag, gray_thresh = cv2.threshold(gray,0,255,cv2.THRESH_BINARY+cv2.THRESH_OTSU)
        gray_mask = 255-gray_thresh
        gray_mask[gray<10] = 0

        search_top = 40
        search_bot = 200
        search_left = 40  #40 for right; 60 for left
        search_right = 260  #260 for right; 280 for left
        #gray_mask[0:search_top, 0:w] = 0
        gray_mask[0:h, 0:search_left] = 0
        gray_mask[0:h, search_right:w] = 0
        #gray_mask[search_bot:h, 0:w] = 0

        RGB = cv2.cvtColor(gray_mask,cv2.COLOR_GRAY2BGR)

        this_aruco_dictionary = cv2.aruco.Dictionary_get(self.ARUCO_DICT["DICT_6X6_250"])
        this_aruco_parameters = cv2.aruco.DetectorParameters_create()
        (corners, ids, rejected) = cv2.aruco.detectMarkers(image, this_aruco_dictionary, parameters=this_aruco_parameters)
        if len(corners) > 0:
	    for marker_id in ids:
		#if self.markID != marker_id:
            	print("Find Aruco! ID: ", str(marker_id[0]))
		self.soundhandle.say(str(marker_id[0]), 'voice_kal_diphone', 20)
		self.markID = marker_id
	
        
        # The main codes for line following
        if self.switch == 1:
            now_time = rospy.Time.now().to_sec()
            self.time_change = now_time - self.lf_start_time
            print(self.time_change)

            result1 = cv2.matchTemplate(RGB, self.tem1, cv2.TM_SQDIFF_NORMED,-1)
            min_val1, max_val1, min_loc1, max_loc1 = cv2.minMaxLoc(result1)
            #result2 = cv2.matchTemplate(RGB, self.tem2, cv2.TM_SQDIFF_NORMED,-1)
            #min_val2, max_val2, min_loc2, max_loc2 = cv2.minMaxLoc(result2)
            #result3 = cv2.matchTemplate(RGB, self.tem3, cv2.TM_SQDIFF_NORMED,-1)
            #min_val3, max_val3, min_loc3, max_loc3 = cv2.minMaxLoc(result3)
	    # print(min_val)

            M = cv2.moments(gray_mask)
                
            if self.time_change <= 4.75:
                self.twist.linear.x=0.22

            if (now_time - self.close) >= 6.2 and self.close != 0:
                self.check = True
                self.close = 0
                
            if self.time_change > 4.75 and self.time_change < 6.15:
                self.twist.angular.z = -1
                self.twist.linear.x = 0.0

            if min_val1 < 0.39 and self.check == True: # or min_val2 < 0.25 or min_val3 < 0.25
                self.corner = self.corner+1;
                self.check = False
                self.close = rospy.Time.now().to_sec()
                #print(min_val, self.check,self.corner)

            if M['m00'] > 0 and self.time_change >= 6.15:
		
		if self.corner == 0:
		    self.corner = 1
                
                cx = int(M['m10']/M['m00'])
                cy = int(M['m01']/M['m00'])

                fpt_x = cx  #(cx1 + cx2)/2
                fpt_y = cy  #(cy1 + cy2)/2

                cv2.circle(bev, (fpt_x, fpt_y), 10, (128,128,128), -1)

                err = w/2 - fpt_x
                theta = math.atan2(err, fpt_y) 

                if self.corner == 2 and self.lefton:
                    self.twist.linear.x = 0.22
                    self.twist.angular.z = 0.0
                    self.cmd_vel_pub.publish(self.twist)
                    rospy.sleep(2.51)
                    self.twist.linear.x = 0.15
                    self.twist.angular.z = 0.45
                    self.cmd_vel_pub.publish(self.twist)
                    rospy.sleep(3.7)
                    self.lefton=False

                if self.corner == 3:
                    if self.alreadyTurn == False:
                            self.lf_end_time = rospy.Time.now().to_sec()
                            self.alreadyTurn = True    
                    
                    turn_time = rospy.Time.now().to_sec()
                    dur = turn_time - self.lf_end_time
                    
                    if dur <= 4.5:
                        self.twist.linear.x = 0.22
                        self.twist.angular.z = 0.0
  			#self.cmd_vel_pub.publish(self.twist)
                    elif dur <= 4.7:
                        self.twist.linear.x = 0.0
                        self.twist.angular.z = -1.05
                        self.cmd_vel_pub.publish(self.twist)
                        rospy.sleep(1.45)
                    else:
                        self.twist.linear.x = 0.22
                        self.twist.angular.z = 0.0
                        self.cmd_vel_pub.publish(self.twist)
                        rospy.sleep(4.5)
                        self.twist.linear.x = 0.0
                        self.twist.angular.z = 0.0
                        self.cmd_vel_pub.publish(self.twist)
                        self.switch = 0
			self.move_to_p1()
                		
                else:
		    if self.back == True:
	            	self.twist.linear.x = 0.22
	        	self.twist.angular.z = 0.0
			self.cmd_vel_pub.publish(self.twist)
			time.sleep(1.0)
			self.back = False
		    else:
                    	self.twist.linear.x = 0.22
			if self.time_change < 35.5:
                    	    self.twist.angular.z =   0.2 * (err*90.0/160)/15 + 0.8 * theta
			else:
			    self.twist.angular.z =   0.5 * (err*90.0/160)/15 + 1.2 * theta
			#if len(corners) > 0:
			    #self.twist.linear.x = 0.22
			    #self.twist.angular.z = 0.0
			    #self.cmd_vel_pub.publish(self.twist)
			    #rospy.sleep(3) 
                    	#self.cmd_vel_pub.publish(self.twist) 
                    	#print(4*theta,  (err*90.0/160)/15, self.twist.linear.x)
                    	#print(self.twist.linear.x,self.twist.angular.z)
		
		if self.visitP2 == False and self.time_change >= 15.5:
		    self.switch = 0
		    print(now_time, self.time_change)  
		    self.move_to_p2()
		    self.visitP2 = True
	        
            self.cmd_vel_pub.publish(self.twist)
	    print(self.corner, self.check)
	    #print(self.twist.linear.x,self.twist.angular.z)
		    
        cv2.imshow("gray mask",gray_mask)
        cv2.waitKey(1)
    

    def move_to_p2(self):
	if self.switch == 0 and self.visitP2 == False:
	    rospy.loginfo("Updating current pose.")
	    initial_pose = PoseWithCovarianceStamped()
            initial_pose.header.stamp = ""
            self.goal = MoveBaseGoal()  
            self.goal.target_pose.pose = Pose(Point(0.050649347305,  -0.020245552063, 0.000), Quaternion(0.000, 0.000, -0.701190399398, 0.712974069509)) #0.224999815226 0.224999815226
            self.goal.target_pose.header.frame_id = 'map'  
            self.goal.target_pose.header.stamp = rospy.Time.now()
            rospy.loginfo("Going to: " + "point-2")
	    self.move_base.send_goal(self.goal)
            finished_within_time = self.move_base.wait_for_result(rospy.Duration(240))
            if not finished_within_time:  
                self.move_base.cancel_goal()  
                rospy.loginfo("Timed out achieving goal")  
            else:  
                state = self.move_base.get_state()  
                if state == GoalStatus.SUCCEEDED:  
                    rospy.loginfo("Success!") 
		    rospy.sleep(self.rest_time)
		    #self.switch = 1
		    #self.visitP2 = True
                    #self.twist.linear.x = 2.0
                    #self.twist.angular.z = 0.0
                    #self.cmd_vel_pub.publish(self.twist)
		    #rospy.sleep(1.0)
                    #self.twist.linear.x = 0.0
                    #self.twist.angular.z = 0.0
                    #self.cmd_vel_pub.publish(self.twist) 
                else:  
                    rospy.loginfo("Goal failed with error code: " + str(goal_states[state]))

            initial_pose.header.stamp = ""
            self.goal = MoveBaseGoal()  
            self.goal.target_pose.pose = Pose(Point(0.27500000596, -0.380000144243, 0.000), Quaternion(0.000, 0.000, -0.701473116394, 0.712695914803)) #-0.701473116394, 0.712695914803
            self.goal.target_pose.header.frame_id = 'map'  
            self.goal.target_pose.header.stamp = rospy.Time.now()
            rospy.loginfo("Going to: " + "line")
	    self.move_base.send_goal(self.goal)
            finished_within_time = self.move_base.wait_for_result(rospy.Duration(240))
            if not finished_within_time:  
                self.move_base.cancel_goal()  
                rospy.loginfo("Timed out achieving goal")  
            else:  
                state = self.move_base.get_state()  
                if state == GoalStatus.SUCCEEDED:  
                    rospy.loginfo("Ready to line following")  
		    self.visitP2 = True
		    self.twist.linear.x = 0.0
		    self.twist.angular.z = -0.5
		    self.cmd_vel_pub.publish(self.twist)
                    rospy.sleep(0.3)
		    self.twist.angular.z = 0.0
		    self.cmd_vel_pub.publish(self.twist)
		    self.back = True
		    self.switch = 1
		    

    def move_to_p1(self):
	locations = collections.OrderedDict()
        locations['point-1'] = Pose(Point(4.24305183411, -0.261348903179, 0.000), Quaternion(0.000, 0.000, -0.707106796641, 0.707106765732))

        if self.switch == 0:
	    initial_pose = PoseWithCovarianceStamped()
            # If can continue navigation, begin the navigation loop and visit a sequence of locations  
            for location in locations.keys(): 
                rospy.loginfo("Updating current pose.")
                initial_pose.header.stamp = "" 
  
                # Set up the next goal location  
                self.goal = MoveBaseGoal()  
                self.goal.target_pose.pose = locations[location]
                self.goal.target_pose.header.frame_id = 'map'  
                self.goal.target_pose.header.stamp = rospy.Time.now()  
  
                # Let the user know where the robot is going next  
                rospy.loginfo("Going to: " + str(location))  
                # Start the robot toward the next location  
                self.move_base.send_goal(self.goal)  
  
                # Allow 5 minutes to get there  
                finished_within_time = self.move_base.wait_for_result(rospy.Duration(240))  
  
                # Check for success or failure  
                if not finished_within_time:  
                    self.move_base.cancel_goal()  
                    rospy.loginfo("Timed out achieving goal")  
                else:  
                    state = self.move_base.get_state()  
                    if state == GoalStatus.SUCCEEDED:  
                        rospy.loginfo("Goal succeeded!")
                    else:  
                        rospy.loginfo("Goal failed with error code: " + str(goal_states[state]))  
  
		rospy.sleep(self.rest_time)
	    self.restart()


    def restart(self):
	initial_pose = PoseWithCovarianceStamped()

        #Back to start point
        rospy.loginfo("Updating current pose.")
        initial_pose.header.stamp = ""
        self.goal = MoveBaseGoal()  
        self.goal.target_pose.pose = Pose(Point(3.278465271, -1.01158380508, 0.000), Quaternion(0.000, 0.000, 1, 0.01))
        self.goal.target_pose.header.frame_id = 'map'  
        self.goal.target_pose.header.stamp = rospy.Time.now()
        rospy.loginfo("Going to: " + "point-start")
	self.move_base.send_goal(self.goal)
        finished_within_time = self.move_base.wait_for_result(rospy.Duration(240))
        if not finished_within_time:  
                self.move_base.cancel_goal()  
                rospy.loginfo("Timed out achieving goal")  
        else:  
                state = self.move_base.get_state()  
                if state == GoalStatus.SUCCEEDED:  
                    rospy.loginfo("Ready to start!")  
                    self.switch = 1
                else:  
                    rospy.loginfo("Goal failed with error code: " + str(self.goal_states[state]))
        
        #Pause navigation
        initial_pose.header.stamp = ""
        #self.move_base.cancel_goal()

        # Start line following
        self.twist = Twist()
        self.corner = 0
        self.check = True
	self.lefton = True
        self.close = 0
        self.alreadyTurn = False
        self.markID = -1

        self.lf_start_time = rospy.Time.now().to_sec()
        self.lf_end_time = 0.0
	self.time_change = 0.0
	rospy.sleep(0.3)	
	self.visitP2 = False
	

    def update_initial_pose(self, initial_pose):  
        self.initial_pose = initial_pose
  
    def shutdown(self):  
        rospy.loginfo("Stopping the robot...")  
        self.move_base.cancel_goal()  
        rospy.sleep(1)  
        self.cmd_vel_pub.publish(Twist())  
        rospy.sleep(1)


        
def trunc(f, n):
  
    # Truncates/pads a float f to n decimal places without rounding  
    slen = len('%.*f' % (n, f))  
    return float(str(f)[:slen])  
  
if __name__ == '__main__':  
    try:  
        RobotRun()  
        rospy.spin()  
    except rospy.ROSInterruptException:  
        rospy.loginfo("Running test finished.")  
