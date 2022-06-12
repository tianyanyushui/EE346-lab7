#!/usr/bin/env python  
import rospy  
import actionlib  
import collections
from actionlib_msgs.msg import *  
from geometry_msgs.msg import Pose, PoseWithCovarianceStamped, Point, Quaternion, Twist  
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal    
from math import pow, sqrt  
  
class MultiNav():  
    def __init__(self):  
        rospy.init_node('MultiNav', anonymous=True)  
        rospy.on_shutdown(self.shutdown)  
  
        self.rest_time = rospy.get_param("~rest_time", 3)
	self.rest_time_mid = rospy.get_param("~rest_time", 0.5)
  
        self.fake_test = rospy.get_param("~fake_test", True)  
  
        # Goal state return values  
        goal_states = ['PENDING', 'ACTIVE', 'PREEMPTED','SUCCEEDED',  
                       'ABORTED', 'REJECTED','PREEMPTING', 'RECALLING',   
                       'RECALLED','LOST']  


        locations = collections.OrderedDict()
        locations['point-2'] = Pose(Point(0.155, 0.059, 0.000), Quaternion(0.000, 0.000, -0.0208198192148, 0.999783244072))
        locations['point-2-3-1'] = Pose(Point(1.73499941826, -1.84999990463, 0.000), Quaternion(0.000, 0.000, -0.645207403619, 0.76400746483))
        locations['point-2-3-2'] = Pose(Point(1.89852333069, -3.92018890381, 0.000), Quaternion(0.000, 0.000, -0.988215959737, 0.15306605411))
        locations['point-3'] = Pose(Point(-0.237, -4.01199965668, 0.000), Quaternion(0.000, 0.000, -0.0389732523175,  0.999240254195))
        locations['point-4'] = Pose(Point(3.74499893188, -4.26999998093, 0.000), Quaternion(0.000, 0.000, 0.999059706664, 0.0433555362052))
        locations['point-4-1-1'] = Pose(Point(1.98499941826, -3.41999912262, 0.000), Quaternion(0.000, 0.000, 0.707106796641, 0.707106765732))
        locations['point-4-1-2'] = Pose(Point(2.20499968529, -0.810000240803, 0.000), Quaternion(0.000, 0.000, -0.0312045236032, 0.999513020279))
        locations['point-1'] = Pose(Point(4.212, -0.240, 0.000), Quaternion(0.000, 0.000, 0.000, 1.000)) #map3



        # Publisher to manually control the robot (e.g. to stop it)  
        self.cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=5)  

        # Subscribe to the move_base action server  
        self.move_base = actionlib.SimpleActionClient("move_base", MoveBaseAction)  
        rospy.loginfo("Waiting for move_base action server...")  
  
        # Wait 60 seconds for the action server to become available  
        self.move_base.wait_for_server(rospy.Duration(60))  
        rospy.loginfo("Connected to move base server")  

        # A variable to hold the initial pose of the robot to be set by the user in RViz  
        initial_pose = PoseWithCovarianceStamped()

        # Variables to keep track of success rate, running time, and distance traveled  
        n_goals = 0  
        n_successes = 0  
        i = 0  
        distance_traveled = 0  
        start_time = rospy.Time.now()  
        running_time = 0  
        location = ""

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
          
        rospy.loginfo("Starting navigation test")  
  	num = 0
        # Begin the navigation loop and visit a sequence of locations  
        for location in locations.keys():  
  	    num+=1
            rospy.loginfo("Updating current pose.")  
            distance = sqrt(pow(locations[location].position.x  
                           - initial_pose.pose.pose.position.x, 2) +  
                           pow(locations[location].position.y -  
                           initial_pose.pose.pose.position.y, 2))  
            initial_pose.header.stamp = ""  
  
            # Store the last location for distance calculations  
            last_location = location  
  
            # Increment the counters  
            i += 1  
            n_goals += 1  
  
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
            finished_within_time = self.move_base.wait_for_result(rospy.Duration(420))  
  
            # Check for success or failure  
            if not finished_within_time:  
                self.move_base.cancel_goal()  
                rospy.loginfo("Timed out achieving goal")  
            else:  
                state = self.move_base.get_state()  
                if state == GoalStatus.SUCCEEDED:  
                    rospy.loginfo("Goal succeeded!")  
                    n_successes += 1  
                    distance_traveled += distance  
                else:  
                    rospy.loginfo("Goal failed with error code: " + str(goal_states[state]))  
  
            # How long have we been running?  
            running_time = rospy.Time.now() - start_time  
            running_time = running_time.secs / 60.0  
  
            # Print a summary success/failure, distance traveled and time elapsed  
            rospy.loginfo("Success so far: " + str(n_successes) + "/" +  
                          str(n_goals) + " = " + str(100 * n_successes/n_goals) + "%")  
            rospy.loginfo("Running time: " + str(trunc(running_time, 1)) +  
                          " min Distance: " + str(trunc(distance_traveled, 1)) + " m")  
	    if num == 2 or num == 3 or num == 6 or num==7:
		rospy.sleep(self.rest_time_mid) 
            else:
                rospy.sleep(self.rest_time)  
            print(num)
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
        MultiNav()  
        rospy.spin()  
    except rospy.ROSInterruptException:  
        rospy.loginfo("AMCL navigation test finished.")  
