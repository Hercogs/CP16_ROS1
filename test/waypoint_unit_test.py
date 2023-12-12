#! /usr/bin/env python3

#!/usr/bin/env python3

import rospy
import actionlib
import math, time

import rostest, unittest, rosunit

from tortoisebot_waypoints.msg import WaypointActionFeedback, WaypointActionResult, WaypointActionAction, WaypointActionGoal
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, Point
from tf import transformations

PKG="tortoisebot_waypoints"
NAME="waypoint_action_integration_test"

class WaypointActionClient(unittest.TestCase):
    def setUp(self):
        rospy.init_node("waypoint_action_client", anonymous=True)
        
        # Create action client
        self.cli = actionlib.SimpleActionClient("tortoisebot_as", WaypointActionAction)
        self.action_status = False

        self.sub_odom = rospy.Subscriber('/odom', Odometry, self.clbk_odom)
        self.position = Point()
        self.yaw = 0.0

        # To test not succesful test, eneter (-0.2, 0.0, 0.0)
        # Test variables
        self.target_x = -0.3
        self.target_y = 0.2
        self.target_yaw = 0.0

        self.linear_err = 0.1
        self.yaw_err = 0.15

        self.call_action()
        time.sleep(0.5)
        
    
    def call_action(self):
        #time.sleep(30) # Wait for gazebo to start
        # Wait for gazebo to start
        rospy.wait_for_service('/gazebo/reset_world')

        time.sleep(5)
        
        while(not self.cli.wait_for_server(timeout=rospy.Duration(1.0))):
            print("Waiting for service...")
        print("Service found")

        goal_msg = WaypointActionGoal()
        goal_msg.position.x = self.target_x
        goal_msg.position.y = self.target_y
        goal_msg.position.z = self.target_yaw

        print("Goal msg: ", goal_msg)

        self.cli.send_goal(goal_msg)
        res = self.cli.wait_for_result(timeout=rospy.Duration(40.0))

        # print(res)
        # print(self.cli.get_result())

        if res:
            self.action_status = self.cli.get_result().success
        else:
            print("Action was not finished in 40 seconds")
            self.cli.cancel_all_goals()

    
    def clbk_odom(self, msg):
        #rospy.loginfo("Clb odom")
        # position
        self.position = msg.pose.pose.position

        # yaw
        quaternion = (
            msg.pose.pose.orientation.x,
            msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z,
            msg.pose.pose.orientation.w)
        euler = transformations.euler_from_quaternion(quaternion)
        self.yaw = euler[2]
    
    def test_action_finish(self):
        # Test action finish
        self.assertTrue((self.action_status == True), "Action did not finisged sucessfully")

    def test_yaw_err(self):
        # Test yaw
        rospy.loginfo("Current yaw: " +  str(self.yaw) + ", Target yaw: " + str(self.target_yaw))
        yaw_diff = self.yaw - self.target_yaw
        self.assertTrue((abs(yaw_diff) < self.yaw_err), "Yaw angle is not in error limits")

    def test_linear_err(self):
        # Test position
        d_x = self.position.x - self.target_x
        d_y = self.position.y - self.target_y
        d_xy = math.sqrt(d_x**2 + d_y**2)
        rospy.loginfo("d_xy: " + str(d_xy))
        self.assertTrue((d_xy < self.linear_err), "Linear error is too large")


    
if __name__ == "__main__":
    rostest.rosrun(PKG, NAME, WaypointActionClient)
