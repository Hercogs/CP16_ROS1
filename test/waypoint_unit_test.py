#! /usr/bin/env python3

#!/usr/bin/env python3

import rospy
import actionlib
import math, time

import rostest, unittest, rosunit

from tortoisebot_waypoints.msg import WaypointActionFeedback, WaypointActionResult, WaypointActionAction, WaypointActionGoal

PKG="tortoisebot_waypoints"
NAME="waypoint_action_integration_test"

class WaypointActionClient(unittest.TestCase):
    def setUp(self):
        rospy.init_node("waypoint_action_client", anonymous=True)
        
        # Create client
        self.cli = actionlib.SimpleActionClient("tortoisebot_as", WaypointActionAction)

        # To test not succesful test, eneter (-0.2, 0.0, 0.0)
        # Test variables
        self.x = -0.2
        self.y = 0.2
        self.yaw = 0.0

        self.linear_err = 0.1
        self.yaw_err = math.pi / 40

        #self.call_action()
    
    def test_call_action(self):
        time.sleep(30) # Wait for gazebo to start
        action_status = False

        while(not self.cli.wait_for_server(timeout=rospy.Duration(1.0))):
            print("Waiting for service...")
        print("Service found")

        goal_msg = WaypointActionGoal()
        goal_msg.position.x = self.x
        goal_msg.position.y = self.y
        goal_msg.position.z = self.yaw

        print("Goal msg: ", goal_msg)

        self.cli.send_goal(goal_msg)
        res = self.cli.wait_for_result(timeout=rospy.Duration(30.0))

        # print(res)
        # print(self.cli.get_result())

        if res:
            action_status = self.cli.get_result().success
        else:
            print("Action was not finished in 30 seconds")
            self.cli.cancel_all_goals()
        
        self.assertTrue((action_status == True), "Test err! Action did not fullfill target")


    
if __name__ == "__main__":
    rostest.rosrun(PKG, NAME, WaypointActionClient)
