#!/usr/bin/env python3

import rospy
import actionlib

from tortoisebot_waypoints.msg import WaypointActionFeedback, WaypointActionResult, WaypointActionAction, WaypointActionGoal


class WaypointActionClient():
    def __init__(self):
        rospy.init_node("waypoint_action_client", anonymous=True)
        
        # Create client
        self._cli = actionlib.SimpleActionClient("tortoisebot_as", WaypointActionAction)

        self.call_action()
    
    def call_action(self):
        while(not self._cli.wait_for_server(timeout=rospy.Duration(1.0))):
            print("Waiting for service...")
        print("Service found")

        goal_msg = WaypointActionGoal()
        goal_msg.position.x = -0.2
        goal_msg.position.y = 0.2
        goal_msg.position.z = 0.0

        print(goal_msg)

        self._cli.send_goal(goal_msg)
        res = self._cli.wait_for_result(timeout=rospy.Duration(25.0))

        print(res)
        print(self._cli.get_result())

        if res:
            print(self._cli.get_result().success)


    
if __name__ == "__main__":
    node = WaypointActionClient()
    rospy.spin()
