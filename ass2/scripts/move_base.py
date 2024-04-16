#!/usr/bin/env python3

import rospy
from std_srvs.srv import Empty, EmptyResponse
from geometry_msgs.msg import Pose
from gazebo_msgs.msg import ModelStates
from ass2.srv import MoveBase
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import actionlib

# Define the movebase_client function
def handle_navigation_request(req):
    # Moves the robot collision-free to a x, y, theta pose (must be valid/reachable in the map)
    rospy.loginfo("started move")
    client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
    server_started = client.wait_for_server(rospy.Duration(5.0))
    if not server_started:
        rospy.logerr("The move_base action server is not available!")
    else:
        rospy.loginfo("Connection to move_base action server established.")
    client.wait_for_server()
    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = "map"
    goal.target_pose.header.stamp = rospy.Time.now()
    goal.target_pose.pose.position.x = req.x
    goal.target_pose.pose.position.y = req.y
    goal.target_pose.pose.position.z = 0.0
    goal.target_pose.pose.orientation.w = req.w
    rospy.loginfo("Goal sent: %s", goal)
    client.send_goal(goal)
    wait = client.wait_for_result()
    rospy.loginfo(wait)
    if not wait:
        rospy.logerr("Action server not available!")
        rospy.signal_shutdown("Action server not available!")
    else:
        rospy.loginfo("Navigation to (%f, %f, %f) completed", req.x, req.y, req.w)
        if(client.get_result()):
            rospy.loginfo(client.get_result())
            return True

def move_base():
    rospy.init_node('move_base_service')
    rospy.Service('move_base_service', MoveBase, handle_navigation_request)
    rospy.loginfo("Ready to navigate to goal.")
    rospy.spin()

if __name__ == "__main__":
    move_base()
