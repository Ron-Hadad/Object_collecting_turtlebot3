#!/usr/bin/env python3

import rospy
from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import Pose
from tf.transformations import euler_from_quaternion
from ass2.srv import GetGPSLocation, GetGPSLocationResponse

def gps_location_callback(req):
    # request a GPS-like pose information from the Gazebo server
    rospy.loginfo("Requesting Global Robot Pose from Gazebo")
    model_state = rospy.wait_for_message("gazebo/model_states", ModelStates)
    me_pose = Pose()
    me_pose = model_state.pose[2]  # Assuming your robot's pose is at index 2
    me_pose_angles = euler_from_quaternion([me_pose.orientation.x, me_pose.orientation.y, me_pose.orientation.z, me_pose.orientation.w])
    rospy.loginfo('My pose is (x,y,theta): ')
    rospy.loginfo(f"{me_pose.position.x}, {me_pose.position.y}, {me_pose_angles[2]}")
    return GetGPSLocationResponse(me_pose.position.x, me_pose.position.y, me_pose_angles[2])

def gps_location_server():
    rospy.init_node('current_pose')
    s = rospy.Service('current_pose', GetGPSLocation, gps_location_callback)
    rospy.loginfo("GPS Location Server Ready.")
    rospy.spin()

if __name__ == "__main__":
    gps_location_server()