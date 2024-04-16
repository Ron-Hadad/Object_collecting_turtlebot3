#!/usr/bin/env python3

import rospy
from gazebo_msgs.msg import ModelStates
from gazebo_msgs.srv import SpawnModel, DeleteModel, DeleteModelRequest
from geometry_msgs.msg import Pose
from ass2.srv import FindObjects, FindObjectsResponse

def distance(x1, y1, x2, y2):
    dist = ((x1 - x2) ** 2 + (y1 - y2) ** 2) ** .5
    return dist

def handle_get_objects(request):
    # Request from Gazebo the global pose of all objects
    rospy.loginfo("Requesting Global Object Poses from Gazebo")
    model_state = rospy.wait_for_message("gazebo/model_states", ModelStates)
    number_of_objects = len(model_state.pose) - 3  # Ignore: [ground_plane, room1, turtlebot3_burger]    	
    rospy.loginfo('I found ' + str(number_of_objects) + ' Objects')
    object_names = model_state.name[3:]
    object_poses = model_state.pose[3:]
    return object_names, object_poses


def find_objects():
    rospy.init_node('find_objects')
    rospy.Service('find_objects', FindObjects, handle_get_objects)
    rospy.loginfo("Ready to provide object poses.")
    rospy.spin()

if __name__ == "__main__":
    find_objects()