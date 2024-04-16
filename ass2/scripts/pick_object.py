#!/usr/bin/env python3

import rospy
import os
from geometry_msgs.msg import Pose
from gazebo_msgs.msg import ModelStates
from gazebo_msgs.srv import SpawnModel, DeleteModel, DeleteModelRequest
from tf.transformations import euler_from_quaternion
from ass2.srv import PickObject, PickObjectResponse
import time

def distance(x1, y1, x2, y2):
    dist = ((x1 - x2) ** 2 + (y1 - y2) ** 2) ** .5
    return dist

#env functions:
def spawn_model(name, file_location=os.path.expanduser('~/.gazebo/models/objects/red_ball.sdf'), spawn_location=[0.0,0.0,1.0]):
    #rospy.init_node('spawn_model', log_level=rospy.INFO)
    pose = Pose()
    pose.position.x = spawn_location[0]
    pose.position.y = spawn_location[1]
    pose.position.z = spawn_location[2]
    spawn_model_client = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)
    spawn_model_client(model_name=name,
                       model_xml=open(file_location, 'r').read(),
                       robot_namespace='/stuff', initial_pose=pose, reference_frame='world')

def delete_model(name):
    # delete model
    srv = rospy.ServiceProxy('/gazebo/delete_model', DeleteModel)
    req = DeleteModelRequest()
    req.model_name = name
    resp = srv(req)

def find_objects(req):
    # Request from Gazebo the global pose of all objects
    rospy.loginfo("Requesting Global Object Poses from Gazebo")
    model_state = rospy.wait_for_message("gazebo/model_states", ModelStates)
    number_of_objects = len(model_state.pose) - 3  # Ignore: [ground_plane, room1, turtlebot3_burger]    	
    rospy.loginfo('I found ' + str(number_of_objects) + ' Objects')
    object_names = model_state.name[3:]
    object_poses = model_state.pose[3:]
    return object_names, object_poses

def isEmpty(req):
    # Function that checks if the knapsack is empty
    rospy.loginfo("Checking for objects in the knapsack")
    model_state_objects = find_objects(req)
    number_of_objects = len(model_state_objects[1]) 
    for n in range(number_of_objects):
        if(model_state_objects[1][n].position.x == 0.0 and
           model_state_objects[1][n].position.y == -0.7):
            return False
    return True

def gps_location():
    # request a GPS like pose information from the Gazebo server
    rospy.loginfo("Requesting Global Robot Pose from Gazebo")
    model_state = rospy.wait_for_message("gazebo/model_states", ModelStates)
    me_pose = Pose()
    me_pose = model_state.pose[2]
    me_pose_angles = euler_from_quaternion([me_pose.orientation.x, me_pose.orientation.y, me_pose.orientation.z, me_pose.orientation.w])
    print('My pose is (x,y,theta): ')
    print(me_pose.position.x, me_pose.position.y, me_pose_angles[2])
    return me_pose.position.x, me_pose.position.y, me_pose_angles[2]


def handle_pick_object_request(req):
    # Function that tries to pick up an object
    object_name = req.object_name
    object_position = req.object_position
    print('Trying to pick up: ' + object_name)
    me_pose = gps_location()
    object_x = object_position[0]
    object_y = object_position[1]    
    dist = distance(me_pose[0], me_pose[1], object_x, object_y)
    
    # Check if the knapsack is empty
    if dist < 0.35 and isEmpty(req):
        delete_model(object_name)
        time.sleep(1)
        spawn_model(name=object_name, spawn_location=[0.0, -0.7, 1.0]) #put in knapsack
        time.sleep(1)
        rospy.loginfo('Successfully picked up: ' + object_name)
        return True
    else: 
        rospy.loginfo('Failed to pick up: ' + object_name + '. Need to be closer to the object to pick it')
        return False

def pick_object():
    rospy.init_node('pick_object')
    rospy.Service('pick_object', PickObject, handle_pick_object_request)
    rospy.loginfo("Ready to handle pick object requests.")
    rospy.spin()

if __name__ == "__main__":
    pick_object()
