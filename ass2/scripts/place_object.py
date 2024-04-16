#!/usr/bin/env python3

import rospy
import os
from gazebo_msgs.msg import ModelStates
from gazebo_msgs.srv import SpawnModel, DeleteModel, DeleteModelRequest
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import Pose
from ass2.srv import PlaceObject

#env functions:
def spawn_model(name, file_location= os.path.expanduser('~/.gazebo/models/objects/red_ball.sdf'), spawn_location=[0.0,0.0,1.0]):
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

def goal_checker():
    rospy.logwarn("goal_checker checks if the goal achived")
    balls_not_collected = []
    model_state = rospy.wait_for_message("gazebo/model_states", ModelStates)
    number_of_objects = len(model_state.pose)  - 4 # ignore: [ground_plane, room1, turtlebot3_burger, and the cube which is last] 
    cube_pose = model_state.pose 
    for i in range(number_of_objects):
        ball_pose = model_state.pose[3+i]
        ball_name = model_state.name[3+i]
        if(ball_pose.x != cube_pose.x or ball_pose.y != cube_pose.y):
            balls_not_collected.append([ball_name +"at ("+ ball_pose.x +","+ ball_pose.y +")   "])
    if(len(balls_not_collected) != 0):
        rospy.logwarn("goal_checker found"+ len(balls_not_collected) +"balls not collected:" + balls_not_collected)    	
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


def distance(x1, y1, x2, y2):
    dist = ((x1 - x2) ** 2 + (y1 - y2) ** 2) ** .5
    return dist

def find_objects(request):
    # Request from Gazebo the global pose of all objects
    rospy.loginfo("Requesting Global Object Poses from Gazebo")
    model_state = rospy.wait_for_message("gazebo/model_states", ModelStates)
    number_of_objects = len(model_state.pose) - 3  # Ignore: [ground_plane, room1, turtlebot3_burger]    	
    rospy.loginfo('I found ' + str(number_of_objects) + ' Objects')
    object_names = model_state.name[3:]
    object_poses = model_state.pose[3:]
    return object_names, object_poses

def isPicked(req):
    # Function that checks if the correct object is in the knapsack
    rospy.loginfo("Checking if this object is in the knapsack")
    model_state_objects = find_objects(req)
    number_of_objects = len(model_state_objects[1]) 
    for n in range(number_of_objects):
        if (model_state_objects[1][n].position.x == 0.0 and
            model_state_objects[1][n].position.y == -0.7):
            if (model_state_objects[0][n] == req.object_name):
                return True
    rospy.loginfo("There are no objects in the knapsack")
    return False

def handle_place_object_request(req):
    # Function that places an object
    object_name = req.object_name
    place_location = req.place_location
    rospy.loginfo('Placing object: ' + object_name)
    me_pose = gps_location()
    dist2 = distance(me_pose[0], me_pose[1], place_location[0], place_location[1])
    
    # Check if the object is picked and if the distance is within range
    if isPicked(req) and dist2 < 0.35:
        delete_model(object_name)
        spawn_model(name=object_name, spawn_location=[place_location[0], place_location[1], 0.3])
        rospy.loginfo('Object placed successfully')
        return True
    else:
        rospy.loginfo('Failed to place the object. Need to be closer to the location or object not picked')
        return False

def place_object():
    rospy.init_node('place_object')
    rospy.Service('place_object', PlaceObject, handle_place_object_request)
    rospy.loginfo("Ready to handle place object requests.")
    rospy.spin()

if __name__ == "__main__":
    place_object()
