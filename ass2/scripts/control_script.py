#!/usr/bin/env python3

import rospy
import os
from ass2.srv import MoveBase, GetGPSLocation, FindObjects,FindObjectsResponse, PickObject, PlaceObject
from gazebo_msgs.srv import SpawnModel, DeleteModel, DeleteModelRequest
from geometry_msgs.msg import Pose
from gazebo_msgs.msg import ModelStates, ModelState
import time, random

#env functions:
def generate_random_location(z):
    found = False
    while(not found):
        random_x = random.uniform(-0.3, 3.3)
        random_y = random.uniform(-2.1, 1.1)
        if(2.0 < random_x < 3.2 and -2.0 < random_y < -1.2 or 0.8 < random_x < 2.4 and -1.1 < random_y < 1.0 or 0.4 < random_x < 0.5 and 0.0 < random_y < 0.2):
            found = True
    return ([random_x,random_y,z])	

def spawn_model(name, file_location=os.path.expanduser('~/.gazebo/models/objects/red_ball.sdf'), spawn_location=[0.0, 0.0, 1.0]):
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

def create_scene():  
    model_state = rospy.wait_for_message("gazebo/model_states", ModelStates)
    if(len(model_state.pose) > 3):
        delete_model('blue_cube') 
    time.sleep(1) 
    spawn_locations = [generate_random_location(0.2),generate_random_location(0.2),generate_random_location(0.2),generate_random_location(0.2)]      
    for n in range(len(spawn_locations)):
        if(len(model_state.pose) > 3):
            delete_model('red_ball'+str(n)) 
        time.sleep(.5)
        spawn_model('red_ball'+str(n), os.path.expanduser('~/.gazebo/models/objects/red_ball.sdf'), spawn_locations[n])
    
    spawn_model('blue_cube', os.path.expanduser('~/.gazebo/models/objects/blue_cube.sdf'), generate_random_location(1) ) 	 	  	
      
def goal_checker():
    rospy.loginfo("goal_checker checks if the goal achived")
    balls_not_collected = []
    model_state = rospy.wait_for_message("gazebo/model_states", ModelStates)
    number_of_objects = len(model_state.pose)  - 4 # ignore: [ground_plane, room1, turtlebot3_burger, and the cube which is last] 
    #rospy.loginfo(model_state.pose)
    idx = model_state.name.index('blue_cube') # find blue_cube pose by index
    for i in range(3,idx):
        ball_name = model_state.name[i]
        balls_not_collected.append(ball_name)
    if(len(balls_not_collected) != 0):
        rospy.loginfo("goal_checker found %d balls not collected: %s" , len(balls_not_collected),balls_not_collected)
        return False
    return True		

# Define the control function
def control_script():

    # Continuously loop until all red balls are placed into the blue cube
    while not goal_checker():
        # Get the locations of all red balls
        rospy.wait_for_service('find_objects')
        try:
            find_objects_service = rospy.ServiceProxy('find_objects', FindObjects)
            response = find_objects_service()
            #rospy.loginfo(response)
            object_names = response.object_names
            object_poses = response.object_poses
            
        except rospy.ServiceException as e:
            rospy.logerr(f"Service call failed: {e}")
            return False
        # move to each ball to pick it up
        rospy.wait_for_service('move_base_service')
        try:
            rospy.logwarn('Robot command- moving towards: ' + object_names[0] + " that is in (%f, %f)", object_poses[0].position.x, object_poses[0].position.y)
            move_base_service = rospy.ServiceProxy('move_base_service', MoveBase)
            model_state = rospy.wait_for_message("gazebo/model_states", ModelStates)
            me_pose = Pose()
            me_pose = model_state.pose[2]
            if(object_poses[0].position.x > me_pose.position.x and object_poses[0].position.y > me_pose.position.y):
                result= move_base_service(object_poses[0].position.x - 0.15, object_poses[0].position.y - 0.15 ,1.0)
            elif(object_poses[0].position.x > me_pose.position.x and object_poses[0].position.y < me_pose.position.y):
                result= move_base_service(object_poses[0].position.x - 0.15, object_poses[0].position.y + 0.15 ,1.0)
            elif(object_poses[0].position.x < me_pose.position.x and object_poses[0].position.y < me_pose.position.y):
                result= move_base_service(object_poses[0].position.x + 0.15, object_poses[0].position.y + 0.15 ,1.0)
            elif(object_poses[0].position.x < me_pose.position.x and object_poses[0].position.y > me_pose.position.y):
                result= move_base_service(object_poses[0].position.x + 0.15, object_poses[0].position.y - 0.15 ,1.0)
            if result:
                rospy.loginfo("Navigation to " + object_names[0] + " that is in (%f, %f) completed", object_poses[0].position.x, object_poses[0].position.y)
        except rospy.ServiceException as e:
            rospy.logerr(f"Service call failed: {e}")
            return False
        # pick the ball
        rospy.wait_for_service('pick_object')
        try:
            rospy.logwarn('Robot command- picking up: ' + object_names[0])
            pick_object_service = rospy.ServiceProxy('pick_object', PickObject)
            object_position = [object_poses[0].position.x,object_poses[0].position.y]
            rospy.loginfo(object_position)
            response = pick_object_service(object_names[0], object_position)
            if response:
                rospy.loginfo('Successfully picked up: ' + object_names[0])
        except rospy.ServiceException as e:
            rospy.logerr(f"Service call failed: {e}")
            return False
        # move to the cube
        rospy.wait_for_service('move_base_service')
        try:
            
            move_base_service = rospy.ServiceProxy('move_base_service', MoveBase)
            idx = object_names.index('blue_cube') # find blue_cube pose by index
            rospy.logwarn('Robot command- moving towards: ' + object_names[idx])
            model_state = rospy.wait_for_message("gazebo/model_states", ModelStates)
            me_pose = Pose()
            me_pose = model_state.pose[2]
            if(object_poses[idx].position.x > me_pose.position.x and object_poses[idx].position.y > me_pose.position.y):
                result= move_base_service(object_poses[idx].position.x - 0.15, object_poses[idx].position.y - 0.15 ,1.0)
            elif(object_poses[idx].position.x > me_pose.position.x and object_poses[idx].position.y < me_pose.position.y):
                result= move_base_service(object_poses[idx].position.x - 0.15, object_poses[idx].position.y + 0.15 ,1.0)
            elif(object_poses[idx].position.x < me_pose.position.x and object_poses[idx].position.y < me_pose.position.y):
                result= move_base_service(object_poses[idx].position.x + 0.15, object_poses[idx].position.y + 0.15 ,1.0)
            elif(object_poses[idx].position.x < me_pose.position.x and object_poses[idx].position.y > me_pose.position.y):
                result= move_base_service(object_poses[idx].position.x + 0.15, object_poses[idx].position.y - 0.15 ,1.0)
            if result:
                rospy.loginfo("Navigation to blue_cube that is in (%f, %f) completed", object_poses[idx].position.x, object_poses[idx].position.y)
        except rospy.ServiceException as e:
            rospy.logerr(f"Service call failed: {e}")
            return False
        # place the ball
        rospy.wait_for_service('place_object')
        try:
            rospy.logwarn('Robot command- placing: ' + object_names[0] + ' on the ' + object_names[idx] )
            place_object_service = rospy.ServiceProxy('place_object', PlaceObject)

            place_location = [object_poses[idx].position.x,object_poses[idx].position.y]
            response = place_object_service(object_names[0], place_location)
            if response:
                rospy.loginfo("Successfully placed: %s on %s", object_names[0], object_names[idx] )
        except rospy.ServiceException as e:
            rospy.logerr(f"Service call failed: {e}")
            return False
    rospy.logwarn("finish to place all the balls in the cube")


if __name__ == "__main__":
    # Initialize ROS node
    rospy.init_node('control_script')
    rospy.loginfo("Control script node initialized.")
    rospy.sleep(5)
    # create the scene and randomally drops 4 red balls and one blue cube  
    rospy.loginfo("create the scene: sets 4 red balls and one blue cube")
    create_scene()
    control_script()
