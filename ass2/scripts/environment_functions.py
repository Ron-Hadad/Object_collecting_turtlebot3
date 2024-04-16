#!/usr/bin/env python3
from gazebo_msgs.srv import SpawnModel, DeleteModel, DeleteModelRequest
import rospy
from geometry_msgs.msg import Pose
from gazebo_msgs.msg import ModelStates, ModelState
import time, random

def initialize_environment():
    rospy.init_node('turtlebot3_env', anonymous=True, log_level=rospy.WARN)

def spawn_model(name, file_location='/home/guy/.gazebo/models/objects/red_ball.sdf', spawn_location=[0.0,0.0,1.0]):
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
    delete_model('blue_cube') 
    time.sleep(1) 
    spawn_locations = [generate_random_location(0.2),generate_random_location(0.2),generate_random_location(0.2),generate_random_location(0.2)]      
    for n in range(len(spawn_locations)):
        delete_model('red_ball'+str(n)) 
        time.sleep(.5)
        spawn_model('red_ball'+str(n), '/home/guy/.gazebo/models/objects/red_ball.sdf', spawn_locations[n])
    
    spawn_model('blue_cube', '/home/guy/.gazebo/models/objects/blue_cube.sdf', generate_random_location(1) ) 		
      

def generate_random_location(z):
    found = False
    while(not found):
        random_x = random.uniform(-0.3, 3.3)
        random_y = random.uniform(-2.1, 1.1)
        if(1.9 < random_x < 3.3 and -2.1 < random_y < -1.1 or 0.7 < random_x < 2.5 and -1 < random_y < 1.1 or 0.3 < random_x < 0.6 and -0.23 < random_y < 0.3):
            found = True
    return ([random_x,random_y,z])		
      
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


if __name__ == '__main__':
	initialize_environment()
	create_scene()	

	

