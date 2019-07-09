#!/usr/bin/env python

import rospy, tf
from gazebo_msgs.srv import DeleteModel, SpawnModel, SetModelState, GetModelState
from gazebo_msgs.msg import ModelState
from geometry_msgs.msg import *
import time
import os
import random
import rospkg 

rospack = rospkg.RosPack()

if __name__ == '__main__':
	rospy.init_node('catchbot_setup')
	rospy.wait_for_service('/gazebo/delete_model')
	rospy.wait_for_service('/gazebo/spawn_sdf_model')
	rospy.wait_for_service('/gazebo/delete_model')
	spawn_model = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)
	set_model_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
	delete_model = rospy.ServiceProxy('/gazebo/delete_model', DeleteModel)
	orient = Quaternion(0,0,0,1)
	print rospack.get_path("catchbot")
	f = open(os.path.join(rospack.get_path("catchbot"), 'models/ball/model.sdf'))
	sdff = f.read()
	for i in range(10):
		x = random.uniform(0,0.1)
		y = random.uniform(0,0.1)
		z = random.uniform(0,0.1)
		spawn_model('ball_'+str(i), sdff, "", Pose(Point(x=x,y=y,z=z), orient ), "world")
		time.sleep(0.4)
		set_model_state(ModelState('ball_'+str(i), Pose(Point(x=x,y=y,z=z), orient ), Twist(Vector3(2, 0, 4),Vector3(0,0,0)), "world"))
		time.sleep(0.6)
	for i in range(10):
		model_name='ball_'+str(i)
		delete_model(str(model_name))
