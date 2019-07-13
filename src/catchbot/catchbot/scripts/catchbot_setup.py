#!/usr/bin/env python

import rospy, tf
import threading
from gazebo_msgs.srv import DeleteModel, SpawnModel, SetModelState, GetModelState
from gazebo_msgs.msg import ModelState, ModelStates
from geometry_msgs.msg import *
import time
import os
import random
import rospkg 
from catchbot.msg import LogicalCam
'''
Should use plugin instead.
'''

rospack = rospkg.RosPack()

Lock = threading.Lock()

balls_arr = []


def logical_cam():
	'''
	Balls position publisher
	'''
	global balls_arr
	pub = rospy.Publisher('/catchbot/logical_cam', LogicalCam, queue_size=10)
	rate = rospy.Rate(10)
	while not rospy.is_shutdown():
		# TODO: CREATE MODELSTATE ARRAY
		Lock.acquire()
		logical_cam_info = LogicalCam()
		logical_cam_info.timestamp = rospy.Time.now()
		for item in balls_arr:
			logical_cam_info.ball_positions.append(item)
		pub.publish(logical_cam_info)
		Lock.release()
		rate.sleep()


def model_states_callback(data):
	'''
	'''
	global balls_arr
	Lock.acquire()
	l = len(data.name)
	balls_arr=[]
	for i in range(l):
		if data.name[i].startswith('ball'):
			balls_arr.append(data.pose[i].position)
	Lock.release()

def main():
	f = open(os.path.join(rospack.get_path("catchbot"), 'models/ball/model.sdf'))
	sdff = f.read()
	for i in range(10):
		x = -2+random.uniform(0,0.1)
		y = random.uniform(0,0.1)
		z = random.uniform(0,0.1)
		spawn_model('ball_'+str(i), sdff, "", Pose(Point(x=x,y=y,z=z), orient ), "world")
		time.sleep(0.4)
		set_model_state(ModelState('ball_'+str(i), Pose(Point(x=x,y=y,z=z), orient ), Twist(Vector3(2, 0, 7),Vector3(0,0,0)), "world"))
		time.sleep(0.6)
	for i in range(10):
		model_name='ball_' + str(i)
		delete_model(str(model_name))

if __name__ == '__main__':
	rospy.init_node('catchbot_setup')
	rospy.wait_for_service('/gazebo/delete_model')
	rospy.wait_for_service('/gazebo/spawn_sdf_model')
	rospy.wait_for_service('/gazebo/set_model_state')
	spawn_model = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)
	set_model_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
	delete_model = rospy.ServiceProxy('/gazebo/delete_model', DeleteModel)

	rospy.Subscriber('/gazebo/model_states', ModelStates, model_states_callback)
	orient = Quaternion(0,0,0,1)
	
	thread = threading.Thread(target=logical_cam, args=())
	thread.start()
	
	mainthread = threading.Thread(target=main, args=())
	mainthread.start()
	rospy.spin()
