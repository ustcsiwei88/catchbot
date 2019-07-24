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
	global balls_arr, cam_time
	pub = rospy.Publisher('/catchbot/logical_cam', LogicalCam, queue_size=10)
	rate = rospy.Rate(120)
	while not rospy.is_shutdown():
		# TODO: CREATE MODELSTATE ARRAY
		Lock.acquire()
		logical_cam_info = LogicalCam()
		logical_cam_info.timestamp = cam_time
		# print "here"
		for item in balls_arr:
			logical_cam_info.ball_positions.append(item)
		pub.publish(logical_cam_info)
		Lock.release()
		rate.sleep()


def model_states_callback(data):
	'''
	'''
	global balls_arr, cam_time
	Lock.acquire()
	l = len(data.name)
	balls_arr=[]
	cam_time = rospy.Time.now()
	for i in range(l):
		if data.name[i].startswith('ball'):
			balls_arr.append(data.pose[i].position)
	Lock.release()

def main():
	f = open(os.path.join(rospack.get_path("catchbot"), 'models/ball/model.sdf'))
	sdff = f.read()
	NUM=20
	for i in range(NUM):
		# if i % 3 == 0:
		# 	print "throwing ball %d"%(i)
		# 	x = -9.0+random.uniform(0,0.1)
		# 	y = 4.5 + random.uniform(-0.1,0.1)
		# 	z = random.uniform(0,0.7)
			
		# 	spawn_model('ball_'+str(i), sdff, "", Pose(Point(x=x,y=y,z=z), orient ), "world")
		# 	# time.sleep(0.4)
		# 	set_model_state(ModelState('ball_'+str(i), Pose(Point(x=x,y=y,z=z), orient ), Twist(Vector3(14, -7, 4),Vector3(0,0,0)), "world"))
		# elif i % 3 ==1:
		# 	print "throwing ball %d"%(i)
		# 	x = -6.0+random.uniform(0,0.1)
		# 	y = -3 + random.uniform(-0.1,0.1)
		# 	z = random.uniform(0,0.6)
		# 	spawn_model('ball_'+str(i), sdff, "", Pose(Point(x=x,y=y,z=z), orient ), "world")
		# 	# time.sleep(0.4)
		# 	set_model_state(ModelState('ball_'+str(i), Pose(Point(x=x,y=y,z=z), orient ), Twist(Vector3(8, 4, 4),Vector3(0,0,0)), "world"))
		# else:
		# 	print "throwing ball %d"%(i)
		# 	x = -3.0+random.uniform(0,0.1)
		# 	y = random.uniform(-0.1,0.1)
		# 	z = random.uniform(0.2,0.7)
		# 	spawn_model('ball_'+str(i), sdff, "", Pose(Point(x=x,y=y,z=z), orient ), "world")
		# 	# time.sleep(0.4)
		# 	set_model_state(ModelState('ball_'+str(i), Pose(Point(x=x,y=y,z=z), orient ), Twist(Vector3(4, 0, 4),Vector3(0,0,0)), "world"))
		
		print "throwing ball %d"%(i)
		x = random.uniform(-3,-10)
		y = random.uniform(-5,5)
		z = random.uniform(0,2)

		t = 0.5 + random.uniform(-0.1,0.1)
		v_x = (-0.65 - x) /t
		v_y = (random.uniform(-0.5,0.5)-y)/t + random.uniform(-y/(10*t),y/(10*t))
		v_z = (random.uniform(0.6,1.1) - z)/t + 0.5*9.8*t

		spawn_model('ball_'+str(i), sdff, "", Pose(Point(x=x,y=y,z=z), orient ), "world")
		set_model_state(ModelState('ball_'+str(i), Pose(Point(x=x,y=y,z=z), orient ), Twist(Vector3(v_x, v_y, v_z),Vector3(0,0,0)), "world"))

		time.sleep(2)
		delete_model('ball_'+str(i))
		time.sleep(1)
	rospy.signal_shutdown("Fininshed Throwing")
	# for i in range(NUM):
	# 	model_name='ball_' + str(i)
	# 	delete_model(str(model_name))

if __name__ == '__main__':
	rospy.init_node('catchbot_setup')
	rospy.wait_for_service('/gazebo/delete_model')
	rospy.wait_for_service('/gazebo/spawn_sdf_model')
	rospy.wait_for_service('/gazebo/set_model_state')
	spawn_model = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)
	set_model_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
	delete_model = rospy.ServiceProxy('/gazebo/delete_model', DeleteModel)
	cam_time=rospy.Time.now()
	rospy.Subscriber('/gazebo/model_states', ModelStates, model_states_callback)
	orient = Quaternion(0,0,0,1)
	
	thread = threading.Thread(target=logical_cam, args=())
	thread.start()
	
	mainthread = threading.Thread(target=main, args=())
	mainthread.start()
	rospy.spin()
