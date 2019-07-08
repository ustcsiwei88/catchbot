#include<vector>
#include<string>
#include<deque>
#include<queue>
#include<algorithm>
#include<cmath>
#include<map>
#include<set>

#include<ros/ros.h>

#include<sensor_msgs/JointState.h>
#include<trajectory_msgs/JointTrajectory.h>

void send_arm_to_state(){

}

void send_gripper_to_state(){

}

int main(int argc, char** argv){
	ros::init(argc,argv,"catchbot_node");
	ros::NodeHandle node;
	ros::Publisher arm_joint_trajectory_publisher = node.advertise<trajectory_msgs::JointTrajectory>("/arm_controller/command", 10);
	ros::Publisher gripper_joint_trajectory_publisher = node.advertise<trajectory_msgs::JointTrajectory>("/gripper_controller/command", 10);

}