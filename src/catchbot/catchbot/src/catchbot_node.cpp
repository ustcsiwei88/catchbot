#include<vector>
#include<string>
#include<deque>
#include<queue>
#include<algorithm>
#include<cmath>
#include<map>
#include<set>

#include<thread>

#include<ros/ros.h>

#include<sensor_msgs/JointState.h>
#include<trajectory_msgs/JointTrajectory.h>

ros::Publisher arm_joint_trajectory_publisher;
ros::Publisher gripper_joint_trajectory_publisher;
using namespace std;
void send_arm_to_state(vector<double>& q, double t=0.1){
	trajectory_msgs::JointTrajectory msg;
	msg.joint_names.clear();
	msg.joint_names.emplace_back("shoulder_pan_joint");
	msg.joint_names.emplace_back("shoulder_lift_joint");
	msg.joint_names.emplace_back("elbow_joint");
	msg.joint_names.emplace_back("wrist_1_joint");
	msg.joint_names.emplace_back("wrist_2_joint");
	msg.joint_names.emplace_back("wrist_3_joint");

	msg.points.resize(1);
	msg.points[0].positions = q;

	msg.points[0].time_from_start = ros::Duration(t);
	arm_joint_trajectory_publisher.publish(msg);
}

void send_gripper_to_state(double stage, double t=0.1){
	trajectory_msgs::JointTrajectory msg;
	msg.joint_names.clear();
	msg.joint_names.emplace_back("gripper_finger1_joint");

	msg.points.resize(1);
	msg.points[0].positions.push_back(stage);

	msg.points[0].time_from_start = ros::Duration(t);
	gripper_joint_trajectory_publisher.publish(msg);
}

void foo(){
	while(1){
		vector<double> q{1.57,0,0,0,0,0}; //elbow [-pi, pi], others[-2pi, 2pi]
		send_arm_to_state(q);
		send_gripper_to_state(0.56); //0 ~ 0.8
	}
}

int main(int argc, char** argv){
	ros::init(argc,argv,"catchbot_node");
	ros::NodeHandle node;
	arm_joint_trajectory_publisher = node.advertise<trajectory_msgs::JointTrajectory>("/arm_controller/command", 10);
	gripper_joint_trajectory_publisher = node.advertise<trajectory_msgs::JointTrajectory>("/gripper_controller/command", 10);
	vector<double> q{1.57,0,0,0,0,0}; //elbow [-pi, pi], others[-2pi, 2pi]
	send_arm_to_state(q);
	send_gripper_to_state(0.56); //0 ~ 0.8
	thread first(foo);
	ros::spin();
	return 0;
}