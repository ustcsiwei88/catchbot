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

#include<geometry_msgs/Point.h>
#include<sensor_msgs/JointState.h>
#include<trajectory_msgs/JointTrajectory.h>

#include<control_msgs/JointTrajectoryControllerState.h>

#include<trajectory_msgs/JointTrajectoryPoint.h>


#include"catchbot/LogicalCam.h"

ros::Publisher arm_joint_trajectory_publisher;
ros::Publisher gripper_joint_trajectory_publisher;
using namespace std;
// using namespace ur5ekin;

void forward(const double* q, double* T);
int inverse(const double* T, double* q_sols, double q6_des);



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

typedef pair<ros::Time, geometry_msgs::Point> ball_pose;

//TODO: USE IT
map<int,vector<ball_pose>> balls_poses;

vector<ball_pose > ball_poses;

vector<double> arm_joints_state;
double gripper_state;


void balls_state_callback(const catchbot::LogicalCamConstPtr& msg){
	// 0.1s sim sec
	// cout<<*msg<<endl;
	cout<<ball_poses.size()<<endl;
	if(msg->ball_positions.size()==0){
		ball_poses.clear();
		return;
	}
	else if(msg->ball_positions[0].z < 0.1){
		ball_poses.clear();
		return;
	}
	ball_poses.emplace_back(msg->timestamp, msg->ball_positions[0]);
	if(ball_poses.size()==7){
		auto p1 = ball_poses[1].second;
		auto p2 = ball_poses[6].second;
		cout<<p1<<endl<<p2<<endl;
		double dt = (ball_poses[6].first - ball_poses[1].first).toSec();
		double x_speed = (p2.x-p1.x)/dt;
		double y_speed = (p2.y-p1.y)/dt;
		double z_speed = (p2.z-p1.z)/dt - dt * 9.8 * 0.5;

		double t = (-0.45-p2.x)/x_speed;
		// . . . p2.x + y_speed*t
		// . . . p2.x + y_speed*t
		// . . . p2.z + z_speed*t - 0.5*g*t*t
		// . . . 1
		double *T = new double[12];
		// cout<<t<<"-=-=-=-="<<x_speed<<"-=-=-=-="<<y_speed<<"-=-=-=-="<<z_speed<<endl;
		T[0]=-1;
		T[1]=0;
		T[2]=0;
		T[3]=p2.x + x_speed*t;
		T[4]=0;
		T[5]=1;
		T[6]=0;
		T[7]=p2.y + y_speed*t;
		T[8]=0;
		T[9]=0;
		T[10]=1;
		T[11]=p2.z + z_speed*t - 0.5*9.8*t*t - 0.7;
		// T[12]=0;
		for(int i=0;i<12;i++){
			cout<<T[i]<<" ";
		}
		cout<<endl;
		double *q_sol=new double[48];
		cout<<inverse(T, q_sol, 0)<<"------"<<endl;
		if(inverse(T, q_sol, 0)){
			vector<double> tmp(6);
			for(int i=0;i<6;i++){
				tmp[i]=q_sol[i];
			}
			send_arm_to_state(tmp, 0.1);
		}
		delete[] T,q_sol;
	}

}

int cnt=0;
void arm_state_callback(const sensor_msgs::JointStateConstPtr& msg){
	
	arm_joints_state[0] = msg->position[3];
	arm_joints_state[1] = msg->position[2];
	arm_joints_state[2] = msg->position[0];
	arm_joints_state[3] = msg->position[4];
	arm_joints_state[4] = msg->position[5];
	arm_joints_state[5] = msg->position[6];

	gripper_state = msg->position[1];
}


// void foo(){
// 	while(1){
// 		vector<double> q{1.57,0,0,0,0,0}; //elbow [-pi, pi], others[-2pi, 2pi]
// 		send_arm_to_state(q);
// 		send_gripper_to_state(0.56); //0 ~ 0.8
// 	}
// }


int main(int argc, char** argv){
	ros::init(argc,argv,"catchbot_node");
	ros::NodeHandle node;
	arm_joints_state.resize(6);
	arm_joint_trajectory_publisher = node.advertise<trajectory_msgs::JointTrajectory>("/arm_controller/command", 10);
	gripper_joint_trajectory_publisher = node.advertise<trajectory_msgs::JointTrajectory>("/gripper_controller/command", 10);
	ros::Subscriber sub_cam = node.subscribe("/catchbot/logical_cam", 10, balls_state_callback);
	ros::Subscriber sub_arm = node.subscribe("/joint_states", 10, arm_state_callback);
	vector<double> q{1.57,0,0,0,0,0}; //elbow [-pi, pi], others[-2pi, 2pi]
	send_arm_to_state(q);
	send_gripper_to_state(0.56); //0 ~ 0.8
	// thread first(foo);
	ros::spin();
	return 0;
}