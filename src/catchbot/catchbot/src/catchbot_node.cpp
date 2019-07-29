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

void send_arm_to_states(const vector<vector<double>>& q, const vector<double>& t){
	trajectory_msgs::JointTrajectory msg;
	msg.joint_names.clear();
	msg.joint_names.emplace_back("shoulder_pan_joint");
	msg.joint_names.emplace_back("shoulder_lift_joint");
	msg.joint_names.emplace_back("elbow_joint");
	msg.joint_names.emplace_back("wrist_1_joint");
	msg.joint_names.emplace_back("wrist_2_joint");
	msg.joint_names.emplace_back("wrist_3_joint");

	msg.points.resize(q.size());
	for(int i=0;i<q.size();i++){
		msg.points[i].positions = q[i];
		msg.points[i].time_from_start = ros::Duration(t[i]);
	}
	arm_joint_trajectory_publisher.publish(msg);
}

void send_gripper_to_state(const double stage, const double t=0.1){
	trajectory_msgs::JointTrajectory msg;
	msg.joint_names.clear();
	msg.joint_names.emplace_back("gripper_finger1_joint");

	msg.points.resize(1);
	msg.points[0].positions.push_back(stage);

	msg.points[0].time_from_start = ros::Duration(t);
	gripper_joint_trajectory_publisher.publish(msg);
}

void send_gripper_to_states(const vector<double>& stage, const vector<double>& t){
	trajectory_msgs::JointTrajectory msg;
	msg.joint_names.clear();
	msg.joint_names.emplace_back("gripper_finger1_joint");

	msg.points.resize(stage.size());
	for(int i=0;i<stage.size();i++){
		msg.points[i].positions.push_back(stage[i]);
		msg.points[i].time_from_start = ros::Duration(t[i]);
	}
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

		double t = (-0.50-p2.x)/x_speed;
		// . . . p2.x + y_speed*t
		// . . . p2.x + y_speed*t
		// . . . p2.z + z_speed*t - 0.5*g*t*t
		// . . . 1
		double T[12]={0};
		// cout<<t<<"-=-=-=-="<<x_speed<<"-=-=-=-="<<y_speed<<"-=-=-=-="<<z_speed<<endl;
		double vz, v, vxy;
		
		// vx = x_speed;
		// vy = y_speed;
		vz = z_speed - 9.8*t;
		v = sqrt(x_speed*x_speed + y_speed*y_speed + vz*vz);

		vxy = sqrt(x_speed*x_speed + y_speed*y_speed);

		T[0]=-x_speed/v;
		T[1]=-y_speed/v;
		T[2]=-vz/v;
		T[3]=p2.x + x_speed*t;
		
		T[4]=y_speed/vxy;
		T[5]=-x_speed/vxy;
		T[6]=0;
		T[7]=p2.y + y_speed*t;

		// Singularity issues? in case x_speed=0 && y_speed=0

		T[8]=-x_speed * vz / (v * vxy);
		T[9]=-y_speed * vz / (v * vxy);
		T[10]=vxy / v;
		T[11]=p2.z + z_speed*t - 0.5*9.8*t*t - 0.7;

		// for(int i=0;i<16;i++){
		// 	cout<<T[i]<<", ";
		// 	if(i%4==3)cout<<endl;
		// }

		// cout<<endl;
		double q_sols[48];

		int sol_num = inverse(T, q_sols, 0);
		// cout<<sol_num<<"------"<<endl;
		// for(int i=0;i<sol_num ;i++){
		// 	for(int j=0;j<6;j++){
		// 		cout<< q_sols[i*6 + j]<<' ';
		// 	}
		// 	cout<<endl;
		// }
		// cout<<endl;
		if(sol_num){
			vector<double> tmp(6);
			for(int i=0;i<6;i++){
				tmp[i]=q_sols[i];
			}
			double t_arrival = t - (ros::Time::now() - ball_poses[6].first).toSec();
			cout<<"t_arrival = "<<t_arrival<<endl;
			send_arm_to_state(tmp, t_arrival/2);
			// send_arm_to_states(vector<vector<double>>{tmp, tmp, tmp}, vector<double>{t/2, t*3/2, 2*t});
			// double t_remain = ros::Time::now() - ball_poses[6].first;

			send_gripper_to_states(vector<double>{0.12,0.52,0.52,0.12}, 
				vector<double>{t_arrival/2,t_arrival,0.5+t_arrival*2,0.5+t_arrival*3});
		}
		
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