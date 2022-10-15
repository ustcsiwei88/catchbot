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
#include<geometry_msgs/PoseStamped.h>
#include<sensor_msgs/JointState.h>
#include<trajectory_msgs/JointTrajectory.h>
#include<control_msgs/JointTrajectoryControllerState.h>
#include<trajectory_msgs/JointTrajectoryPoint.h>

#define PI 3.1415926535
#include"catchbot/LogicalCam.h"

#include <ur_rtde/rtde_control_interface.h>
#include <ur_rtde/rtde_receive_interface.h>
#define IP_ADDR "172.19.97.157"

#define JUST_FOLLOW

using namespace std;
using namespace ur_rtde;

ros::Publisher arm_joint_trajectory_publisher;
ros::Publisher gripper_joint_trajectory_publisher;
vector<double> q_default{1.5031, -1.1882, 1.7874, 2.1394, -1.4560, 0};

RTDEControlInterface rtde_control(IP_ADDR);

// using namespace ur5ekin;
std::mutex mtx;

void forward(const double* q, double* T);
int inverse(const double* T, double* q_sols, double q6_des);
vector<double> arm_joints_state;

void arm_state_callback(){
	RTDEReceiveInterface rtde_receive(IP_ADDR);
	while(1){
		mtx.lock();
		arm_joints_state = rtde_receive.getActualQ();
		mtx.unlock();
		this_thread::sleep_for(0.005s);
	}
}

void send_arm_to_state(vector<double>& q, double t=0.1){
	cout << "Moving to ";
	for(double d: q) cout << d << ' '; cout << endl;
	
	cout << "Moving to degree ";
	for(double d: q) cout << d / M_PI * 180 << ' '; cout << endl;
	
	rtde_control.moveJ(q, 0.4, 0.4); // vel acc
}

inline double sm_step(double s, double g){
	if(s < g){
		if(g > s + .04) return s + .04;
		else return g;
	}else{
		if(g < s - .04) return s - .04;
		else return g;
	}
}

void servo_arm_to_state(vector<double>& q, double t=0.1){
	mtx.lock();
	for(int i=0;i<6;i++){
		q[i] = sm_step(arm_joints_state[i], q[i]);
	}
	mtx.unlock();
	// cout << "Servoing to ";
	// for(double d: q) cout << d << ' '; cout << endl;
	
	// cout << "Servoing to degree ";
	// for(double d: q) cout << d / M_PI * 180 << ' '; cout << endl;

	rtde_control.servoJ(q, 1, 1, 1. / 120.0, 0.1, 300); // vel acc lookahead_time gain
}


/*
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
*/

void send_gripper_to_state(const double stage, const double t=0.1){
	/*
	trajectory_msgs::JointTrajectory msg;
	msg.joint_names.clear();
	msg.joint_names.emplace_back("gripper_finger1_joint");

	msg.points.resize(1);
	msg.points[0].positions.push_back(stage);

	msg.points[0].time_from_start = ros::Duration(t);
	gripper_joint_trajectory_publisher.publish(msg);
	*/
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

double gripper_state;

#ifdef JUST_FOLLOW
void balls_state_callback(const geometry_msgs::PoseStampedConstPtr& msg){
	// 0.1s sim sec
	// cout<<*msg<<endl;
	// cout<<ball_poses.size()<<endl;
	// if(msg->ball_positions.size()==0){
	// 	ball_poses.clear();
	// 	return;
	// }
	// else if(msg->ball_positions[0].z < 0.1){
	// 	ball_poses.clear();
	// 	return;
	// }
	// if(msg->pose.position.x < 0.5) return;
	double x = msg->pose.position.x;
	double y = msg->pose.position.y;
	double z = msg->pose.position.z;
	ball_poses.emplace_back(msg->header.stamp, geometry_msgs::Point());
	// coarsely change into robot frame
	ball_poses.back().second.x = -y + .2;
	ball_poses.back().second.y = x - 5.89; 
	ball_poses.back().second.z = z - 0.8;
	
	if(ball_poses.size() >= 7){
		auto p1 = ball_poses[1].second;
		auto p2 = ball_poses.back().second;
		// cout << p1 << endl << p2 << endl;
		// double dt = (ball_poses[6].first - ball_poses[1].first).toSec();
		// double x_speed = (p2.x-p1.x) / dt;
		// double y_speed = (p2.y-p1.y) / dt;
		// double z_speed = (p2.z-p1.z) / dt - dt * 9.8 * 0.5;

		// double t = (-0.55 - p2.y)/y_speed;
		
		// . . . p2.x + y_speed*t
		// . . . p2.x + y_speed*t
		// . . . p2.z + z_speed*t - 0.5*g*t*t
		// . . . 1
		double T[12]={0};
		// cout<< "t = " << t << ", x_peed = " << x_speed << ", y_speed = " << y_speed
		// 	<< ", z_speed = "<< z_speed <<endl;
		// double vz, v, vxy;
		
		// vx = x_speed;
		// vy = y_speed;
		// vz = z_speed - 9.8 * t;
		// v = sqrt(x_speed*x_speed + y_speed*y_speed + vz*vz);

		// vxy = sqrt(x_speed*x_speed + y_speed*y_speed);

		// just let it point towards y neg
		T[0] = 0;
		T[4] = -1;
		T[8] = 0;
		T[3] = clamp(p2.x, -0.4, 0.4);
		
		T[1] = 0;
		T[5] = 0;
		T[9] = 1;
		// T[7] = p2.y;
		T[7] = -0.65;

		T[2] = -1;
		T[6] = 0;
		T[10]= 0;
		T[11]= clamp(p2.z, 0.2, 0.7); 
		// if(p2.z < 0.2) return;
		// for(int i=0;i<12;i++){
		// 	cout << T[i] << ", ";
		// 	if(i%4==3) cout << endl;
		// }

		// cout<<endl;
		double q_sols[48];

		int sol_num = inverse(T, q_sols, 0);
		cout << "Solution count " << sol_num << endl;
		// for(int i=0;i<sol_num ;i++){
		// 	for(int j=0;j<6;j++){
		// 		cout<< q_sols[i*6 + j]<<' ';
		// 	}
		// 	cout<<endl;
		// }
		// cout<<endl;

		if(sol_num){
			// vector<int> tmp(sol_num);
			// for(int i=0;i<n;i++){
			// 	tmp[i]=i;
			// }
			mtx.lock();
			auto cur_conf = arm_joints_state;
			mtx.unlock();
			cur_conf = q_default; // use default for now
			vector<double> tmp1(sol_num, 0);
			int id=0;
			for(int i=0;i<sol_num;i++){
				for(int j=0;j<6;j++){
					if(q_sols[i*6+j] - cur_conf[j] > PI){
						q_sols[i*6+j] -= 2*PI;
					}
					else if(q_sols[i*6+j] - cur_conf[j] < -PI){
						q_sols[i*6+j] += 2*PI;
					}
				}
				for(int j=0;j<6;j++)
					tmp1[i] = max(tmp1[i], fabs(cur_conf[j] - q_sols[i*6+j]));
				if(tmp1[id] > tmp1[i]){
					id=i;
				}
			}
			vector<double> tmp(6);
			for(int i=0; i<6; i++){
				tmp[i]=q_sols[i + id*6];
			}
			// cout << "min diff = " << tmp1[id] << endl;
			// double t_arrival = t - (ros::Time::now() - ball_poses[6].first).toSec();
			// cout<<"t_arrival = "<<t_arrival<<endl;
			
			servo_arm_to_state(tmp);
			// send_arm_to_states(vector<vector<double>>{tmp, tmp, tmp}, vector<double>{t/2, t*3/2, 2*t});
			// double t_remain = ros::Time::now() - ball_poses[6].first;

			// send_gripper_to_states({0.12,0.67,0.67,0.12}, 
			// 	{t_arrival/2,t_arrival,0.5+t_arrival*2,0.5+t_arrival*3});
		}
		
	}

}
#else
void balls_state_callback(const geometry_msgs::PoseStampedConstPtr& msg){
	// 0.1s sim sec
	// cout<<*msg<<endl;
	// cout<<ball_poses.size()<<endl;
	// if(msg->ball_positions.size()==0){
	// 	ball_poses.clear();
	// 	return;
	// }
	// else if(msg->ball_positions[0].z < 0.1){
	// 	ball_poses.clear();
	// 	return;
	// }
	if(msg->pose.position.x < 0.5) return;
	double x = msg->pose.position.x;
	double y = msg->pose.position.y;
	double z = msg->pose.position.z;
	ball_poses.emplace_back(msg->header.stamp, geometry_msgs::Point());
	// coarsely change into robot frame
	ball_poses.back().second.x = -y + .2;
	ball_poses.back().second.y = x - 5.89; 
	ball_poses.back().second.z = z - 0.8;
	
	if(ball_poses.size()==7){
		auto p1 = ball_poses[1].second;
		auto p2 = ball_poses[6].second;
		// cout << p1 << endl << p2 << endl;
		double dt = (ball_poses[6].first - ball_poses[1].first).toSec();
		double x_speed = (p2.x-p1.x) / dt;
		double y_speed = (p2.y-p1.y) / dt;
		double z_speed = (p2.z-p1.z) / dt - dt * 9.8 * 0.5;

		double t = (-0.55 - p2.y)/y_speed;
		
		// . . . p2.x + y_speed*t
		// . . . p2.x + y_speed*t
		// . . . p2.z + z_speed*t - 0.5*g*t*t
		// . . . 1
		double T[12]={0};
		cout<< "t = " << t << ", x_peed = " << x_speed << ", y_speed = " << y_speed
			<< ", z_speed = "<< z_speed <<endl;
		double vz, v, vxy;
		
		// vx = x_speed;
		// vy = y_speed;
		vz = z_speed - 9.8 * t;
		v = sqrt(x_speed*x_speed + y_speed*y_speed + vz*vz);

		vxy = sqrt(x_speed*x_speed + y_speed*y_speed);

		T[0]=-x_speed/v;
		T[4]=-y_speed/v;
		T[8]=-vz/v;
		T[3]=p2.x + x_speed*t;
		
		T[1]=y_speed/vxy;
		T[5]=-x_speed/vxy;
		T[9]=0;
		T[7]=p2.y + y_speed*t;

		T[2]=-x_speed * vz / (v * vxy);
		T[6]=-y_speed * vz / (v * vxy);
		T[10]=vxy / v;
		T[11]=p2.z + z_speed*t - 0.5*9.8*t*t;

		for(int i=0;i<12;i++){
			cout<<T[i]<<", ";
			if(i%4==3)cout<<endl;
		}

		// cout<<endl;
		double q_sols[48];

		int sol_num = inverse(T, q_sols, 0);
		cout << "Solution count " << sol_num << endl;
		// for(int i=0;i<sol_num ;i++){
		// 	for(int j=0;j<6;j++){
		// 		cout<< q_sols[i*6 + j]<<' ';
		// 	}
		// 	cout<<endl;
		// }
		// cout<<endl;

		if(sol_num){
			// vector<int> tmp(sol_num);
			// for(int i=0;i<n;i++){
			// 	tmp[i]=i;
			// }
			mtx.lock();
			auto cur_conf = arm_joints_state;
			mtx.unlock();
			cur_conf = q_default; // use default for now
			vector<double> tmp1(sol_num, 0);
			int id=0;
			for(int i=0;i<sol_num;i++){
				for(int j=0;j<6;j++){
					if(q_sols[i*6+j] - cur_conf[j] > PI){
						q_sols[i*6+j] -= 2*PI;
					}
					else if(q_sols[i*6+j] - cur_conf[j] < -PI){
						q_sols[i*6+j] += 2*PI;
					}
				}
				for(int j=0;j<3;j++)
					tmp1[i] = max(tmp1[i], fabs(cur_conf[j] - q_sols[i*6+j]));
				if(tmp1[id] > tmp1[i]){
					id=i;
				}
			}
			vector<double> tmp(6);
			for(int i=0; i<6; i++){
				tmp[i]=q_sols[i + id*6];
			}
			cout << "min diff = " << tmp1[id] << endl;
			double t_arrival = t - (ros::Time::now() - ball_poses[6].first).toSec();
			cout<<"t_arrival = "<<t_arrival<<endl;
			send_arm_to_state(tmp, t_arrival/2);
			// send_arm_to_states(vector<vector<double>>{tmp, tmp, tmp}, vector<double>{t/2, t*3/2, 2*t});
			// double t_remain = ros::Time::now() - ball_poses[6].first;

			send_gripper_to_states({0.12,0.67,0.67,0.12}, 
				{t_arrival/2,t_arrival,0.5+t_arrival*2,0.5+t_arrival*3});
		}
		
	}

}
#endif

int cnt=0;
// void arm_state_callback(const sensor_msgs::JointStateConstPtr& msg){
	
// 	arm_joints_state[0] = msg->position[3];
// 	arm_joints_state[1] = msg->position[2];
// 	arm_joints_state[2] = msg->position[0];
// 	arm_joints_state[3] = msg->position[4];
// 	arm_joints_state[4] = msg->position[5];
// 	arm_joints_state[5] = msg->position[6];

// 	gripper_state = msg->position[1];
// }


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
	cout << "Catch bot started" << endl;
	arm_joints_state.resize(6);
	arm_joint_trajectory_publisher = node.advertise<trajectory_msgs::JointTrajectory>("/arm_controller/command", 10);
	gripper_joint_trajectory_publisher = node.advertise<trajectory_msgs::JointTrajectory>("/gripper_controller/command", 10);
	// ros::Subscriber sub_cam = node.subscribe("/catchbot/logical_cam", 10, balls_state_callback);
#ifdef JUST_FOLLOW
	ros::Subscriber sub_cam = node.subscribe("/vrpn_client_node/RigidBody02/pose", 10, balls_state_callback);
#else
	ros::Subscriber sub_cam = node.subscribe("/vrpn_client_node/RigidBody01/pose", 10, balls_state_callback);
#endif
	// ros::Subscriber sub_arm = node.subscribe("/joint_states", 10, arm_state_callback);
	thread t1(arm_state_callback);
	// vector<double> q{1.5031, -1.1882, 1.7874, 2.1394, -1.4560, 0}; //elbow [-pi, pi], others[-2pi, 2pi]
	// rtde_control = RTDEControlInterface (IP_ADDR);
	send_arm_to_state(q_default);
	send_gripper_to_state(0.56); // 0 ~ 0.8
	// thread first(foo);
	ros::AsyncSpinner spinner(4); // Use 4 threads
    spinner.start();
    ros::waitForShutdown();
	return 0;
}
