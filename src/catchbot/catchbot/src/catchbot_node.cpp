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

#define ABS(x) (x)>=0 ? (x):(-(x))

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

void send_arm_to_states(vector<vector<double>>& q, vector<double> t){
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

void send_gripper_to_state(double stage, double t=0.1){
	trajectory_msgs::JointTrajectory msg;
	msg.joint_names.clear();
	msg.joint_names.emplace_back("gripper_finger1_joint");

	msg.points.resize(1);
	msg.points[0].positions.push_back(stage);

	msg.points[0].time_from_start = ros::Duration(t);
	gripper_joint_trajectory_publisher.publish(msg);
}

void send_gripper_to_states(vector<double> stage, vector<double> t){
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

void q_switch(double *a, double *b){
	double *t = new double[6];
	for(int i = 0; i<6; i++){
		t[i] = a[i];
		a[i] = b[i];
		b[i] = t[i];
	}
	delete t;
}
void q_sort_based_on_max_rotation(double *q_sol, double *q_sol_max_rotation, int sol_num){
	for(int j = 0; j<sol_num-1; j++){
		int min=j;
		int ptr=j;
		for(; ptr<sol_num;ptr++){
			if(q_sol_max_rotation[ptr]<q_sol_max_rotation[min]) min = ptr;
		}
		q_switch(q_sol+6*j,q_sol+6*min);
		double t = q_sol_max_rotation[j];
		q_sol_max_rotation[min] = q_sol_max_rotation[j];
		q_sol_max_rotation[j] = q_sol_max_rotation[min];
	}
	for(int i = 0;i<sol_num;i++){
		for(int j = 0;j<6;j++){
			cout << q_sol[i*6+j] << " ";
		}
		cout << endl;
	}
}
void q_sort(double *q_sol, double *q_sol_, int sol_num){
	double *q_sol_max_rotation = new double[sol_num];
	for(int i = 0; i<sol_num;i++){
		q_sol_max_rotation[i] = q_sol_[6*i];
		for(int j = 0;j<6;j++){
			if(q_sol_[6*i+j] > q_sol_max_rotation[i]) q_sol_max_rotation[i] = q_sol_[6*i+j];
		}
	}
	q_sort_based_on_max_rotation(q_sol,q_sol_max_rotation,sol_num);
}

void balls_state_callback(const catchbot::LogicalCamConstPtr& msg){
	// 0.1s sim sec
	// cout<<*msg<<endl;
	//cout<<ball_poses.size()<<endl;
	if(msg->ball_positions.size()==0){
		ball_poses.clear();
		return;
	}
	else if(msg->ball_positions[0].z < 0.1){
		ball_poses.clear();
		return;
	}
	ball_poses.emplace_back(msg->timestamp, msg->ball_positions[0]);
	if(ball_poses.size()==6){
		auto p1 = ball_poses[1].second;
		auto p2 = ball_poses[5].second;
		//cout<<p1<<endl<<p2<<endl;
		double dt = (ball_poses[5].first - ball_poses[1].first).toSec();
		double x_speed = (p2.x-p1.x)/dt;
		double y_speed = (p2.y-p1.y)/dt;
		double z_speed = (p2.z-p1.z)/dt - dt * 9.8 * 0.5;
		
		cout << "Flying time 1:" << dt << endl;

		double t = (-0.65-p2.x)/x_speed;
		cout << "Flying time 2:" << t << endl;
		double v_x = x_speed;
		double v_y = y_speed;
		double v_z = z_speed - 9.8 * t;
		double v_xy = sqrt(v_x*v_x + v_y*v_y);
		double v = sqrt(v_x*v_x + v_y*v_y + v_z*v_z);
		//cout << v_x << "===" << v_z << "===" << v << endl;
		// . . . p2.x + y_speed*t
		// . . . p2.x + y_speed*t
		// . . . p2.z + z_speed*t - 0.5*g*t*t
		// . . . 1
		double *T = new double[12];
		// cout<<t<<"-=-=-=-="<<x_speed<<"-=-=-=-="<<y_speed<<"-=-=-=-="<<z_speed<<endl;
		T[0]=-v_x/v;
		T[1]=-v_y/v_xy;
		T[2]=(v_z*v_x)/(v*v_xy);
		T[3]=p2.x + x_speed*t;
		T[4]=-v_y/v;
		T[5]=v_x/v_xy;
		T[6]=(v_y*v_z)/(v*v_xy);
		T[7]=p2.y + y_speed*t;
		T[8]=-v_z/v;
		T[9]=0;
		T[10]=-v_xy/v;
		T[11]=p2.z + z_speed*t - 0.5*9.8*t*t;
		cout<<endl;
		double *q_sol=new double[48];
		int sol_num = inverse(T, q_sol, 0);
		cout<< sol_num <<"------"<<endl;
		if(sol_num){
			double *q_sol_ = new double[48];
			for(int i=0; i<6*sol_num;i++) q_sol_[i] = ABS(q_sol[i]-arm_joints_state[i%6]);
			cout << "before" << endl;
			cout << arm_joints_state[0] << " " << arm_joints_state[1] << " " << arm_joints_state[2] << " " << arm_joints_state[3] << " " << arm_joints_state[4] << " " << arm_joints_state[5] << endl;
			cout << endl;
			// for(int i = 0;i<sol_num;i++){
			// 	for(int j = 0;j<6;j++){
			// 		cout << q_sol[i*6+j] << " ";
			// 	}
			// 	cout << endl;
			// }
			// cout << "q rotate" << endl;
			// for(int i = 0;i<sol_num;i++){
			// 	for(int j = 0;j<6;j++){
			// 		cout << q_sol_[i*6+j] << " ";
			// 	}
			// 	cout << endl;
			// }
			int i = 0;
			for(int j = 0;j<6;j++){
				cout << q_sol[i*6+j] << " ";
			}
			cout << endl;
			for(int j = 0;j<6;j++){
				cout << q_sol_[i*6+j] << " ";
			}
			cout << endl;
			cout << endl;
			// cout << "sorted " << endl;
			// q_sort(q_sol,q_sol_,sol_num);

			vector<double> tmp(6);
			for(int i=0;i<6;i++){
				tmp[i]=q_sol[i];
			}
			send_arm_to_state(tmp, t/4);
			// send_arm_to_states(vector<vector<double>>{tmp, tmp, tmp}, vector<double>{t/2, t*3/2, 2*t});
			send_gripper_to_states(vector<double>{0.12,0.52,0.52,0.12}, vector<double>{t/2,t,t*3/2,t*2});
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
	//cout << arm_joints_state[0] << " " << arm_joints_state[1] << " " << arm_joints_state[2] << " " << arm_joints_state[3] << " " << arm_joints_state[4] << " " << arm_joints_state[5] << endl;

	gripper_state = msg->position[1];
}


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