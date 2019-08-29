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
#include<geometry_msgs/Wrench.h>
#include<geometry_msgs/Vector3.h>
#include<sensor_msgs/JointState.h>
#include<trajectory_msgs/JointTrajectory.h>

#include<control_msgs/JointTrajectoryControllerState.h>

#include<trajectory_msgs/JointTrajectoryPoint.h>

#include"catchbot/LogicalCam.h"

using namespace std;

typedef pair<ros::Time, geometry_msgs::Point> ball_pose;
vector<ball_pose> ball_poses;
typedef pair<ros::Time, geometry_msgs::Vector3> ball_velocity;
vector<ball_velocity> ball_velocities;

ros::Publisher air_friction_publisher;

int step = 0;


void send_air_friction(vector<double>& force){
	geometry_msgs::Wrench msg;
	msg.force.x = force[0];
	msg.force.y = force[1];
	msg.force.z = force[2];
	msg.torque.x = 0;
	msg.torque.y = 0;
	msg.torque.z = 0;
	
	air_friction_publisher.publish(msg);
}

void balls_state_callback(const catchbot::LogicalCamConstPtr& msg){
	if(msg->ball_positions.size()==0){
		ball_poses.clear();
		ball_velocities.clear();
		return;
	}
	else if(msg->ball_positions[0].z < 0.1){
		ball_poses.clear();
		ball_velocities.clear();
		return;
	}
	ball_poses.emplace_back(msg->timestamp, msg->ball_positions[0]);

	vector<double> force(3);
	geometry_msgs::Vector3 velocity;
	step = ball_poses.size();
	
	if(step > 10){
		auto p1 = ball_poses[step-1].second;
		auto p2 = ball_poses[step-2].second;
		double dt = (ball_poses[step-1].first - ball_poses[step-2].first).toSec();
		cout << "time: " << ball_poses[step-1].first.toSec() << endl;

		velocity.x = (p1.x - p2.x)/dt;
		velocity.y = (p1.y - p2.y)/dt;
		velocity.z = (p1.z - p2.z)/dt;

		cout << "v: " << velocity.x << " " << velocity.y << " " << velocity.z << endl;
		ball_velocities.emplace_back(msg->timestamp, velocity);

		double v = sqrt(velocity.x*velocity.x + velocity.y*velocity.y + velocity.z*velocity.z);

		double alpha = 0.02;
		force[0] = -alpha * v * velocity.x;
		force[1] = -alpha * v * velocity.y;
		force[2] = -alpha * v * velocity.z;

		cout << "f: " << force[0] << " " << force[1] << " " << force[2] << endl << endl;
		send_air_friction(force);
	}
}


int main(int argc, char** argv){
	ros::init(argc,argv,"catchbot_air_firction");
	ros::NodeHandle node;
	air_friction_publisher = node.advertise<geometry_msgs::Wrench>("/catchbot/air_friction", 1000);
	ros::Subscriber sub_cam = node.subscribe("/catchbot/logical_cam", 10, balls_state_callback);

	ros::spin();
	return 0;
}