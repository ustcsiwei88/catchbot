#include<vector>
#include<string>
#include<deque>
#include<queue>
#include<algorithm>
#include<cmath>
#include<map>
#include<set>
#include<thread>

#define PI 3.1415926535

#include <ur_rtde/rtde_control_interface.h>
#include <ur_rtde/rtde_receive_interface.h>
#include <ur_rtde/robotiq_gripper.h>
// #define MOCAP_IP_ADDR "169.254.30.167"
#define MOCAP_IP_ADDR "192.168.1.15"
#define IP_ADDR "192.168.1.18"
#define RATE 120

#define HOR_GRP

#include <NatNetTypes.h>
#include <NatNetCAPI.h>
#include <NatNetClient.h>

// #define JUST_FOLLOW
// #define JUST_FOLLOW_GRIPPER

using namespace std;
using namespace ur_rtde;


#ifdef HOR_GRP
vector<double> q_default{0, -1.1882, 1.7874, 2.1394, -1.4560, 0};
#else
vector<double> q_default{0, -1.1882, 1.7874, 2.1394, -1.4560, 1.5708};
#endif

RTDEControlInterface rtde_control(IP_ADDR);
ur_rtde::RobotiqGripper gripper(IP_ADDR, 63352, true);

void _WriteHeader(FILE* fp, sDataDescriptions* pBodyDefs);
void _WriteFrame(FILE* fp, sFrameOfMocapData* data);
void _WriteFooter(FILE* fp);
void NATNET_CALLCONV ServerDiscoveredCallback( const sNatNetDiscoveredServer* pDiscoveredServer, void* pUserContext );
void NATNET_CALLCONV MocapDataHandler(sFrameOfMocapData* data, void* pUserData);    // receives data from the server
void NATNET_CALLCONV MessageHandler(Verbosity msgType, const char* msg);      // receives NatNet error messages
void resetClient();
int ConnectClient();

static const ConnectionType kDefaultConnectionType = ConnectionType_Unicast;

NatNetClient* g_pClient = NULL;
FILE* g_outputFile;

std::vector< sNatNetDiscoveredServer > g_discoveredServers;
sNatNetClientConnectParams g_connectParams;
char g_discoveredMulticastGroupAddr[kNatNetIpv4AddrStrLenMax] = NATNET_DEFAULT_MULTICAST_ADDRESS;
int g_analogSamplesPerMocapFrame = 0;
sServerDescription g_serverDescription;



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
		this_thread::sleep_for(0.004s);
	}
}

void send_arm_to_state(vector<double>& q, double t=0.1){
	cout << "Moving to ";
	for(double d: q) cout << d << ' '; cout << endl;
	
	cout << "Moving to degree ";
	for(double d: q) cout << d / M_PI * 180 << ' '; cout << endl;
	
	rtde_control.moveJ(q, 1.0, 1.0); // vel acc
}

#ifdef JUST_FOLLOW
#define GAP 0.05
#else
#define GAP 0.2
#endif
inline double sm_step(double s, double g){
	// small step to ensure continuity
	if(s < g){
		if(g > s + GAP) return s + GAP;
		else return g;
	}else{
		if(g < s - GAP) return s - GAP;
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

	rtde_control.servoJ(q, 2, 2, 1. / RATE, 0.1, 300); // vel acc lookahead_time gain
}



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

struct Point{
	double x, y, z;
};
// typedef pair<ros::Time, Point> ball_pose;
typedef pair<double, Point> ball_pose;

//TODO: USE IT
map<int,vector<ball_pose>> balls_poses;

vector<ball_pose > ball_poses;

double gripper_state;

#ifdef JUST_FOLLOW
void balls_state_callback(const Point& msg, const double t){
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
	double x = msg.x;
	double y = msg.y;
	double z = msg.z;
	if(z > 1.4) return;
	ball_poses.emplace_back(t, Point());
	// coarsely change into robot frame
	// at spring 
	// ball_poses.back().second.x = -y + .2;
	// ball_poses.back().second.y = x - 5.89; 
	// ball_poses.back().second.z = z - 0.8;
	
	// at hill 122
	ball_poses.back().second.x = -y - 0.33;
	ball_poses.back().second.y = x + 0.035; 
	ball_poses.back().second.z = z;
	
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

		// just let it point towards x neg
		T[0] = -1;
		T[4] = 0;
		T[8] = 0;
		T[3] = -0.65;
		
		T[1] = 0;
		T[5] = 0;
		T[9] = 1;
		// T[7] = p2.y;
		T[7] = clamp(p2.y, -0.4, 0.4);

		T[2] = 0;
		T[6] = 1;
		T[10]= 0;
		T[11]= clamp(p2.z, 0.25, 0.75); 
		// if(p2.z < 0.2) return;
		// for(int i=0;i<12;i++){
		// 	cout << T[i] << ", ";
		// 	if(i%4==3) cout << endl;
		// }

		// cout<<endl;
		double q_sols[48];

		int sol_num = inverse(T, q_sols, 0);
		// cout << "Solution count " << sol_num << endl;
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
void balls_state_callback(const Point& msg, const double t){
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
	
	static bool thrown = false;
	double x = msg.x;
	double y = msg.y;
	double z = msg.z;
	ball_poses.emplace_back(t, Point());
	// coarsely change into robot frame
	// at spring 
	// ball_poses.back().second.x = -y + .2;
	// ball_poses.back().second.y = x - 5.89; 
	// ball_poses.back().second.z = z - 0.8;
	
	// at hill 122
	ball_poses.back().second.x = -y - 0.33;
	ball_poses.back().second.y = x + 0.035; 
	ball_poses.back().second.z = z;
	if(ball_poses.back().second.x < -2.7) {
		ball_poses.pop_back(); return;
	}else if(ball_poses.back().second.x > -0.7 || ball_poses.back().second.z > 1.4) {
		thrown = true;
		ball_poses.pop_back(); // return;
	}else if(thrown){
		ball_poses.pop_back(); // return;
	}
	
	
	int Step_Size = 20;
	const int maxStep_Size = 20;
	const int minStep_Size = 7;
	
	const double g = 9.80;

	if(ball_poses.size() >= minStep_Size){
		// use the past 20 steps
		
		double sum_xt = 0, sum_x = 0;
		double sum_yt = 0, sum_y = 0;
		double sum_zt = 0, sum_z = 0;
		double sum_t = 0, sum_tt = 0;

		int Step_Size = min((int)ball_poses.size(), maxStep_Size);

		for (int st = ball_poses.size() - Step_Size, i=0; i < Step_Size; i++){
			double t = (ball_poses[st + i].first - ball_poses[st].first);
			double x = ball_poses[st + i].second.x;
			double y = ball_poses[st + i].second.y;
			double z = ball_poses[st + i].second.z + 0.5 * g * t * t;
			// cout << "x y z " << x << " " << y << " " << "z " << ball_poses[st + i].second.z << endl;
			sum_t += t; sum_tt += t * t;
			sum_x += x; sum_xt += t * x;
			sum_y += y; sum_yt += t * y;
			sum_z += z; sum_zt += t * z;
		}

		double x_speed = (sum_xt * Step_Size - sum_x * sum_t) / (Step_Size * sum_tt - sum_t * sum_t);
		double x0 = (sum_x - x_speed * sum_t) / Step_Size;
		
		double y_speed = (sum_yt * Step_Size - sum_y * sum_t) / (Step_Size * sum_tt - sum_t * sum_t);
		double y0 = (sum_y - y_speed * sum_t) / Step_Size;
		
		double z_speed = (sum_zt * Step_Size - sum_z * sum_t) / (Step_Size * sum_tt - sum_t * sum_t);
		double z0 = (sum_z - z_speed * sum_t) / Step_Size;

		// double z_speed = (p2.z-p1.z) / dt - dt * g * 0.5;

		// double t = (-0.55 - p2.y)/y_speed;
		double t = (-0.55 - x0) / x_speed;

		static bool activated = false;
		double t0 = (-0.55 - ball_poses.back().second.x) / x_speed;
		if(t0 < 0.21){
			if(! activated) gripper.activate(), activated = true;
			gripper.move(0.3, 1.0, 0.8, ur_rtde::RobotiqGripper::START_MOVE);
		}
		
		// . . . p2.x + y_speed*t
		// . . . p2.x + y_speed*t
		// . . . p2.z + z_speed*t - 0.5*g*t*t
		// . . . 1
		double T[12]={0};
		// cout<< "t = " << t << ", x_speed = " << x_speed << ", y_speed = " << y_speed
		// 	<< ", z_speed = "<< z_speed <<endl;
		double vz, v, vxy;
		
		// vx = x_speed;
		// vy = y_speed;
		vz = z_speed - 9.80 * t;
		v = sqrt(x_speed*x_speed + y_speed*y_speed + vz*vz);

		vxy = sqrt(x_speed*x_speed + y_speed*y_speed);

		T[0]=-x_speed/v;
		T[4]=-y_speed/v;
		T[8]=-vz/v;
		T[3]=x0 + x_speed*t;
#ifdef HOR_GRP
		T[1]=y_speed/vxy;
		T[5]=-x_speed/vxy;
		T[9]=0;
#else		
		T[1]=-x_speed * vz / (v * vxy);
		T[5]=-y_speed * vz / (v * vxy);
		T[9]=vxy / v;
#endif
		T[7]=y0 + y_speed*t;
		T[7] = clamp(T[7], -0.4, 0.4);

#ifdef HOR_GRP		
		T[2]=-x_speed * vz / (v * vxy);
		T[6]=-y_speed * vz / (v * vxy);
		T[10]=vxy / v;
#else		
		T[2] = -y_speed/vxy;
		T[6] = x_speed/vxy;
		T[10] = 0;
#endif		
		T[11]=z0 + z_speed*t - 0.5*9.8*t*t;
		T[11]= clamp(T[11], 0.25, 0.95); 

		// for(int i=0;i<12;i++){
		// 	cout<<T[i]<<", ";
		// 	if(i%4==3)cout<<endl;
		// }

		// cout<<endl;
		double q_sols[48];

		int sol_num = inverse(T, q_sols, 1.5708);
		// cout << "Solution count " << sol_num << endl;
		// if(!thrown) printf(" %lf  %lf \n", x_speed, y_speed);
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
				tmp[i]=q_sols[i + id*6]; //cout << tmp[i] << ' ';
			}//cout << endl;
			// cout << "min diff = " << tmp1[id] << endl;
			// double t_arrival = t - (ros::Time::now() - ball_poses[6].first).toSec();
			// cout<<"t_arrival = "<<t_arrival<<endl;
			// send_arm_to_state(tmp);
			// exit(1);
			
			servo_arm_to_state(tmp);

			// send_arm_to_states(vector<vector<double>>{tmp, tmp, tmp}, vector<double>{t/2, t*3/2, 2*t});
			// double t_remain = ros::Time::now() - ball_poses[6].first;

			// send_gripper_to_states({0.12,0.67,0.67,0.12}, 
			// 	{t_arrival/2,t_arrival,0.5+t_arrival*2,0.5+t_arrival*3});
		}
		
	}

}
#endif
// Establish a NatNet Client connection
int ConnectClient()
{
    // Release previous server
    g_pClient->Disconnect();

    // Init Client and connect to NatNet server
    int retCode = g_pClient->Connect( g_connectParams );
    if (retCode != ErrorCode_OK)
    {
        printf("Unable to connect to server.  Error code: %d. Exiting.\n", retCode);
        return ErrorCode_Internal;
    }
    else
    {
        // connection succeeded

        void* pResult;
        int nBytes = 0;
        ErrorCode ret = ErrorCode_OK;

        // print server info
        memset( &g_serverDescription, 0, sizeof( g_serverDescription ) );
        ret = g_pClient->GetServerDescription( &g_serverDescription );
        if ( ret != ErrorCode_OK || ! g_serverDescription.HostPresent )
        {
            printf("Unable to connect to server. Host not present. Exiting.\n");
            return 1;
        }
        printf("\n[SampleClient] Server application info:\n");
        printf("Application: %s (ver. %d.%d.%d.%d)\n", g_serverDescription.szHostApp, g_serverDescription.HostAppVersion[0],
            g_serverDescription.HostAppVersion[1], g_serverDescription.HostAppVersion[2], g_serverDescription.HostAppVersion[3]);
        printf("NatNet Version: %d.%d.%d.%d\n", g_serverDescription.NatNetVersion[0], g_serverDescription.NatNetVersion[1],
            g_serverDescription.NatNetVersion[2], g_serverDescription.NatNetVersion[3]);
        printf("Client IP:%s\n", g_connectParams.localAddress );
        printf("Server IP:%s\n", g_connectParams.serverAddress );
        printf("Server Name:%s\n", g_serverDescription.szHostComputerName);

        // get mocap frame rate
        ret = g_pClient->SendMessageAndWait("FrameRate", &pResult, &nBytes);
        if (ret == ErrorCode_OK)
        {
            float fRate = *((float*)pResult);
            printf("Mocap Framerate : %3.2f\n", fRate);
        }
        else
            printf("Error getting frame rate.\n");

        // get # of analog samples per mocap frame of data
        ret = g_pClient->SendMessageAndWait("AnalogSamplesPerMocapFrame", &pResult, &nBytes);
        if (ret == ErrorCode_OK)
        {
            g_analogSamplesPerMocapFrame = *((int*)pResult);
            printf("Analog Samples Per Mocap Frame : %d\n", g_analogSamplesPerMocapFrame);
        }
        else
            printf("Error getting Analog frame rate.\n");
    }

    return ErrorCode_OK;
}

// MessageHandler receives NatNet error/debug messages
void NATNET_CALLCONV MessageHandler( Verbosity msgType, const char* msg )
{
    // Optional: Filter out debug messages
    if ( msgType < Verbosity_Info )
    {
        return;
    }

    printf( "\n[NatNetLib]" );

    switch ( msgType )
    {
        case Verbosity_Debug:
            printf( " [DEBUG]" );
            break;
        case Verbosity_Info:
            printf( "  [INFO]" );
            break;
        case Verbosity_Warning:
            printf( "  [WARN]" );
            break;
        case Verbosity_Error:
            printf( " [ERROR]" );
            break;
        default:
            printf( " [?????]" );
            break;
    }

    printf( ": %s\n", msg );
}

void NATNET_CALLCONV MocapDataHandler(sFrameOfMocapData* data, void* pUserData)
{
    NatNetClient* pClient = (NatNetClient*) pUserData;

    const uint64_t softwareLatencyHostTicks = data->TransmitTimestamp - data->CameraDataReceivedTimestamp;
    const double softwareLatencyMillisec = (softwareLatencyHostTicks * 1000) / static_cast<double>(g_serverDescription.HighResClockFrequency);

    const double transitLatencyMillisec = pClient->SecondsSinceHostTimestamp( data->TransmitTimestamp ) * 1000.0;

    int i=0;

    // printf("FrameID : %d\n", data->iFrame);
    // printf("Timestamp : %3.2lf\n", data->fTimestamp);
    // printf("Software latency : %.2lf milliseconds\n", softwareLatencyMillisec);

    const bool bSystemLatencyAvailable = data->CameraMidExposureTimestamp != 0;
/*
    if ( bSystemLatencyAvailable )
    {
        // System latency here is defined as the span of time between:
        //   a) The midpoint of the camera exposure window, and therefore the average age of the photons (CameraMidExposureTimestamp)
        // and
        //   b) The time immediately prior to the NatNet frame being transmitted over the network (TransmitTimestamp)
        const uint64_t systemLatencyHostTicks = data->TransmitTimestamp - data->CameraMidExposureTimestamp;
        const double systemLatencyMillisec = (systemLatencyHostTicks * 1000) / static_cast<double>(g_serverDescription.HighResClockFrequency);
        const double clientLatencyMillisec = pClient->SecondsSinceHostTimestamp( data->CameraMidExposureTimestamp ) * 1000.0;

        printf( "System latency : %.2lf milliseconds\n", systemLatencyMillisec );
        printf( "Total client latency : %.2lf milliseconds (transit time +%.2lf ms)\n", clientLatencyMillisec, transitLatencyMillisec );
    }
    else
    {
        printf( "Transit latency : %.2lf milliseconds\n", transitLatencyMillisec );
    }
*/
    

	// Rigid Bodies
	// printf("Rigid Bodies [Count=%d]\n", data->nRigidBodies);
	// for(i=0; i < data->nRigidBodies; i++)
	// {
    //     // params
    //     // 0x01 : bool, rigid body was successfully tracked in this frame
    //     bool bTrackingValid = data->RigidBodies[i].params & 0x01;

	// 	printf("Rigid Body [ID=%d  Error=%3.2f  Valid=%d]\n", data->RigidBodies[i].ID, data->RigidBodies[i].MeanError, bTrackingValid);
	// 	printf("\tx\ty\tz\tqx\tqy\tqz\tqw\n");
	// 	printf("\t%3.2f\t%3.2f\t%3.2f\t%3.2f\t%3.2f\t%3.2f\t%3.2f\n",
	// 		data->RigidBodies[i].x,
	// 		data->RigidBodies[i].y,
	// 		data->RigidBodies[i].z,
	// 		data->RigidBodies[i].qx,
	// 		data->RigidBodies[i].qy,
	// 		data->RigidBodies[i].qz,
	// 		data->RigidBodies[i].qw);
	// }

	

	// labeled markers - this includes all markers (Active, Passive, and 'unlabeled' (markers with no asset but a PointCloud ID)
    bool bOccluded;     // marker was not visible (occluded) in this frame
    bool bPCSolved;     // reported position provided by point cloud solve
    bool bModelSolved;  // reported position provided by model solve
    bool bHasModel;     // marker has an associated asset in the data stream
    bool bUnlabeled;    // marker is 'unlabeled', but has a point cloud ID that matches Motive PointCloud ID (In Motive 3D View)
	bool bActiveMarker; // marker is an actively labeled LED marker

	// printf("Markers [Count=%d]\n", data->nLabeledMarkers);
	for(i=0; i < data->nLabeledMarkers; i++)
	{
        bOccluded = ((data->LabeledMarkers[i].params & 0x01)!=0);
        bPCSolved = ((data->LabeledMarkers[i].params & 0x02)!=0);
        bModelSolved = ((data->LabeledMarkers[i].params & 0x04) != 0);
        bHasModel = ((data->LabeledMarkers[i].params & 0x08) != 0);
        bUnlabeled = ((data->LabeledMarkers[i].params & 0x10) != 0);
		bActiveMarker = ((data->LabeledMarkers[i].params & 0x20) != 0);

        sMarker marker = data->LabeledMarkers[i];

        // Marker ID Scheme:
        // Active Markers:
        //   ID = ActiveID, correlates to RB ActiveLabels list
        // Passive Markers: 
        //   If Asset with Legacy Labels
        //      AssetID 	(Hi Word)
        //      MemberID	(Lo Word)
        //   Else
        //      PointCloud ID
        int modelID, markerID;
        NatNet_DecodeID( marker.ID, &modelID, &markerID );
		
        // char szMarkerType[512];
        // if (bActiveMarker)
        //     strcpy(szMarkerType, "Active");
        // else if(bUnlabeled)
        //     strcpy(szMarkerType, "Unlabeled");
        // else
        //     strcpy(szMarkerType, "Labeled");
        // printf("%s Marker [ModelID=%d, MarkerID=%d] [size=%3.2f] [pos=%3.2f,%3.2f,%3.2f]\n",
        //     szMarkerType, modelID, markerID, marker.size, marker.x, marker.y, marker.z);
		balls_state_callback(Point{.x = marker.x, .y = marker.y, .z = marker.z}, data->fTimestamp);
	}


}

void setup_mocap(){
	unsigned char ver[4];
    NatNet_GetVersion( ver );
    printf( "NatNet Sample Client (NatNet ver. %d.%d.%d.%d)\n", ver[0], ver[1], ver[2], ver[3] );

    // Install logging callback
    NatNet_SetLogCallback( MessageHandler );

    // create NatNet client
    g_pClient = new NatNetClient();

    // set the frame callback handler
    g_pClient->SetFrameReceivedCallback( MocapDataHandler, g_pClient );	// this function will receive data from the server

    // If no arguments were specified on the command line,
    // attempt to discover servers on the local network.
    
    {
        g_connectParams.connectionType = kDefaultConnectionType;
		g_connectParams.serverAddress = MOCAP_IP_ADDR;
    }

    int iResult;

    // Connect to Motive
    iResult = ConnectClient();
    if (iResult != ErrorCode_OK)
    {
        printf("Error initializing client. See log for details. Exiting.\n");
        return;
    }
    else
    {
        printf("Client initialized and ready.\n");
    }


	// Send/receive test request
    void* response;
    int nBytes;
	printf("[SampleClient] Sending Test Request\n");
	iResult = g_pClient->SendMessageAndWait("TestRequest", &response, &nBytes);
	if (iResult == ErrorCode_OK)
	{
		printf("[SampleClient] Received: %s\n", (char*)response);
	}

	// Retrieve Data Descriptions from Motive
	printf("\n\n[SampleClient] Requesting Data Descriptions...\n");
	sDataDescriptions* pDataDefs = NULL;
	iResult = g_pClient->GetDataDescriptionList(&pDataDefs);
	if (iResult != ErrorCode_OK || pDataDefs == NULL){
		printf("[SampleClient] Unable to retrieve Data Descriptions.\n");
	}else{
        printf("[SampleClient] Received %d Data Descriptions:\n", pDataDefs->nDataDescriptions );
	}

	
	// Create data file for writing received stream into
	
	// // Ready to receive marker stream!
	// printf("\nClient is connected to server and listening for data...\n");
	// bool bExit = false;
	// g_connectParams.connectionType = ConnectionType_Multicast;
    // iResult = ConnectClient();
    

	// Done - clean up.
	// while(1);
	// if (g_pClient)
	// {
	// 	g_pClient->Disconnect();
	// 	delete g_pClient;
	// 	g_pClient = NULL;
	// }

	return ;
}

int main(int argc, char** argv){
	
	// set up gripper
	gripper.connect();
	if (!gripper.isActive())
	{
		gripper.emergencyRelease(RobotiqGripper::OPEN);
	}
	std::cout << "Fault status: 0x" << std::hex << gripper.faultStatus() << std::dec << std::endl;
	std::cout << "activating gripper" << std::endl;
	gripper.activate();
	gripper.setUnit(RobotiqGripper::POSITION, RobotiqGripper::UNIT_NORMALIZED);
	std::cout << "OpenPosition: " << gripper.getOpenPosition() << "  ClosedPosition: " << gripper.getClosedPosition()
				<< std::endl;

	gripper.setForce(0.0);
  	gripper.setSpeed(1.0);
	int status = gripper.move(0.1, 1, 0.2, RobotiqGripper::WAIT_FINISHED);
	status = gripper.move(0.9, 1, 0.2, RobotiqGripper::WAIT_FINISHED);
	
	cout << "Catch bot started" << endl;

	// set up arm
	arm_joints_state.resize(6);
	thread t1(arm_state_callback);

	send_arm_to_state(q_default);
	send_gripper_to_state(0.56); // 0 ~ 0.8

	// set up mocap
	setup_mocap();
#ifdef JUST_FOLLOW
	// ros::Subscriber sub_cam = node.subscribe("/vrpn_client_node/RigidBody/pose", 10, balls_state_callback);
#else
	
#endif
	// ros::Subscriber sub_arm = node.subscribe("/joint_states", 10, arm_state_callback);
	
	t1.join();

	return 0;
}
