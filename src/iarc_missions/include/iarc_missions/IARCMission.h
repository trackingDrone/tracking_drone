#ifndef IARCMISSION_H
#define IARCMISSION_H
#include <ros/ros.h>
#include <std_msgs/Int8.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float32MultiArray.h>
#include <iostream>
#include <dji_sdk/LocalPosition.h>

#include "iarc_msgs/QuadState.h"
#include "iarc_msgs/QuadVelocity.h"
#include "iarc_msgs/SendCommand.h"
#include "iarc_msgs/SendPose.h"
#include "iarc_msgs/SendQRCode.h"
#include "iarc_msgs/SendQRMatrix.h"
#include "iarc_msgs/Start_Cure.h"
#include "iarc_msgs/Start_QRScan.h"
#include "iarc_msgs/Pose.h"
#include "iarc_msgs/QRCode.h"
#include "iarc_msgs/QRMatrix.h"
#include "iarc_msgs/Command.h"

//#include <goal_detected/Pose3D.h>
#include <geometry_msgs/Point32.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PointStamped.h>
#include <dji_sdk/dji_drone.h>
#include <dji_sdk/AttitudeQuaternion.h>
#include <dji_sdk_lib/DJI_Flight.h>

#include <vector>
#include <geometry_msgs/PointStamped.h>
#include <opencv2/opencv.hpp>
#include <stdio.h>                 
#include <unistd.h>               //C 和 C++ 程序设计语言中提供对 POSIX 操作系统 API 的访问功能的头文件的名称 
#include <fcntl.h>                //fcntl.h定义了很多宏和open,fcntl函数原型；unistd.h定义了更多的函数原型 
#include <termios.h>              //这是Linux 下串口驱动头文件;一般只能在Linux下。
#include <string>
#include <stdlib.h> 


enum MissionState_{REQUEST_CONTROL,RELEASE_CONTROL,TAKE_OFF,STAY_STILL,LAND,GO_HOME,DRONE_ARM,DRONE_DISARM,GO_AHEAD,GO_LITTLE_AHEAD,GO_BACK,GO_LITTLE_BACK,GO_LEFT,GO_LITTLE_LEFT,GO_RIGHT,GO_LITTLE_RIGHT,GO_UP,GO_DOWN,GO_LITTLE_DOWN,GO_CLOCKWISE,GO_ANTICLOCKWISE,START_CURE,START_VIDEO};//

#define PI 3.1415926
using namespace std;
using namespace cv;
namespace mission{
class IARCMission
{
public:
	ros::NodeHandle nh_;
	ros::NodeHandle nh_param;
	ros::Publisher missionState_pub; 
	ros::Publisher startcure_pub;
	ros::Publisher start_qrscan_pub;
	ros::Subscriber quad_state_sub;	//target position from computer vision (package = goal_detected)
	ros::Subscriber quater_sub;
	ros::Subscriber tar_velocity_sub;//target velocity when encoutering obstacles
	ros::Subscriber pose_sub;	//target position from computer vision (package = goal_detected)
	ros::Subscriber command_sub;
	ros::Subscriber qrmatrix_sub;
	ros::Subscriber qrcode_sub;
	ros::Subscriber qr_track_sub;

	ros::ServiceClient send_command_client;
	ros::ServiceClient send_pose_client;
	ros::ServiceClient send_qrmatrix_client;
	ros::ServiceClient send_qrcode_client;
	iarc_msgs::SendCommand command_srv;
 	iarc_msgs::SendPose pose_srv;
	iarc_msgs::SendQRCode qrcode_srv;
 	iarc_msgs::SendQRMatrix qrmatrix_srv;
 	iarc_msgs::Start_Cure if_cure;
 	iarc_msgs::Start_QRScan if_qrscan;

	geometry_msgs::Point32 flight_ctrl_dst;
	geometry_msgs::Point quadrotorGroundPos;
	std_msgs::Int8 mission_state_msg;
	ros::Time free_time;
	ros::Time free_time_prev;
	ros::Duration freeTimer;
	
	DJIDrone *CDJIDrone;
	
	iarc_msgs::QRCode qrcode_rev;
	iarc_msgs::QRMatrix matrix_rev;
	int command_rev;
	int control_state;
	double yaw_origin,yaw_origin_rad,sta_x,sta_y,xMax,yMax;
	int quadState;
	struct irobotPose_
	{
		float x;
		float y;
		float angle;
	};	
	irobotPose_ quad_pose;
	IARCMission(ros::NodeHandle nh);
	~IARCMission();
	void initialize();
	void Display_Main_Menu();
	void quad_state_callback(const iarc_msgs::QuadState &msg);
	void dji_quaternion_callback(const dji_sdk::AttitudeQuaternion::ConstPtr &msg);
	void tar_velocity_callback(const std_msgs::Float32MultiArrayConstPtr &msg);

	void command_callback(const iarc_msgs::Command &msg);
	void pose_callback(const iarc_msgs::Pose &msg);
	void qrcode_callback(const iarc_msgs::QRCode &msg);
	void qrmatrix_callback(const iarc_msgs::QRMatrix &msg);
	void qrtrack_callback(const std_msgs::Float32MultiArray &msg);
	
	//MISSION PARTS
	bool mission_takeoff();
	bool mission_land();
	void missionStayStill();
	void missionGoAhead();
	void missionGoBack();
	void missionGoLeft();
	void missionGoRight();
	void missionGoUp();
	void missionGoDown();
	void missionGoClockwise();
	void missionGoAntiClockwise();
	void missionStartCure();
	void missionStartQRScan();
	void missionStartVideo();

        void missionGoLittleAhead();
	void missionGoLittleBack();
	void missionGoLittleLeft();
	void missionGoLittleRight();
	void missionGoLittleDown();
        void missionTurnAround();

	void mission_drone_1();
	void mission_drone_2();
	void mission_drone_1_2();

	int timeToSearchRedPart;
	int maxNumSearchGreenPart;
	int numSearchGreenPart;
	
	ros::Time boundaryUpdateTime_X;
	ros::Time boundaryUpdateTime_Y;
	ros::Duration durationNotUpdate_X;
	ros::Duration durationNotUpdate_Y;
	
	double maxWaitTime;
	double MaxCruiseTime;
	double boundarySafetySpace;
	ros::Time time_now;
	ros::Time time_last;
	ros::Time time_last_right_direction;
	ros::Time time_topCount;
	ros::Time time_missionStart;
	ros::Duration duration_wrong_direction;
	ros::Duration duration;
	ros::Duration duration_mission;
	ros::Duration duration_topCount;
	float time_in_20s;
	float time_reactAftHead;
	float time_reactAftTop;
	float quad_velocity[2];
	float Quater[4];
	float yaw;
	double time_step_1;
	double time_step_2;
	double horizon_step;
	double vertical_step;
	double yaw_step;
    int mission_flag=0;
	double time_step_up,time_step_clockwise,time_step_ahead,time_step_left;
	double horizon_step_ahead,horizon_step_left,little_part_left,little_part_ahead;

	Mat img;
	VideoWriter vw;
	friend class Flight;
	float delta_vx, delta_vy;
        float obstacle_velocity[2];
        bool obstacle_emergency =false;
        float obstacle_dist;
 
        float QRtrack_velocity[3];
        float qrtrack_end;//-1:tacking   1:end
};

};
#endif

