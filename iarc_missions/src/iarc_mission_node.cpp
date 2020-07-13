#include <ros/ros.h>
#include "IARCMission.h"
#include <geometry_msgs/Twist.h>
using namespace std;
using namespace DJI::onboardSDK;

ros::NodeHandle nh;
mission::IARCMission CIARCMission(nh);

void DroneVelCallback(geometry_msgs::Twist msg)
{
	double sample_time=0.02;
	CIARCMission.CDJIDrone->attitude_control(Flight::HorizontalLogic::HORIZONTAL_POSITION |
                Flight::VerticalLogic::VERTICAL_VELOCITY |
                Flight::YawLogic::YAW_ANGLE |
                Flight::HorizontalCoordinate::HORIZONTAL_BODY |
                Flight::SmoothMode::SMOOTH_ENABLE,
				msg.linear.x*sample_time,msg.linear.y*sample_time,0,
				CIARCMission.yaw_origin+msg.angular.z*sample_time);
				CIARCMission.yaw_origin+=msg.angular.z*sample_time;
	            // CDJIDrone->attitude_control( Flight::HorizontalLogic::HORIZONTAL_POSITION |
                // Flight::VerticalLogic::VERTICAL_VELOCITY |
                // Flight::YawLogic::YAW_ANGLE |
                // Flight::HorizontalCoordinate::HORIZONTAL_BODY |
                // Flight::SmoothMode::SMOOTH_ENABLE,
                // horizon_step_ahead, 0, 0, yaw_origin );
	
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "srtp_mission_node");
	ros::NodeHandle nh;
	CIARCMission=mission::IARCMission(nh);

	ros::Subscriber drone_vel_sub = nh.subscribe("/tracking_node/drone_vel", 10, &DroneVelCallback);

	ros::spin();
	return 0;
}