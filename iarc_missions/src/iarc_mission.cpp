#include <IARCMission.h>
#include <math.h>
#include <ros/init.h>
using namespace std;
using namespace DJI::onboardSDK;
namespace mission{
/*
VERTICAL_VELOCITY = 0x00,
VERTICAL_POSITION = 0x10,
VERTICAL_THRUST = 0x20,
HORIZONTAL_ANGLE = 0x00,
HORIZONTAL_VELOCITY = 0x40,
HORIZONTAL_POSITION = 0X80 
*/
void IARCMission::Display_Main_Menu()
{
    printf("\r\n");
    printf("+-------------------------- < Main menu > ------------------------+\n");
	printf("| [0] Request Control           |\n");	
	printf("| [1] Release Control           |\n");
	printf("| [2] Takeoff                   |\n");	
	printf("| [3] Stay still                |\n");	
	printf("| [4] Landing                   |\n");	
	printf("| [5] Go Home                   |\n");	
	printf("| [6] Arm the Drone             |\n");	
	printf("| [7] Disarm the Drone          |\n");
	printf("| [8] Go ahead                  |\n");
        printf("| [9] Go little ahead           |\n");
	printf("| [10] Go back                  |\n");
        printf("| [11] Go little back           |\n");
	printf("| [12] Go left                  |\n");
        printf("| [13] Go little left           |\n");
	printf("| [14] Go right                 |\n");
        printf("| [15] Go little right          |\n");
	printf("| [16] Go up                    |\n");
	printf("| [17] Go down                  |\n");
	printf("| [18] Go little down           |\n");
        printf("| [19] Go clockwise             |\n");
	printf("| [20] Go anticlockwise         |\n");
        printf("| [21] Start cure               |\n");
        printf("| [22] Start video              |\n");
        printf("+-----------------------------------------------------------------+\n");
}
IARCMission::IARCMission(ros::NodeHandle nh):nh_(nh),nh_param("~")
{
	
    initialize();
    // printf("This is automatic mode\r\n");
    // printf("Don't input anything\r\n");
    // printf("----------------------------------------\r\n");
	CDJIDrone->request_sdk_permission_control();
	sleep(2);
	std_msgs::Int8 stateMsg;
	ros::Rate loop_rate(50);
	while(ros::ok())
	{
		
		ros::spinOnce();
		if(control_state){ 
            if(quadState!=21){
                if_cure.start_cure = 0;
                startcure_pub.publish(if_cure);
            }
            if(quadState!=22){
                if_qrscan.start_qrscan = 0;
               start_qrscan_pub.publish(if_qrscan);
            }
        }
        else{
            Display_Main_Menu();
            printf("input 0/1/2 etc..then press enter key\r\n");
            printf("use `rostopic echo` to query drone status\r\n");
            printf("----------------------------------------\r\n");
            printf("input: ");
            cin>>quadState;
            cout<<quadState<<endl;
        }
		switch(quadState)
		{
		
			case REQUEST_CONTROL:
			{
				CDJIDrone->request_sdk_permission_control();
				break;
			}
			case RELEASE_CONTROL:
			{
				CDJIDrone->release_sdk_permission_control();
				break;
			}
			case TAKE_OFF:
			{
				mission_takeoff();
				break;
			}
			case STAY_STILL:
			{
                missionStayStill();
                break;
            }
			case LAND:
			{
				mission_land();
				break;
			}
			case GO_HOME:
            { 
            	CDJIDrone->gohome();
                break;
			}
			case DRONE_ARM:
			{
				CDJIDrone->drone_arm();
			    break;
			}
			case DRONE_DISARM:
			{	
				CDJIDrone->drone_disarm();
			    break;
			}
			case GO_AHEAD:
			{   missionGoAhead();
				break;
			}
            case GO_LITTLE_AHEAD:
            {   missionGoLittleAhead();
                break;
            }
			case GO_BACK:
			{
                missionGoBack();
                break;
            }
            case GO_LITTLE_BACK:
            {
                missionGoLittleBack();
                break;
            }
			case GO_LEFT:
			{
                missionGoLeft();
                break;
            }
            case GO_LITTLE_LEFT:
            {
                missionGoLittleLeft();
                break;
            }
			case GO_RIGHT:
			{
                missionGoRight();
                break;
            }
            case GO_LITTLE_RIGHT:
            {
                missionGoLittleRight();
                break;
            }
			case GO_UP:
			{
				missionGoUp();
    			break;
			}
			case GO_DOWN:
			{
				 missionGoDown();
				 break;
			}
            case GO_LITTLE_DOWN:
            {
                 missionGoLittleDown();
                 break;
            }
			case GO_ANTICLOCKWISE:
			{
				missionGoAntiClockwise();
				 break;
			}
			case GO_CLOCKWISE:
			{
				missionGoClockwise();
				 break;
			}
			case START_CURE:
            {
                missionStartCure();
                break;
            }

            case START_VIDEO:
            {
               missionStartVideo();
                break;
            }
		}
		stateMsg.data = quadState;
		missionState_pub.publish(stateMsg);
		loop_rate.sleep();
	}
}



IARCMission::~IARCMission()
{
	ROS_INFO("Destroying IARCMission......");
}


void IARCMission::initialize()
{
    if(!nh_param.getParam("horizon_step_ahead",horizon_step_ahead))horizon_step_ahead = 0.5;
    if(!nh_param.getParam("horizon_step_left",horizon_step_left))horizon_step_left = 1;
    if(!nh_param.getParam("little_part_left",little_part_left))little_part_left = 0.6;
    if(!nh_param.getParam("little_part_ahead",little_part_ahead))little_part_ahead = 0.35;
    if(!nh_param.getParam("vertical_step",vertical_step))vertical_step = 0.5;
    if(!nh_param.getParam("time_step_up",time_step_up))time_step_up = 40;
    if(!nh_param.getParam("time_step_clockwise",time_step_clockwise))time_step_clockwise = 30;
    if(!nh_param.getParam("time_step_ahead",time_step_ahead))time_step_ahead = 120;
    if(!nh_param.getParam("time_step_left",time_step_left))time_step_left = 20;
    if(!nh_param.getParam("yaw_step",yaw_step))yaw_step = 0.8;
    if(!nh_param.getParam("control_state",control_state))control_state = 1;

   obstacle_emergency = 0;
    missionState_pub = nh_.advertise<std_msgs::Int8>("/iarc_missions/missionState", 10);
    startcure_pub = nh_.advertise<iarc_msgs::Start_Cure>("/iarc_missions/if_start_track", 10);
    start_qrscan_pub = nh_.advertise<iarc_msgs::Start_QRScan>("/iarc_missions/if_scan_qrcode", 10);
    
    
    quad_state_sub = nh_.subscribe("/iarc_missions/quad_state", 10, &IARCMission::quad_state_callback, this);
    quater_sub = nh_.subscribe("/dji_sdk/attitude_quaternion",10, &IARCMission::dji_quaternion_callback,this);
    tar_velocity_sub = nh_.subscribe("/obstacleAvoidance/target_velocity", 1, &IARCMission::tar_velocity_callback, this);
    pose_sub = nh_.subscribe("/commu_node/pose",10,&IARCMission::pose_callback,this);
    command_sub = nh_.subscribe("/commu_node/command",10, &IARCMission::command_callback,this);
    
    CDJIDrone = new DJIDrone(nh_);
	// ROS_ERROR_STREAM_ONCE("Start global timer, please WAIT for irobot turn and then press any key to continue...");
	// getchar();
	time_missionStart = ros::Time::now();
	time_last = ros::Time::now();
	time_last_right_direction = ros::Time::now();
	time_now = ros::Time::now();
	duration = time_now - time_last;
	duration_wrong_direction = duration;
	duration_mission = time_now - time_missionStart;

    ROS_INFO("Mission Initialized");

}

//===========================callbacks==================================================

void IARCMission::pose_callback(const iarc_msgs::Pose &msg){
    quad_pose.x = msg.x;
    quad_pose.y = msg.y;
    quad_pose.angle = msg.angle;
       
}

void IARCMission::command_callback(const  iarc_msgs::Command &msg){
    command_rev = msg.command_num;
    quadState=command_rev;
    mission_flag = 0;
}

    
void IARCMission::quad_state_callback(const iarc_msgs::QuadState &msg){
	quadState = msg.quad_state;
}

void IARCMission::dji_quaternion_callback(const dji_sdk::AttitudeQuaternion::ConstPtr &msg)
{
    Quater[0] = msg->q0;
    Quater[1] = msg->q1;
    Quater[2] = msg->q2;
    Quater[3] = msg->q3;
    yaw = atan2(2.0 * (Quater[3] * Quater[0] + Quater[1] * Quater[2]) , - 1.0 + 2.0 * (Quater[0] * Quater[0] + Quater[1] * Quater[1]));
    yaw_origin=yaw/PI*180;
    //ROS_INFO("yaw = %f",yaw);
}

void IARCMission::tar_velocity_callback(const std_msgs::Float32MultiArrayConstPtr &msg)
{
  obstacle_velocity[0] = msg->data[0];
  obstacle_velocity[1] = msg->data[1];
  obstacle_emergency = (msg->data[2] > 0.5);
  obstacle_dist = msg->data[3];
  //ROS_INFO("obstacle distance = %3.1f",obstacle_dist);
}




//===========================Missions==================================================
void IARCMission::missionStayStill()
{
	ros::spinOnce();
	CDJIDrone->request_sdk_permission_control();
	ROS_WARN("in Stay!");
    float delta_vx, delta_vy;
	//CDJIDrone->request_sdk_permission_control();
	if(quadState==STAY_STILL){
     for(int i = 0;i < 100;i++){
        ros::spinOnce();
        if(obstacle_emergency){
            delta_vx = obstacle_velocity[0];
            delta_vy = obstacle_velocity[1];
            CDJIDrone->attitude_control(0x40, delta_vx, delta_vy, 0, yaw_origin);
            usleep(20000);
        }else{
            CDJIDrone->attitude_control(0x40, 0, 0, 0, yaw_origin);
            usleep(20000);
        }
    }
    }
    mission_flag = 1;
    while((ros::ok())&&quadState==STAY_STILL&&control_state==1&&mission_flag==1){
        ros::spinOnce();
        if(obstacle_emergency){
            delta_vx = obstacle_velocity[0];
            delta_vy = obstacle_velocity[1];
            CDJIDrone->attitude_control(0x40, delta_vx, delta_vy, 0, yaw_origin);
            usleep(20000);
        }else{
            CDJIDrone->attitude_control(0x40, 0, 0, 0, yaw_origin);
            usleep(10000);
        }
    }

}


void IARCMission::missionGoAhead()
{
    ros::spinOnce();
    ROS_WARN("in GoAhead!");
    float delta_vx, delta_vy;
    //CDJIDrone->request_sdk_permission_control();
    for(int i = 0;i < time_step_ahead;i++){
        ros::spinOnce();
        if(obstacle_emergency){
            delta_vx = obstacle_velocity[0];
            delta_vy = obstacle_velocity[1];
            CDJIDrone->attitude_control(0x40, delta_vx, delta_vy, 0, yaw_origin);
            usleep(20000);
        }else{
            CDJIDrone->attitude_control( Flight::HorizontalLogic::HORIZONTAL_POSITION |
                Flight::VerticalLogic::VERTICAL_VELOCITY |
                Flight::YawLogic::YAW_ANGLE |
                Flight::HorizontalCoordinate::HORIZONTAL_BODY |
                Flight::SmoothMode::SMOOTH_ENABLE,
                horizon_step_ahead, 0, 0, yaw_origin );
            usleep(20000);
        }
    }
    mission_flag = 1;
    while((ros::ok())&&quadState==GO_AHEAD&&control_state==1&&mission_flag==1){
        ros::spinOnce();
        if(obstacle_emergency){
            delta_vx = obstacle_velocity[0];
            delta_vy = obstacle_velocity[1];
            CDJIDrone->attitude_control(0x40, delta_vx, delta_vy, 0, yaw_origin);
            usleep(20000);
        }else{
            CDJIDrone->attitude_control(0x40, 0, 0, 0, yaw_origin);
            usleep(10000);
        }
    }
}

void IARCMission::missionGoLittleAhead()
{
    ros::spinOnce();
    ROS_WARN("in Go Little Ahead!");
    float delta_vx, delta_vy;
    //CDJIDrone->request_sdk_permission_control();
    for(int i = 0;i < time_step_ahead*little_part_ahead;i++){
        ros::spinOnce();
        if(obstacle_emergency){
            delta_vx = obstacle_velocity[0];
            delta_vy = obstacle_velocity[1];
            CDJIDrone->attitude_control(0x40, delta_vx, delta_vy, 0, yaw_origin);
            usleep(20000);
        }else{
            CDJIDrone->attitude_control( Flight::HorizontalLogic::HORIZONTAL_POSITION |
                Flight::VerticalLogic::VERTICAL_VELOCITY |
                Flight::YawLogic::YAW_ANGLE |
                Flight::HorizontalCoordinate::HORIZONTAL_BODY |
                Flight::SmoothMode::SMOOTH_ENABLE,
                horizon_step_ahead, 0, 0, yaw_origin );
            usleep(20000);
        }
    }
    mission_flag = 1;
    while((ros::ok())&&quadState==GO_LITTLE_AHEAD&&control_state==1&&mission_flag==1){
        ros::spinOnce();
        if(obstacle_emergency){
            delta_vx = obstacle_velocity[0];
            delta_vy = obstacle_velocity[1];
            CDJIDrone->attitude_control(0x40, delta_vx, delta_vy, 0, yaw_origin);
            usleep(20000);
        }else{
            CDJIDrone->attitude_control(0x40, 0, 0, 0, yaw_origin);
            usleep(10000);
        }
    }
}

void IARCMission::missionGoBack(){
    ros::spinOnce();
    ROS_WARN("in Goback!");
    float delta_vx, delta_vy;
    //CDJIDrone->request_sdk_permission_control();
    for(int i = 0;i < time_step_ahead;i++){
        if(obstacle_emergency){
            delta_vx = obstacle_velocity[0];
            delta_vy = obstacle_velocity[1];
            CDJIDrone->attitude_control(0x40, delta_vx, delta_vy, 0, yaw_origin);
            usleep(20000);
        }else{
            CDJIDrone->attitude_control( Flight::HorizontalLogic::HORIZONTAL_POSITION |
                Flight::VerticalLogic::VERTICAL_VELOCITY |
                Flight::YawLogic::YAW_ANGLE |
                Flight::HorizontalCoordinate::HORIZONTAL_BODY |
                Flight::SmoothMode::SMOOTH_ENABLE,
                -horizon_step_ahead, 0, 0, yaw_origin );
            usleep(20000);
        }
    }
    mission_flag = 1;
    while((ros::ok())&&quadState==GO_BACK&&control_state==1&&mission_flag==1){
       ros::spinOnce();
        if(obstacle_emergency){
            delta_vx = obstacle_velocity[0];
            delta_vy = obstacle_velocity[1];
            CDJIDrone->attitude_control(0x40, delta_vx, delta_vy, 0, yaw_origin);
            usleep(20000);
        }else{
            CDJIDrone->attitude_control(0x40, 0, 0, 0, yaw_origin);
            usleep(10000);
        }
    }
}

void IARCMission::missionGoLittleBack(){
    ros::spinOnce();
    ROS_WARN("in Go Little back!");
    float delta_vx, delta_vy;
    //CDJIDrone->request_sdk_permission_control();
    for(int i = 0;i < time_step_ahead*little_part_ahead;i++){
        if(obstacle_emergency){
            delta_vx = obstacle_velocity[0];
            delta_vy = obstacle_velocity[1];
            CDJIDrone->attitude_control(0x40, delta_vx, delta_vy, 0, yaw_origin);
            usleep(20000);
        }else{
            CDJIDrone->attitude_control( Flight::HorizontalLogic::HORIZONTAL_POSITION |
                Flight::VerticalLogic::VERTICAL_VELOCITY |
                Flight::YawLogic::YAW_ANGLE |
                Flight::HorizontalCoordinate::HORIZONTAL_BODY |
                Flight::SmoothMode::SMOOTH_ENABLE,
                -horizon_step_ahead, 0, 0, yaw_origin );
            usleep(20000);
        }
    }
    mission_flag = 1;
    while((ros::ok())&&quadState==GO_LITTLE_BACK&&control_state==1&&mission_flag==1){
       ros::spinOnce();
        if(obstacle_emergency){
            delta_vx = obstacle_velocity[0];
            delta_vy = obstacle_velocity[1];
            CDJIDrone->attitude_control(0x40, delta_vx, delta_vy, 0, yaw_origin);
            usleep(20000);
        }else{
            CDJIDrone->attitude_control(0x40, 0, 0, 0, yaw_origin);
            usleep(10000);
        }
    }
}

void IARCMission::missionGoLeft(){
    ros::spinOnce();
    ROS_WARN("in Goleft!");
    float delta_vx, delta_vy;
    //CDJIDrone->request_sdk_permission_control();
    for(int i = 0;i < time_step_left;i++){
        ros::spinOnce();
        if(obstacle_emergency){
            delta_vx = obstacle_velocity[0];
            delta_vy = obstacle_velocity[1];
            CDJIDrone->attitude_control(0x40, delta_vx, delta_vy, 0, yaw_origin);
            usleep(20000);
        }else{
            CDJIDrone->attitude_control( Flight::HorizontalLogic::HORIZONTAL_POSITION |
                Flight::VerticalLogic::VERTICAL_VELOCITY |
                Flight::YawLogic::YAW_ANGLE |
                Flight::HorizontalCoordinate::HORIZONTAL_BODY |
                Flight::SmoothMode::SMOOTH_ENABLE,
                0, -horizon_step_left, 0, yaw_origin);
            usleep(20000);
        }
    }
    mission_flag = 1;
    while((ros::ok())&&quadState==GO_LEFT&&control_state==1&&mission_flag==1){
        ros::spinOnce();
        if(obstacle_emergency){
            delta_vx = obstacle_velocity[0];
            delta_vy = obstacle_velocity[1];
            CDJIDrone->attitude_control(0x40, delta_vx, delta_vy, 0, yaw_origin);
            usleep(20000);
        }else{
            CDJIDrone->attitude_control(0x40, 0, 0, 0, yaw_origin);
            usleep(10000);
        }
    }
}

void IARCMission::missionGoLittleLeft(){
    ros::spinOnce();
    ROS_WARN("in Go little left!");
    float delta_vx, delta_vy;
    //CDJIDrone->request_sdk_permission_control();
    for(int i = 0;i < time_step_left*little_part_left;i++){
        ros::spinOnce();
        if(obstacle_emergency){
            delta_vx = obstacle_velocity[0];
            delta_vy = obstacle_velocity[1];
            CDJIDrone->attitude_control(0x40, delta_vx, delta_vy, 0, yaw_origin);
            usleep(20000);
        }else{
            CDJIDrone->attitude_control( Flight::HorizontalLogic::HORIZONTAL_POSITION |
                Flight::VerticalLogic::VERTICAL_VELOCITY |
                Flight::YawLogic::YAW_ANGLE |
                Flight::HorizontalCoordinate::HORIZONTAL_BODY |
                Flight::SmoothMode::SMOOTH_ENABLE,
                0, -horizon_step_left, 0, yaw_origin);
            usleep(20000);
        }
    }
    mission_flag = 1;
    while((ros::ok())&&quadState==GO_LITTLE_LEFT&&control_state==1&&mission_flag==1){
        ros::spinOnce();
        if(obstacle_emergency){
            delta_vx = obstacle_velocity[0];
            delta_vy = obstacle_velocity[1];
            CDJIDrone->attitude_control(0x40, delta_vx, delta_vy, 0, yaw_origin);
            usleep(20000);
        }else{
            CDJIDrone->attitude_control(0x40, 0, 0, 0, yaw_origin);
            usleep(10000);
        }
    }
}

void IARCMission::missionGoRight(){
    ros::spinOnce();
    ROS_WARN("in Goright!");
    float delta_vx, delta_vy;
    //CDJIDrone->request_sdk_permission_control();
    for(int i = 0;i < time_step_left;i++){
        ros::spinOnce();
        if(obstacle_emergency){
            delta_vx = obstacle_velocity[0];
            delta_vy = obstacle_velocity[1];
            CDJIDrone->attitude_control(0x40, delta_vx, delta_vy, 0, yaw_origin);
            usleep(20000);
        }else{
            CDJIDrone->attitude_control( Flight::HorizontalLogic::HORIZONTAL_POSITION |
                Flight::VerticalLogic::VERTICAL_VELOCITY |
                Flight::YawLogic::YAW_ANGLE |
                Flight::HorizontalCoordinate::HORIZONTAL_BODY |
                Flight::SmoothMode::SMOOTH_ENABLE,
                0, horizon_step_left, 0, yaw_origin);
            usleep(20000);
        }
    }
    mission_flag = 1;
     while((ros::ok())&&quadState==GO_RIGHT&&control_state==1&&mission_flag==1){
        ros::spinOnce();
        if(obstacle_emergency){
            delta_vx = obstacle_velocity[0];
            delta_vy = obstacle_velocity[1];
            CDJIDrone->attitude_control(0x40, delta_vx, delta_vy, 0, yaw_origin);
            usleep(20000);
        }else{
            CDJIDrone->attitude_control(0x40, 0, 0, 0, yaw_origin);
            usleep(10000);
        }
    }    
}

void IARCMission::missionGoLittleRight(){
    ros::spinOnce();
    ROS_WARN("in Go little right!");
    float delta_vx, delta_vy;
    //CDJIDrone->request_sdk_permission_control();
    for(int i = 0;i < time_step_left*little_part_left;i++){
        ros::spinOnce();
        if(obstacle_emergency){
            delta_vx = obstacle_velocity[0];
            delta_vy = obstacle_velocity[1];
            CDJIDrone->attitude_control(0x40, delta_vx, delta_vy, 0, yaw_origin);
            usleep(20000);
        }else{
            CDJIDrone->attitude_control( Flight::HorizontalLogic::HORIZONTAL_POSITION |
                Flight::VerticalLogic::VERTICAL_VELOCITY |
                Flight::YawLogic::YAW_ANGLE |
                Flight::HorizontalCoordinate::HORIZONTAL_BODY |
                Flight::SmoothMode::SMOOTH_ENABLE,
                0, horizon_step_left, 0, yaw_origin);
            usleep(20000);
        }
    }
    mission_flag = 1;
     while((ros::ok())&&quadState==GO_LITTLE_RIGHT&&control_state==1&&mission_flag==1){
        ros::spinOnce();
        if(obstacle_emergency){
            delta_vx = obstacle_velocity[0];
            delta_vy = obstacle_velocity[1];
            CDJIDrone->attitude_control(0x40, delta_vx, delta_vy, 0, yaw_origin);
            usleep(20000);
        }else{
            CDJIDrone->attitude_control(0x40, 0, 0, 0, yaw_origin);
            usleep(10000);
        }
    }    
}

void IARCMission::missionGoUp(){
    ros::spinOnce();
    ROS_WARN("in GoUp!");
    float delta_vx, delta_vy;
    //CDJIDrone->request_sdk_permission_control();
    for(int i = 0;i < time_step_up;i++)
    {
        CDJIDrone->attitude_control(0x00, 0, 0, vertical_step,yaw_origin );
        usleep(20000);
    }
    mission_flag = 1;
     while((ros::ok())&&quadState==GO_UP&&control_state==1&&mission_flag==1){
        ros::spinOnce();
        if(obstacle_emergency){
            delta_vx = obstacle_velocity[0];
            delta_vy = obstacle_velocity[1];
            CDJIDrone->attitude_control(0x40, delta_vx, delta_vy, 0, yaw_origin);
            usleep(20000);
        }else{
            CDJIDrone->attitude_control(0x40, 0, 0, 0, yaw_origin);
            usleep(10000);
        }
    }    
}

void IARCMission::missionGoDown(){
    ros::spinOnce();
    ROS_WARN("in GoDown!");
    float delta_vx, delta_vy;
    CDJIDrone->request_sdk_permission_control();
    for(int i = 0;i < time_step_up;i++)
    {
        CDJIDrone->attitude_control( 0x00, 0, 0, -vertical_step,yaw_origin );
        usleep(20000);
    }
    mission_flag = 1;
    while((ros::ok())&&quadState==GO_DOWN&&control_state==1&&mission_flag==1){
        ros::spinOnce();
        if(obstacle_emergency){
            delta_vx = obstacle_velocity[0];
            delta_vy = obstacle_velocity[1];
            CDJIDrone->attitude_control(0x40, delta_vx, delta_vy, 0, yaw_origin);
            usleep(20000);
        }else{
            CDJIDrone->attitude_control(0x40, 0, 0, 0, yaw_origin);
            usleep(10000);
        }
    }    
}

void IARCMission::missionGoLittleDown(){
    ros::spinOnce();
    ROS_WARN("in Go little Down!");
    float delta_vx, delta_vy;
    //CDJIDrone->request_sdk_permission_control();
    for(int i = 0;i < time_step_up*little_part_left;i++)
    {
        CDJIDrone->attitude_control( 0x00, 0, 0, -vertical_step,yaw_origin );
        usleep(20000);
    }
    mission_flag = 1;
    while((ros::ok())&&quadState==GO_LITTLE_DOWN&&control_state==1&&mission_flag==1){
        ros::spinOnce();
        if(obstacle_emergency){
            delta_vx = obstacle_velocity[0];
            delta_vy = obstacle_velocity[1];
            CDJIDrone->attitude_control(0x40, delta_vx, delta_vy, 0, yaw_origin);
            usleep(20000);
        }else{
            CDJIDrone->attitude_control(0x40, 0, 0, 0, yaw_origin);
            usleep(10000);
        }
    }    
}

void IARCMission::missionGoClockwise(){
    ros::spinOnce();
    ROS_WARN("in GoClockwise!");
    float delta_vx, delta_vy;
    //CDJIDrone->request_sdk_permission_control();
   for(int i = 0;i < time_step_clockwise;i++)
    {
        CDJIDrone->attitude_control( Flight::HorizontalLogic::HORIZONTAL_POSITION |
                Flight::VerticalLogic::VERTICAL_VELOCITY |
                Flight::YawLogic::YAW_ANGLE |
                Flight::HorizontalCoordinate::HORIZONTAL_BODY |
                Flight::SmoothMode::SMOOTH_ENABLE,
                0, 0, 0, yaw_origin );
        yaw_origin+=yaw_step;
        usleep(20000);
    }
    mission_flag = 1;
    while((ros::ok())&&quadState==GO_CLOCKWISE&&control_state==1&&mission_flag==1){
        ros::spinOnce();
        if(obstacle_emergency){
            delta_vx = obstacle_velocity[0];
            delta_vy = obstacle_velocity[1];
            CDJIDrone->attitude_control(0x40, delta_vx, delta_vy, 0, yaw_origin);
            usleep(20000);
        }else{
            CDJIDrone->attitude_control(0x40, 0, 0, 0, yaw_origin);
            usleep(10000);
        }
    }    
}

void IARCMission::missionGoAntiClockwise(){
    ros::spinOnce();
    ROS_WARN("in GoAntiClockwise!");
    float delta_vx, delta_vy;
    //CDJIDrone->request_sdk_permission_control();
    for(int i = 0;i < time_step_clockwise;i++)
    {
        CDJIDrone->attitude_control( Flight::HorizontalLogic::HORIZONTAL_POSITION |
                Flight::VerticalLogic::VERTICAL_VELOCITY |
                Flight::YawLogic::YAW_ANGLE |
                Flight::HorizontalCoordinate::HORIZONTAL_BODY |
                Flight::SmoothMode::SMOOTH_ENABLE,
                0, 0, 0, yaw_origin );
        yaw_origin-=yaw_step;
        usleep(20000);
    }
    mission_flag = 1;
    while((ros::ok())&&quadState==GO_ANTICLOCKWISE&&control_state==1&&mission_flag==1){
        ros::spinOnce();
        if(obstacle_emergency){
            delta_vx = obstacle_velocity[0];
            delta_vy = obstacle_velocity[1];
            CDJIDrone->attitude_control(0x40, delta_vx, delta_vy, 0, yaw_origin);
            usleep(20000);
        }else{
            CDJIDrone->attitude_control(0x40, 0, 0, 0, yaw_origin);
            usleep(10000);
        }
    }    
}

void IARCMission::missionStartCure(){
    CDJIDrone->request_sdk_permission_control();
    if_cure.start_cure  = 1;
    float delta_vx, delta_vy;
    startcure_pub.publish(if_cure);
    mission_flag = 1;
    ROS_WARN("in Cure!");
    while((ros::ok())&&quadState==START_CURE&&control_state==1&&mission_flag==1){
        ros::spinOnce();
        if(obstacle_emergency){
            delta_vx = obstacle_velocity[0];
            delta_vy = obstacle_velocity[1];
            CDJIDrone->attitude_control(0x40, delta_vx, delta_vy, 0, yaw_origin);
            usleep(20000);
        }else{
            CDJIDrone->attitude_control(0x40, 0, 0, 0, yaw_origin);
            usleep(10000);
        }
    }
    if_cure.start_cure  = 1;
    startcure_pub.publish(if_cure);
}

bool IARCMission::mission_takeoff()
{
	ros::spinOnce();
    float delta_vx, delta_vy;
	CDJIDrone->request_sdk_permission_control();
    ROS_WARN("Take off!");
	CDJIDrone->takeoff();
    mission_flag = 1;
    while((ros::ok())&&quadState==TAKE_OFF&&control_state==1&&mission_flag==1){
        ros::spinOnce();
        if(obstacle_emergency){
            delta_vx = obstacle_velocity[0];
            delta_vy = obstacle_velocity[1];
            CDJIDrone->attitude_control(0x40, delta_vx, delta_vy, 0, yaw_origin);
            usleep(20000);
        }else{
            CDJIDrone->attitude_control(0x40, 0, 0, 0, yaw_origin);
            usleep(10000);
        }
    }    
	return true;
}

void IARCMission::missionStartVideo(){
    VideoCapture cap(0);
    if (!cap.isOpened())
    {
        cout << "Error opening video stream" << endl;
    }

    int frame_width = cap.get(CV_CAP_PROP_FRAME_WIDTH);
    int frame_height = cap.get(CV_CAP_PROP_FRAME_HEIGHT);

    VideoWriter video("/home/nvidia/Desktop/video.avi", CV_FOURCC('M', 'J', 'P', 'G'), 25, Size(frame_width, frame_height));
    while (1)
    {
        Mat frame;             
        cap >> frame;
        if (frame.empty())
            break;

        video.write(frame);

        imshow("Frame", frame);

        // 按ESC 推出录制
        char c = (char)waitKey(1);
        if (c == 27)
            break;
    }

    cap.release();
    video.release();             
    destroyAllWindows();
}


bool IARCMission::mission_land()
{
	//CDJIDrone->request_sdk_permission_control();
    ROS_WARN("Landing!");
	for(int i = 0; i < 100; i ++) 
	{
		CDJIDrone->attitude_control(0x00, 0, 0, -0.6, yaw_origin);
		usleep(20000);
	}
    while((ros::ok())&&quadState==LAND&&control_state==1){
        ros::spinOnce();
        CDJIDrone->attitude_control(0x40, 0, 0, 0, yaw_origin);
        usleep(10000);
    }   
	return true;
}

};

