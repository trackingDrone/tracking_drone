#include "mk_gimbalCtr.h"
namespace mk_gimbalctr{
  using namespace std;

  Gimbal_control_mk::Gimbal_control_mk():
    nhParam_("~")
  {
    init();
  }
  Gimbal_control_mk::~Gimbal_control_mk(){}
  
  void Gimbal_control_mk::init(){
    //init
    piMelon_ = 3.141592654;
    //timeBegin_ = ros::Time::now().toSec();
    //timeDuration_ = timeBegin_;
    //callback
    ifFoundHelmet_ = false;
    ifFinishTrack_ = false;
    gimbalState_ = 0;
    gimbalPosDist_.dx = 100.0;gimbalPosDist_.dy = 100.0;
    gimbalPosAngle_.pitch = 0.0; gimbalPosAngle_.roll = 0.0;gimbalPosAngle_.yaw = 0.0;
    gimbalPosDeltaAngle_.pitch = 1.0; gimbalPosDeltaAngle_.roll = 1.0; gimbalPosDeltaAngle_.yaw = 0.0;
    gimbalPosMaxAngle_.pitch = 10; gimbalPosMaxAngle_.roll = 25;gimbalPosMaxAngle_.yaw = 90;
    gimbalPosMinAngle_.pitch = -15; gimbalPosMinAngle_.roll = -25;gimbalPosMinAngle_.yaw = -90;
    //timeBeginingOfAction_ = 0.0;
    //timeDurationOfAction_ = 0.0;
    if(!nhParam_.getParam("stepRoll",stepRoll_))  stepRoll_ = 5.0;
    if(!nhParam_.getParam("stepPitch",stepPitch_))  stepPitch_ = 5.0;
    if(!nhParam_.getParam("stepYaw",stepYaw_))  stepYaw_ = 1.0;

    if(!nhParam_.getParam("rollOffset",rollOffset))  rollOffset = 90;
    if(!nhParam_.getParam("pitchOffset",pitchOffset))  pitchOffset = 90;
    if(!nhParam_.getParam("yawOffset",yawOffset))  yawOffset = 54;

    if(!nhParam_.getParam("offset",offset_))  offset_ = 0.12;
    if(!nhParam_.getParam("sleepTime",sleepTime_))  sleepTime_ = 0.15;

    if(!nhParam_.getParam("pitchDiff",pitchDiff_))  pitchDiff_ = 16.0;
    if(!nhParam_.getParam("yawDiff",yawDiff_))  yawDiff_ = 2.5;
    if(!nhParam_.getParam("rollDiff",rollDiff_))  rollDiff_ = 16.0;

    if(!nhParam_.getParam("pitchLow",pitchLow_))  pitchLow_ = 1200.0;//1200-1500
    if(!nhParam_.getParam("yawLow",yawLow_))  yawLow_ = 1200.0;
    if(!nhParam_.getParam("rollLow",rollLow_))  rollLow_ = 1100.0;//1100-1900
    //initTime_ = 0.5;
    //if(!nhParam_.getParam("gimbalSearchRange",gimbalSearchRange_)) gimbalSearchRange_ = 0.8;
    //if(!nhParam_.getParam("searchTime",searchTime_)) searchTime_ = 4.0;
    //if(!nhParam_.getParam("keepTime",keepTime_)) keepTime_ = 0.5  ;
    taskProceedSub_ = nh_.subscribe("/iarc_missions/if_start_cure",1,&Gimbal_control_mk::taskProceed_Callback,this);
    RoiPosSub_ = nh_.subscribe("/RoiPose",1,&Gimbal_control_mk::RoiPos_Callback,this);
    pwmPub_ = nh_.advertise<iarc_msgs::Pwm>("Pwm",5);

  }

    void Gimbal_control_mk::taskProceed_Callback(const iarc_msgs::Start_Cure::ConstPtr &msg){
      if_cure = msg->start_cure;
    }

    void Gimbal_control_mk::RoiPos_Callback(const iarc_msgs::RoiPos &RoiPos){
        ifFoundHelmet_ = RoiPos.detectornot;
        //cout<<"ifFoundHelmet_"<<ifFoundHelmet_<<endl;
        gimbalPosDist_.dx = RoiPos.dx;
	//cout<<"Roi.dx:"<<gimbalPosDist_.dx<<endl;
        gimbalPosDist_.dy = RoiPos.dy;
	//cout<<"Roi.dy:"<<gimbalPosDist_.dy<<endl;

        if(!ifFoundHelmet_)
        {
          gimbalState_ = Gimbal_search;
        }
        else {
          gimbalPosDeltaAngle_.pitch = gimbalPosDist_.dy * stepPitch_ / 5;
          gimbalPosDeltaAngle_.roll = gimbalPosDist_.dx * stepRoll_ / 5;
          //cout<<"gimbalPosDeltaAngle_.pitch = "<<gimbalPosDeltaAngle_.pitch<<endl;
          //cout<<"gimbalPosDeltaAngle_.roll = "<<gimbalPosDeltaAngle_.roll<<endl;
          gimbalState_ = Gimbal_track;  
          //gimbalPosDeltaAngle_.roll = 
        }
      //the angle here
      //gimbalPosDeltaAngle_ = ...
    }

    void Gimbal_control_mk::pwmPub(PoseAng finalAngle){
      //cout<<"finalAngle.pitch = "<<finalAngle.pitch<<endl;
      //cout<<"finalAngle.roll = "<<finalAngle.roll<<endl;
      // 2号机中位参数
      PWM_.pitch = finalAngle.pitch + pitchOffset;
      PWM_.yaw = finalAngle.yaw + yawOffset;
      PWM_.roll = finalAngle.roll + rollOffset;
      //cout<<"multi "<<pitchDiff_*finalAngle.pitch+1500<<endl;
      pwmPub_.publish(PWM_);
    }
  

}
