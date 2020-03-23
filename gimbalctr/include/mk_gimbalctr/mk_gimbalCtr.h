#include "ros/ros.h"
#include "std_msgs/String.h"
#include <iostream>
#include "math.h"
#include <std_msgs/Int8.h>
#include "iarc_msgs/RoiPos.h"
#include "iarc_msgs/Pwm.h"
#include "iarc_msgs/Start_Cure.h"
#include <queue>
//#include "pidCtr.h"
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>

namespace mk_gimbalctr{
  enum GimbalState{Gimbal_init, Gimbal_search, Gimbal_keepoversee, Gimbal_track, };
  using namespace std;

  // struct PoseXY
  // {
  //   double x;
  //   double y;
  // };

  struct PoseAng
  {
    double pitch;//leftandright
    double roll;//upanddown
    double yaw;
  };

  struct distXY
  {
    double dx;
    double dy;
  };

  class Gimbal_control_mk{
  public:
    //init
    double piMelon_;
    int if_cure;
    //double timeBegin_,timeDuration_;
    //callback
    bool ifFoundHelmet_;
    bool ifFinishTrack_;
    //gimbal action func
    int gimbalState_;
    distXY gimbalPosDist_;
    PoseAng gimbalPosAngle_;
    PoseAng gimbalPosDeltaAngle_;
    PoseAng gimbalPosMaxAngle_;
    PoseAng gimbalPosMinAngle_;
    iarc_msgs::Pwm PWM_;
    //generous_parameter
    //double timeBeginingOfAction_, timeDurationOfAction_;
    //bool ifBeforeStart_;
    double stepRoll_, stepPitch_, stepYaw_;
    double offset_, sleepTime_;
    int pitchDiff_,yawDiff_,rollDiff_;
    int pitchLow_,yawLow_,rollLow_;

    int pitchOffset, yawOffset, rollOffset;
    //gimbalaction_init
    //double initTime_;
    //gimbalaction_search
    //double gimbalSearchRange_;
    //double searchTime_;
    //gimbalaction_keepoversee
    //double keepTime_;
    //gimbalaction_track
    //node
    ros::NodeHandle nh_;
    ros::NodeHandle nhParam_;
    ros::Subscriber taskProceedSub_, RoiPosSub_;
    ros::Publisher pwmPub_;
    Gimbal_control_mk();
    ~Gimbal_control_mk();
    //init
    void init();
    void mainloop();
    //main func

    /**
     * @brief gimbal controller (infinite state machine)       
     */
    void gimbalActions();

    /**
     * @brief 
     * 1. 
     * 2.
     */
    void publishAllInfo();
    //callback func
    void taskProceed_Callback(const iarc_msgs::Start_Cure::ConstPtr &if_cure);
    void RoiPos_Callback(const iarc_msgs::RoiPos &RoiPos);
    //publish func
    void pwmPub(PoseAng finalAngle);
    //gimbalaction func
    //gimbalaction_init
    void gimbalAction_init();
    //search or track
    void searchOrTrack();
    //gimbalaction_search
    void gimbalAction_search();
    //gimbalaction_keepoversee
    void gimbalAction_keepoversee();
    //gimbalaction_track
    void gimbalAction_track();
    
    void publishIfTrackFinished();
  };



}
