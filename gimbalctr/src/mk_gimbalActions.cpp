
#include "mk_gimbalCtr.h"
namespace mk_gimbalctr {
//===========================gimbalaction==================================================
void Gimbal_control_mk::gimbalActions() {
  if (if_cure) {
    cout << "in Cure State!!!" << endl;
    switch (gimbalState_) {
      case Gimbal_init: {
        cout << "gimbalAction : init" << endl;
        gimbalAction_init();
        break;
      }
      case Gimbal_search: {
        cout << "gimbalAction : search" << endl;
        gimbalAction_search();
        break;
      }
      case Gimbal_keepoversee: {
        cout << "gimbalAction : keep oversee" << endl;
        gimbalAction_keepoversee();
        break;
      }
      case Gimbal_track: {
        cout << "gimbalAction : track" << endl;
        gimbalAction_track();
        break;
      }
      default: {
        cout << "wrong gimbal State!!!!" << endl;
        break;
      }
    }
  } else {
    // cout<<"in QRcode State!!!"<<endl;
    // gimbalState_ = Gimbal_keepoversee;
    gimbalAction_keepoversee();
  }
}

//------gimbalact ion functions are all be used for updating variable
// gimbalPos_Target init(): initialize to pos (0,0)  (pitch, yaw)
void Gimbal_control_mk::gimbalAction_init() {
  // cout<<"in init"<<endl;
  gimbalPosAngle_.pitch = 0.0;
  gimbalPosAngle_.roll = 0.0;
  gimbalPosAngle_.yaw = 0.0;
  if (if_cure) {
    gimbalState_ = Gimbal_search;
  } else {
    gimbalState_ = Gimbal_keepoversee;
  }
}

void Gimbal_control_mk::gimbalAction_search() {
  // cout<<"gimbalPosAngle_.pitch = "<<gimbalPosAngle_.pitch<<endl;
  // cout<<"gimbalPosMaxAngle_.pitch = "<<gimbalPosMaxAngle_.pitch<<endl;
  // search here
  int loopTimes = 1000;
  for (; gimbalPosAngle_.pitch > gimbalPosMinAngle_.pitch;
       gimbalPosAngle_.pitch -= stepPitch_) {
    pwmPub(gimbalPosAngle_);
    ros::Duration(0.5).sleep();
    ros::spinOnce();
    if (ifFoundHelmet_ || !if_cure) {
      break;
    }
    for (; gimbalPosAngle_.roll < gimbalPosMaxAngle_.roll;
         gimbalPosAngle_.roll += stepRoll_) {
      /*
      for(int i = 0;i <= loopTimes;++i){
              pwmPub(gimbalPosAngle_);
              if(ifFoundHelmet_){
              break;
          }
            }
      */
      pwmPub(gimbalPosAngle_);
      ros::Duration(0.5).sleep();
      ros::spinOnce();
      if (ifFoundHelmet_ || !if_cure) {
        break;
      }
    }
    for (; gimbalPosAngle_.roll > gimbalPosMinAngle_.roll;
         gimbalPosAngle_.roll -= stepRoll_) {
      pwmPub(gimbalPosAngle_);
      ros::Duration(0.5).sleep();
      ros::spinOnce();
      if (ifFoundHelmet_ || !if_cure) {
        break;
      }
    }
  }
  ros::spinOnce();
  if (ifFoundHelmet_) {
    gimbalState_ = Gimbal_track;
  } else if (!if_cure) {
    gimbalState_ = Gimbal_keepoversee;
  }
  for (; gimbalPosAngle_.pitch < gimbalPosMaxAngle_.pitch;
       gimbalPosAngle_.pitch += stepPitch_) {
    pwmPub(gimbalPosAngle_);
    ros::Duration(0.5).sleep();
    ros::spinOnce();
    if (ifFoundHelmet_ || !if_cure) {
      break;
    }
    for (; gimbalPosAngle_.roll < gimbalPosMaxAngle_.roll;
         gimbalPosAngle_.roll += stepRoll_) {
      pwmPub(gimbalPosAngle_);
      if (ifFoundHelmet_ || !if_cure) {
        break;
      }
    }
    for (; gimbalPosAngle_.roll > gimbalPosMinAngle_.roll;
         gimbalPosAngle_.roll -= stepRoll_) {
      pwmPub(gimbalPosAngle_);
      ros::Duration(0.5).sleep();
      ros::spinOnce();
      if (ifFoundHelmet_ || !if_cure) {
        break;
      }
    }
  }
  ros::spinOnce();
  if (ifFoundHelmet_) {
    gimbalState_ = Gimbal_track;
  } else if (!if_cure) {
    gimbalState_ = Gimbal_keepoversee;
  }
}

void Gimbal_control_mk::gimbalAction_keepoversee() {
  gimbalPosAngle_.pitch = 0.0;
  gimbalPosAngle_.roll = 0.0;
  gimbalPosAngle_.yaw = 0.0;
  pwmPub(gimbalPosAngle_);
  ros::Duration(sleepTime_).sleep();

  if (if_cure) {
    gimbalState_ = Gimbal_search;
  } else {
    gimbalState_ = Gimbal_keepoversee;
  }
}

void Gimbal_control_mk::gimbalAction_track() {
  // cout << " in track "<<endl;s
  // PoseAng gimbalPoseAngleCur = gimbalPosAngle_;
  ros::spinOnce();

  if (gimbalPosDeltaAngle_.pitch > 0 && gimbalPosDeltaAngle_.pitch > offset_) {
    gimbalPosAngle_.pitch -= 1;
  } else if (gimbalPosDeltaAngle_.pitch < 0 &&
             gimbalPosDeltaAngle_.pitch < (-1) * offset_) {
    gimbalPosAngle_.pitch += 1;
  }
  if (gimbalPosDeltaAngle_.roll > 0 && gimbalPosDeltaAngle_.roll > offset_) {
    gimbalPosAngle_.roll += 1;
  } else if (gimbalPosDeltaAngle_.roll < 0 &&
             gimbalPosDeltaAngle_.roll < (-1) * offset_) {
    gimbalPosAngle_.roll -= 1;
  }

  /*
    static double last_delta_roll = 0.0, last_delta_pitch = 0.0, int_delta_roll
    = 0.0, int_delta_pitch = 0.0; const double kp_roll = 1.0,  ki_roll = 0.0,
    kd_roll = 0.0; const double kp_pitch = 1.0, ki_pitch = 0.0, kd_pitch = 0.0;
    


    gimbalPosAngle_.roll = kp_roll * gimbalPosDeltaAngle_.roll + kp_roll *
    (gimbalPosDeltaAngle_.roll - last_delta_roll) + ki_roll * int_delta_roll;
    gimbalPosAngle_.pitch = kp_pitch * gimbalPosDeltaAngle_.pitch + kp_pitch *
    (gimbalPosDeltaAngle_.pitch - last_delta_pitch) + ki_pitch *
    int_delta_pitch; last_delta_roll = gimbalPosDeltaAngle_.roll;
    last_delta_pitch = gimbalPosDeltaAngle_.pitch;
    int_delta_roll += gimbalPosDeltaAngle_.roll;
    int_delta_pitch += gimbalPosDeltaAngle_.pitch;
    if(int_delta_roll > 6) int_delta_roll = 6;
    if(int_delta_roll > -6) int_delta_roll = -6;
    if(int_delta_pitch > 6) int_delta_pitch = 6;
    if(int_delta_pitch > -6) int_delta_pitch = -6;*/

  // gimbalPosAngle_.pitch -= gimbalPosDeltaAngle_.pitch;
  // gimbalPosAngle_.roll += gimbalPosDeltaAngle_.roll;
  // gimbalPosAngle_.yaw += gimbalPosDeltaAngle_.yaw;
  /*cout<<"gimbalPosAngle_.pitch = "<<gimbalPosAngle_.pitch<<endl;
  cout<<"gimbalPosAngle_.roll = "<<gimbalPosAngle_.roll<<endl;
  cout<<"gimbalPosAngle_.yaw = "<<gimbalPosAngle_.yaw<<endl;*/
  if (gimbalPosAngle_.pitch >= gimbalPosMaxAngle_.pitch)
    gimbalPosAngle_.pitch = gimbalPosMaxAngle_.pitch;
  if (gimbalPosAngle_.roll >= gimbalPosMaxAngle_.roll)
    gimbalPosAngle_.roll = gimbalPosMaxAngle_.roll;
  // if(gimbalPosAngle_.yaw >= gimbalPosMaxAngle_.yaw) gimbalPosAngle_.yaw =
  // gimbalPosMaxAngle_.yaw;
  if (gimbalPosAngle_.pitch <= gimbalPosMinAngle_.pitch)
    gimbalPosAngle_.pitch = gimbalPosMinAngle_.pitch;
  if (gimbalPosAngle_.roll <= gimbalPosMinAngle_.roll)
    gimbalPosAngle_.roll = gimbalPosMinAngle_.roll;
  // if(gimbalPosAngle_.yaw <= gimbalPosMinAngle_.yaw) gimbalPosAngle_.yaw =
  // gimbalPosMinAngle_.yaw;

  ros::Duration(sleepTime_).sleep();
  pwmPub(gimbalPosAngle_);
  // cout<<"in track: gimbalPos_Target.pitch: "<<gimbalPos_Target.pitch<<"
  // gimbalPos_Target.yaw : "<<gimbalPos_Target.yaw <<endl;

  if (!ifFoundHelmet_) {
    gimbalState_ = Gimbal_search;
  } else {
    ifFinishTrack_ = true;
  }
}

}  // namespace mk_gimbalctr
