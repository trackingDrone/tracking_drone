#include "mk_gimbalCtr.h"
namespace mk_gimbalctr{
using namespace std;

void Gimbal_control_mk::mainloop(){
  //ros::Rate loop_rate_class(loopRate_);
  
  while(ros::ok()){
    //timeDuration_ = ros::Time::now().toSec() - timeBegin_;

    //refreshDetectionInfo();
    gimbalActions();  
    // publishAllInfo();
    ros::spinOnce();
    //loop_rate_class.sleep();
  }

}
}

int main(int argc,char** argv){
  ros::init(argc,argv,"mk_gimbalctr");
  mk_gimbalctr::Gimbal_control_mk gimbal_ctr;
  gimbal_ctr.mainloop();
  return 0;
}
