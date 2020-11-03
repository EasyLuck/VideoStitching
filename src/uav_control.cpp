#include <ros/ros.h>
#include <string>
#include <std_msgs/String.h>
#include <sstream>
#include "../include/image_stitching/uav_control.hpp"
#include <QMatrix>
/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace image_stitching {

/*****************************************************************************
** Implementation
*****************************************************************************/

uav_control::uav_control(){
  isAutoFly = false;
  autoFlyThreadStatue = true;

  // 目标重合度
  targetOverlap_left = 70;
  targetOverlap_right = 70;

  overlap_upper = 5;
  overlap_lower = -5;

  flayState_left[0] = false;
  flayState_left[1] = false;

  flayState_right[0] = false;
  flayState_right[1] = false;
}

uav_control::~uav_control() {
}


void uav_control::run()
{
  while (autoFlyThreadStatue) {
    if(isAutoFly)
    {
      autoFly_mutex_.lock();

      if(currentOverlap_left > targetOverlap_left + overlap_upper)
      {
        // 左飞
        flayState_left[0] = true;
        flayState_right[0] = false;
      }
      else if(currentOverlap_left < targetOverlap_left + overlap_lower)
      {
        // 右飞
        flayState_right[0] = true;
        flayState_left[0] = false;
      }
      else
      {
        flayState_left[0] = false;
        flayState_right[0] = false;
      }

      // 飞行命令发生变化 重新发送控制指令
      if(flayState_left[0] != flayState_left[1])
        flayLeftSignal(1,flayState_left[0]);
      if(flayState_right[0] != flayState_right[1])
        flayRightSignal(1,flayState_right[0]);

      // 更新flayState_xx[1]
      flayState_left[1] = flayState_left[0];
      flayState_right[1] = flayState_right[0];

      // ****************uav3****************
      if(currentOverlap_right > targetOverlap_right + overlap_upper)
      {
        // 左飞
        uav3flayState_left[0] = true;
        uav3flayState_right[0] = false;
      }
      else if(currentOverlap_right < targetOverlap_right + overlap_lower)
      {
        // 右飞
        uav3flayState_right[0] = true;
        uav3flayState_left[0] = false;
      }
      else
      {
        uav3flayState_left[0] = false;
        uav3flayState_right[0] = false;
      }

      // 飞行命令发生变化 重新发送控制指令
      if(uav3flayState_left[0] != uav3flayState_left[1])
        flayLeftSignal(1,uav3flayState_left[0]);
      if(uav3flayState_right[0] != uav3flayState_right[1])
        flayRightSignal(1,uav3flayState_right[0]);

      // 更新flayState_xx[1]
      uav3flayState_left[1] = uav3flayState_left[0];
      uav3flayState_right[1] = uav3flayState_right[0];


      autoFly_mutex_.unlock();
      msleep(100);  // 10Hz
    }
    else
    {
      std::cout << "this is autoFlyThread" << std::endl;
      msleep(500);
    }
  }
}

void uav_control::deal_overlapRateSignal(double overlapRate_left,double overlapRate_right)
{
  this->currentOverlap_left = overlapRate_left;
  this->currentOverlap_right = overlapRate_right;
}




}  // namespace image_stitching
