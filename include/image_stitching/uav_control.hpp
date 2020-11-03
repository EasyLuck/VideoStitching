#ifndef UAV_CONTROL_HPP
#define UAV_CONTROL_HPP

#include <iostream>
#include <QThread>
#include <QImage>
#include <QMutex>
#include <QWaitCondition>
#include <QTime>
//#include "uav.hpp"

namespace image_stitching {

/*****************************************************************************
** Class
*****************************************************************************/

class uav_control : public QThread
{
    Q_OBJECT
public:
  uav_control();
  virtual ~uav_control();

  /* 用于控制拼接进程 */
  mutable QMutex autoFly_mutex_;


  /* true --> 执行拼接程序 */
  bool isAutoFly;
  bool autoFlyThreadStatue;

  double targetOverlap_left,targetOverlap_right;
  double currentOverlap_left,currentOverlap_right;
  double overlap_upper,overlap_lower;
  bool flayState_left[2],flayState_right[2];
  bool uav3flayState_left[2],uav3flayState_right[2];
  void run();


  //订阅回调函数
//  void receiveImage_cb(const sensor_msgs::ImageConstPtr& msg);
//  void receiveBatteryData_cb(const CommonCommonStateBatteryStateChanged::ConstPtr& msg);
public Q_SLOTS:
  void deal_overlapRateSignal(double overlapRate_left,double overlapRate_right);
Q_SIGNALS://Qt信号
  void forwardSignal(int,bool);
  void backwardSignal(int,bool);
  void flayLeftSignal(int,bool);
  void flayRightSignal(int,bool);
  void flayUpSignal(int,bool);
  void flayDownSignal(int,bool);
  void turnLeftSignal(int,bool);
  void turnRightSignal(int,bool);


private:


};

}  // namespace image_stitching


#endif // UAV_CONTROL_HPP
