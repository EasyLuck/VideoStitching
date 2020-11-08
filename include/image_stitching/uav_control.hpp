#ifndef UAV_CONTROL_HPP
#define UAV_CONTROL_HPP

#ifndef Q_MOC_RUN
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Point.h>
#include <tf/tf.h>
#endif

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

  double a_bit;
  double b_bit;
  double e2_bit;
  double PI_bit;
  int gpsReceiveCnt;
  double start_point_lat , start_point_lon;

  //GPS  0 元素不使用 从1开始
  double gps_yz[4][2];
  bool gps_status[4];

  geometry_msgs::Point currentPosition[4];
  double currentYaw[4];
  double yawOffset[4];
  double yawOffset_21,yawOffset_23;

  double targetOverlap_left,targetOverlap_right;
  double currentOverlap_left,currentOverlap_right;
  double overlap_upper,overlap_lower;
  bool flayState_left[2],flayState_right[2];
  bool uav3flayState_left[2],uav3flayState_right[2];
  void run();
  void gps_xyz(double lat2, double lon2, double *x_east, double *y_north, double start_point_lat_rad, double start_point_lon_rad);
  void uav_LRcontrol();
  void uav_FBcontrol();

  void limiter(double *input, double max, double min);
  //订阅回调函数
//  void receiveImage_cb(const sensor_msgs::ImageConstPtr& msg);
//  void receiveBatteryData_cb(const CommonCommonStateBatteryStateChanged::ConstPtr& msg);
public Q_SLOTS:
  void deal_overlapRateSignal(double overlapRate_left,double overlapRate_right);
  void deal_uavgpsDataSignal(int UAVx, double latitude, double longitude);
  void deal_uavodomDataSignal(int UAVx, geometry_msgs::Pose currentPose);  // 发送GPS信息

Q_SIGNALS://Qt信号
  void forwardSignal(int,bool);
  void backwardSignal(int,bool);
  void flayLeftSignal(int,bool);
  void flayRightSignal(int,bool);
  void flayUpSignal(int,bool);
  void flayDownSignal(int,bool);
  void turnLeftSignal(int,bool);
  void turnRightSignal(int,bool);

  void uav_FBcontrolSignal(double, double, double, double);


private:


};

}  // namespace image_stitching


#endif // UAV_CONTROL_HPP
