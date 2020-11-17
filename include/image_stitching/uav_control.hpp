#ifndef UAV_CONTROL_HPP
#define UAV_CONTROL_HPP

#ifndef Q_MOC_RUN
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <tf/tf.h>
#endif

#include <iostream>
#include <QThread>
#include <QImage>
#include <QMutex>
#include <QWaitCondition>
#include <QTime>

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>

//#include "uav.hpp"

namespace image_stitching {

/*****************************************************************************
** Class
*****************************************************************************/

class uav_control : public QThread
{
    Q_OBJECT
public:
  uav_control(QWidget *parent = 0);
  virtual ~uav_control();

  /* 用于控制拼接进程 */
  mutable QMutex autoFly_mutex_;


  /* true --> 执行拼接程序 */
  bool isAutoFly;
  bool autoFlyThreadStatue;

  double PI;

  struct PID
  {
      double kp;
      double ki;
      double kd;

      double out;
      double outMax;
      double outMin;

      double error;
      double error_last;
      double dt;
  };
  struct uavPID
  {
    PID x;
    PID y;
    PID z;
    PID yaw;
  };

  uavPID uav1pid,uav2pid,uav3pid;

   //GPS  0 元素不使用 从1开始
  double a_bit;
  double b_bit;
  double e2_bit;
  int gpsReceiveCnt;
  double start_point_lat , start_point_lon;
  double gps_yz[4][2];
  bool gps_status[4];

  // odom 控制   uav1 -- 1   uav2 -- 2   uav3 -- 3 (元素为4的数组，0元素不使用)
  geometry_msgs::Point currentPosition[4];
  geometry_msgs::Twist currentVelocity[4];
//  geometry_msgs::Point lastPosition[4];
  geometry_msgs::Pose uav2targetPose;

  double currentYaw[4];
  double yawOffset[4];              // 与正北方向的偏差
  double yawOffset_21,yawOffset_23;
  double y_Offset_21,y_Offset_23;   //当前位置偏差
  double y_Offset_21_init, y_Offset_23_init;  //  初始位置偏差，用于计算global坐标系
  bool setYawOffset_ok;             //标记是否已经校准yaw
  double delatT;
  geometry_msgs::Twist uav1TargetVelocity,uav2TargetVelocity,uav3TargetVelocity;

  // 是否手动调整
  bool is_manualControl[4];

  // 根据图像进行调整
  double targetOverlap_left,targetOverlap_right;
  double currentOverlap_left[11],currentOverlap_right[11];
  double overlap_upper,overlap_lower;
  bool flayState_left[2],flayState_right[2];
  bool uav3flayState_left[2],uav3flayState_right[2];
  bool stitchingErr_right,stitchingErr_left;

  bool isStitching;         //  是否启动图像拼接
  bool is_imageControl[4];  //  是否正在根据拼接效果调整



  void run();
//  void gps_xyz(double lat2, double lon2, double *x_east, double *y_north, double start_point_lat_rad, double start_point_lon_rad);
  void uav_LRcontrol();
  void uav_FBcontrol();

  void limiter(double *input, double max, double min);
  void filter(double left, double right);
public Q_SLOTS:
  void deal_overlapRateSignal(double overlapRate_left,double overlapRate_right);
//  void deal_uavgpsDataSignal(int UAVx, double latitude, double longitude);
  void deal_uavodomDataSignal(int UAVx, nav_msgs::Odometry currentOdom);  // 发送GPS信息

Q_SIGNALS://Qt信号
  void forwardSignal(int,bool);
  void backwardSignal(int,bool);
  void flayLeftSignal(int,bool);
  void flayRightSignal(int,bool);
  void flayUpSignal(int,bool);
  void flayDownSignal(int,bool);
  void turnLeftSignal(int,bool);
  void turnRightSignal(int,bool);

  void uavTargetVelocitySignal(geometry_msgs::Twist, geometry_msgs::Twist,geometry_msgs::Twist);


private:


};

}  // namespace image_stitching


#endif // UAV_CONTROL_HPP
