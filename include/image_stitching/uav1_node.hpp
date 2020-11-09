#ifndef UAV1_NODE_HPP
#define UAV1_NODE_HPP

#ifndef Q_MOC_RUN
#include <geometry_msgs/Pose.h>
#endif

#include <string>
#include <QThread>
#include <QImage>
#include "uav.hpp"

namespace image_stitching {

/*****************************************************************************
** Class
*****************************************************************************/

class uav1 : public QThread, public uav
{
    Q_OBJECT
public:
  uav1(int argc, char** argv);
  virtual ~uav1();

  void run();

  bool init(std::string uavname);

  //订阅回调函数
  void receiveImage_cb(const sensor_msgs::ImageConstPtr& msg);
  void receiveBatteryData_cb(const CommonCommonStateBatteryStateChanged::ConstPtr& msg);
  void gpsData_cb(const sensor_msgs::NavSatFix::ConstPtr& msg);
  void odomData_cb(const nav_msgs::Odometry::ConstPtr& msg);

Q_SIGNALS://Qt信号
  void showUav1ImageSignal(QImage);   // 发送QImage
  void uav1RgbimageSignal(cv::Mat);   // 发送Opencv转换Rgbimage
  void batteryDataSignal(int,bool);  // 发送电池电量信息
  void gpsDataSignal(int, double, double);  // 发送GPS信息
  void odomDataSignal(int, nav_msgs::Odometry);  // 发送GPS信息
  void rosShutdown(int);

private:
  int init_argc;
  char** init_argv;

};

}  // namespace image_stitching


#endif // UAV1_NODE_HPP
