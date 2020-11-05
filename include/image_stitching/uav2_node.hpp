#ifndef UAV2_NODE_HPP
#define UAV2_NODE_HPP

#include <string>
#include <QThread>
#include <QImage>
#include "uav.hpp"

namespace image_stitching {

/*****************************************************************************
** Class
*****************************************************************************/

class uav2 : public QThread, public uav
{
    Q_OBJECT
public:
  uav2(int argc, char** argv );
  virtual ~uav2();

  bool isRunning;

  void run();

  bool init(std::string uavname);

  //订阅回调函数
  void receiveImage_cb(const sensor_msgs::ImageConstPtr& msg);
  void receiveBatteryData_cb(const CommonCommonStateBatteryStateChanged::ConstPtr& msg);

Q_SIGNALS://Qt信号
  void showUav2ImageSignal(QImage);
  void uav2RgbimageSignal(cv::Mat);
  void showUav2BatteryData(int,bool);  // 发送电池电量信息
  void rosShutdown(int);

private:
  int init_argc;
  char** init_argv;

};

}  // namespace image_stitching


#endif // UAV1_NODE_HPP
