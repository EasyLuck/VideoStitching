#ifndef UAV3_NODE_HPP
#define UAV3_NODE_HPP

#include <string>
#include <QThread>
#include <QImage>
#include "uav.hpp"

namespace image_stitching {

/*****************************************************************************
** Class
*****************************************************************************/

class uav3 : public QThread, public uav
{
    Q_OBJECT
public:
  uav3(int argc, char** argv );
  virtual ~uav3();

  bool isRunning;

  void run();

  bool init(std::string uavname);

  //订阅回调函数
  void receiveImage_cb(const sensor_msgs::ImageConstPtr& msg);
  void receiveBatteryData_cb(const CommonCommonStateBatteryStateChanged::ConstPtr& msg);

Q_SIGNALS://Qt信号
  void showUav3ImageSignal(QImage);
  void uav3RgbimageSignal(cv::Mat);
  void showUav3BatteryData(int,bool);  // 发送电池电量信息
  void rosShutdown(int);

private:
  int init_argc;
  char** init_argv;

};

}  // namespace image_stitching


#endif // UAV3_NODE_HPP
