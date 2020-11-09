#ifndef UAV_HPP
#define UAV_HPP

#ifndef Q_MOC_RUN
#include <ros/ros.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <tf/tf.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Int32.h>
#include <std_msgs/UInt8.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/NavSatFix.h>
#include <geometry_msgs/Pose.h>
//#include <geometry_msgs/PoseStamped.h>
#include <image_stitching/CommonCommonStateBatteryStateChanged.h>
#endif

#include <QImage>
#include <iostream>
class uav
{
public:
  uav();

  // 发布
  ros::Publisher takeoff_pub;
  ros::Publisher land_pub;
  ros::Publisher cmd_pub;
  ros::Publisher cameraControl_pub;
  // 订阅
  ros::Subscriber state_sub;
  ros::Subscriber batteryData_sub;
  ros::Subscriber gpsData_sub;
  ros::Subscriber odomData_sub;
  image_transport::Subscriber receiveImage_sub;
  // 变量
  cv::Mat receiveImage;//保存接受到的图像
  QImage ImageToQImage;//转换为QImage
  bool receiveImageFlag;//接受到图像为true 处理完设置为false

  geometry_msgs::Pose cuurrentPose;
  nav_msgs::Odometry cuurrentOdom;
  //控制无人机的移动方向
  bool forward;
  bool backward;
  bool flayLeft;
  bool flayRight;
  bool flayUp;
  bool flayDown;
  bool turnLeft;
  bool turnRight;

//  bool isRunning; //节点是否正在运行
  bool isRun;
  geometry_msgs::Twist cmd_vel;
  geometry_msgs::Twist cameraControl_vel;

  double pidout;
  int batteryData;  // 电池电量

  tf::Quaternion q;

  double a_bit;
  double b_bit;
  double e2_bit;
  double PI_bit;

//  ros::Rate *rate;

  void takeoff();
  void land();
  void cmd(double x,double y,double z,double yaw);
  void state_cb(const nav_msgs::Odometry::ConstPtr& msg);
  void cameraControl(double vertical, double horizontal);

  void moveControl();

  void gps_xyz(double lat2, double lon2, double *x_east, double *y_north, double start_point_lat_rad, double start_point_lon_rad);

  double xControl();

public Q_SLOTS:

//Q_SIGNALS://Qt信号
//  void showUavBatteryData(int,bool);  // 发送电池电量信息

};

#endif // UAV_HPP







