#include <ros/ros.h>
#include <string>
#include <std_msgs/String.h>
#include <sstream>
#include "../include/image_stitching/uav2_node.hpp"
#include <QMatrix>
/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace image_stitching {
using namespace std;
/*****************************************************************************
** Implementation
*****************************************************************************/

uav2::uav2(int argc, char** argv ) :
  init_argc(argc),
  init_argv(argv)
  {
  isRun = false;
}

uav2::~uav2() {
    if(ros::isStarted()) {
      ros::shutdown(); // explicitly needed since we use ros::start();
      ros::waitForShutdown();
    }
  wait();
}

bool uav2::init(std::string uavname)
{
  ros::init(init_argc,init_argv,"uav2_node");
  if ( ! ros::master::check() )
  {
    return false;
  }

//  ros::start(); // explicitly needed since our nodehandle is going out of scope.
  ros::NodeHandle nh;// 第一次创建节点时会自动调用start()
  image_transport::ImageTransport it(nh);

  takeoff_pub= nh.advertise<std_msgs::Empty>(uavname + "/takeoff", 1);         // 发布 起飞命令
  land_pub   = nh.advertise<std_msgs::Empty>(uavname + "/land", 1);            // 发布 降落命令
  cmd_pub    = nh.advertise<geometry_msgs::Twist>(uavname + "/cmd_vel", 1);    // 发布 移动命令
  cameraControl_pub = nh.advertise<geometry_msgs::Twist>(uavname + "/camera_control", 1);    // 发布相机控制命令
  batteryData_sub= nh.subscribe(uavname + "/states/common/CommonState/BatteryStateChanged",2,&uav2::receiveBatteryData_cb,this);    // 发布 移动命令
  receiveImage_sub = it.subscribe(uavname + "/image_raw",5,&uav2::receiveImage_cb,this);// 订阅 图像信息
//  gpsData_sub = nh.subscribe<sensor_msgs::NavSatFix>(uavname + "/fix",1,&uav2::gpsData_cb, this);
  odomData_sub = nh.subscribe<nav_msgs::Odometry>(uavname + "/odom",1,&uav2::odomData_cb, this);
  //用于在gazebo仿真中测试
//  receiveImage_sub = it.subscribe("iris_2/camera_Monocular/image_raw",5,&uav2::receiveImage_cb,this);

  start();//开启线程 自动调用run()函数

  return true;
}

void uav2::run()
{
  ros::Rate loop_rate(20);

  ros::Duration(1).sleep();
  cameraControl(20, 0);

  while ( ros::ok() )
  {
    moveControl();

    if(isRun == false) break;

    ros::spinOnce();
    loop_rate.sleep();
  }
  std::cout << "uav2 Ros shutdown, proceeding to close the gui." << std::endl;
  Q_EMIT rosShutdown(2); // used to signal the gui for a shutdown (useful to roslaunch)
}


//接受uav1的图像
void uav2::receiveImage_cb(const sensor_msgs::ImageConstPtr& msg)
{
    try
    {
        receiveImage = cv_bridge::toCvCopy(msg,sensor_msgs::image_encodings::RGB8)->image;
        Q_EMIT uav2RgbimageSignal(receiveImage);
//        ImageToQImage = QImage(receiveImage.data,receiveImage.cols,receiveImage.rows,receiveImage.step[0],QImage::Format_RGB888);
//        Q_EMIT showUav2ImageSignal(ImageToQImage);
//        receiveImageFlag = true ;
    }
    catch (cv_bridge::Exception& e)
    {
        std::cout << "sub1Image_cb could not convert from " << msg->encoding.c_str() << "to 'brg8'." << std::endl;
    }
}
void uav2::receiveBatteryData_cb(const CommonCommonStateBatteryStateChanged::ConstPtr& msg)
{
  batteryData = msg->percent;
  Q_EMIT batteryDataSignal(batteryData,true);
}

void uav2::gpsData_cb(const sensor_msgs::NavSatFix::ConstPtr& msg)
{
  cout << "uav2 gpsData_cb" << endl;
  if(msg->status.status == 0)
    Q_EMIT gpsDataSignal(2,msg->latitude, msg->longitude);
  else
  {
    Q_EMIT gpsDataSignal(2,0.0, 0.0);
    cout << "uav2 no GPS!" << endl;
  }
}
void uav2::odomData_cb(const nav_msgs::Odometry::ConstPtr& msg)
{
//  cout << "uav2 odomData_cb" << endl;

  cuurrentOdom.pose = msg->pose;
  cuurrentOdom.twist = msg->twist;
  Q_EMIT odomDataSignal(2,cuurrentOdom);
}

}  // namespace image_stitching
