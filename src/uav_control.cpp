#include <ros/ros.h>
#include <string>
#include <std_msgs/String.h>
#include <iostream>
#include <math.h>
#include "../include/image_stitching/uav_control.hpp"
#include <QMatrix>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace image_stitching {
using namespace  std;
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

  a_bit = 6378137.0;
  b_bit = 6356752.31414;
  e2_bit = (pow((a_bit*a_bit-b_bit*b_bit),0.5)/a_bit)*(pow((a_bit*a_bit-b_bit*b_bit),0.5)/a_bit);
  PI_bit = 3.1415926;

  gpsReceiveCnt = 0;

  for (int i=0;i<4;i++) {
    for (int j=0;j<2;j++) {
      gps_yz[i][j]=0;
    }
  }
}

uav_control::~uav_control() {

}


void uav_control::run()
{
  while (autoFlyThreadStatue) {
    if(isAutoFly)
    {
      autoFly_mutex_.lock();

      // 控制左右
//      uav_LRcontrol();
//      if(gps_status[1] == true && gps_status[2] == true && gps_status[3] == true)
      {
        uav_FBcontrol();
      }
      autoFly_mutex_.unlock();
      msleep(100);  // 10Hz
    }
    else
    {
//      std::cout << "this is autoFlyThread" << std::endl;
//      cout << "gps_yz:" << endl;
//      cout << "       " << gps_yz[1][0] << " " << gps_yz[1][1] << endl;
//      cout << "       " << gps_yz[2][0] << " " << gps_yz[2][1] << endl;
//      cout << "       " << gps_yz[3][0] << " " << gps_yz[3][1] << endl;
      msleep(100);
    }
  }
  gpsReceiveCnt = 0;
}

void uav_control::deal_overlapRateSignal(double overlapRate_left,double overlapRate_right)
{
  this->currentOverlap_left = overlapRate_left;
  this->currentOverlap_right = overlapRate_right;
}
void uav_control::deal_uavodomDataSignal(int UAVx, geometry_msgs::Pose currentPose)
{
  currentPosition[UAVx] = currentPose.position;
  tf::Quaternion q;
  tf::quaternionMsgToTF(currentPose.orientation,q);
  double roll,pitch, yaw;
  tf::Matrix3x3(q).getRPY(roll,pitch,yaw);
//  yaw = yaw - yawOffset[UAVx];
//  if( yaw > PI_bit)
//    yaw = yaw - 2*PI_bit;
//  if( yaw < -PI_bit)
//    yaw = yaw + 2*PI_bit;
  currentYaw[UAVx] = yaw;

  if(UAVx == 1)
    cout << "roll,pitch, yaw  :  " << roll*180.0/PI_bit << " " << pitch*180.0/PI_bit << " " << yaw*180.0/PI_bit << endl;
  if(UAVx == 2)
    cout << "roll,pitch, yaw  :------------------------------" << roll*180.0/PI_bit << " " << pitch*180.0/PI_bit << " " << yaw*180.0/PI_bit << endl;
//  if(UAVx == 3)
//    cout << "----------------" << roll*180.0/PI_bit << " " << pitch*180.0/PI_bit << " " << yaw*180.0/PI_bit << endl;


  //  cout << "deal_uavodomDataSignal UAVx : " << UAVx << endl;
}

/*
 * 世界坐标系  x--北  y--西
 *                    0       1       2
 * position_global  uav1.x  uav2.x  uav3.x  0
 *                  uav1.y  uav2.y  uav3.y  1
*/
void uav_control::uav_FBcontrol()
{

  double uav1output=0,uav3output=0;
  double yawOut_uav1=0,yawOut_uav3=0;

  double K1_uav12 = 0.2;
  double K3_uav23 = 0.3;
  double yaw_K_uav1 = 0.3;
  double yaw_K_uav3 = 0.3;

  // 计算 yaw 控制输出量
  double pi = 3.1415926;
  double yawErr_12 = currentYaw[2] - currentYaw[1] - yawOffset_21;
  if(abs(yawErr_12) > pi)
  {
    if(yawErr_12 > 0)
      yawErr_12 = yawErr_12 - 2*pi;
    if(yawErr_12 < 0)
      yawErr_12 = yawErr_12 + 2*pi;
  }
  yawOut_uav1 = yaw_K_uav1 * yawErr_12;

  // 计算X轴控制输出量

  Eigen::Matrix<double, 2, 1> uav1Position_global , uav1Position_body;
  Eigen::Matrix<double, 2, 2> uav1_R;
  uav1Position_global << currentPosition[1].x, currentPosition[1].y;
  uav1_R << cos(currentYaw[1]), sin(currentYaw[1]), -sin(currentYaw[1]), cos(currentYaw[1]);
  uav1Position_body = uav1_R * uav1Position_global;

  Eigen::Matrix<double, 2, 1> uav2Position_global , uav2Position_body;
  Eigen::Matrix<double, 2, 2> uav2_R;
  uav2Position_global << currentPosition[2].x, currentPosition[2].y;
  uav2_R << cos(currentYaw[2]), sin(currentYaw[2]), -sin(currentYaw[2]), cos(currentYaw[1]);
  uav2Position_body = uav2_R * uav2Position_global;

  uav1output = K1_uav12*( uav2Position_body(0,0) - uav1Position_body(0,0)); // uav2 - uav1

  limiter(&uav1output,0.2,-0.2);
  limiter(&yawOut_uav1,0.2,-0.2);


  Q_EMIT uav_FBcontrolSignal(uav1output,uav3output,yawOut_uav1,yawOut_uav3);

//  cout << "currentYaw: " << (currentYaw[1] - yawOffset[1])*180/3.14159 << "  "<< (currentYaw[2] - yawOffset[2])*180/3.14159 << endl;
//  cout << "yawOut_uav1: " << yawOut_uav1 << "  "<< yawOut_uav3 << endl;
  cout << "currentPosition:             " << currentPosition[1].x << "  "<< currentPosition[2].x;
  cout << "  --------------------------  " << uav1Position_body(0,0) << "  "<< uav2Position_body(0,0) << endl;

}

void uav_control::limiter(double *input, double max, double min)
{
  if(*input >= max)
    *input =max;
  if(*input <= min)
    *input =min;
}
void uav_control::uav_LRcontrol()
{
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
}


void uav_control::deal_uavgpsDataSignal(int UAVx, double latitude, double longitude)
{
  if(latitude == 0)// GPS有错误
  {
    gps_status[UAVx] = false;
  }
  else
  {
    gps_status[UAVx] = true;
    if(gpsReceiveCnt <= 9)
    {
      if(UAVx == 2)
      {
          start_point_lat += latitude * PI_bit /180.0;
          start_point_lon += longitude * PI_bit /180.0;
          if(gpsReceiveCnt == 9)
          {
              start_point_lat = start_point_lat / 10.0;
              start_point_lon = start_point_lon / 10.0;
              cout << "start_point_lat start_point_lon  " << start_point_lat << "  " << start_point_lon << endl;
          }
          gpsReceiveCnt++;
      }
    }
    else
    {
      double x_east, y_north;
      gps_xyz(latitude,longitude,&x_east,&y_north,start_point_lat,start_point_lon);
      gps_yz[UAVx][0] = x_east;
      gps_yz[UAVx][1] = y_north;
    }
  }
}

void uav_control::gps_xyz(double lat2, double lon2, double *x_east, double *y_north, double start_point_lat_rad, double start_point_lon_rad)
{

    double lat2_rad = lat2 * PI_bit /180.0;
    double lon2_rad = lon2 * PI_bit /180.0;
    // cout << "lat2_rad" << lat2_rad << "  " << lon2_rad << endl;

    double dB = lat2_rad - start_point_lat_rad;
    double dL = lon2_rad - start_point_lon_rad;
    double Bm = (start_point_lat_rad + lat2_rad) /2;

    double mm = a_bit * (1-e2_bit)*pow(1-e2_bit*sin(Bm)*sin(Bm), -1.5);
    double nm = a_bit * pow(1-e2_bit*sin(Bm)*sin(Bm), -0.5);

    double dZ = cos(Bm) *nm *dL;  	//指向正东
    double dX = mm *dB;          		//指向正北

    *x_east = dZ;
    *y_north = dX;
    // cout << "x_east y_north  " << dZ << "  " << dX << endl;
}

}  // namespace image_stitching
