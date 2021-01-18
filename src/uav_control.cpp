#include <ros/ros.h>
#include <string>
#include <std_msgs/String.h>
#include <iostream>
#include <math.h>
#include "../include/image_stitching/uav_control.hpp"
#include <QMatrix>


/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace image_stitching {
using namespace  std;
/*****************************************************************************
** Implementation
*****************************************************************************/

uav_control::uav_control(QWidget *parent)
{
  isAutoFly = false;
  autoFlyThreadStatue = true;

  // 目标重合度
  targetOverlap_left = 70;
  targetOverlap_right = 70;
  currentOverlap_left[10] = targetOverlap_left;
  currentOverlap_right[10] = targetOverlap_right;

  overlap_upper = 5;
  overlap_lower = -5;

  flayState_left[0] = false;
  flayState_left[1] = false;

  flayState_right[0] = false;
  flayState_right[1] = false;

  PI = 3.1415926;

  for (int i=0; i<10;i++)
  {
    currentOverlap_left[i] = 90.0;
    currentOverlap_right[i] = 90.0;
  }

  for (int i=0; i<4; i++)
  {
    currentYaw[i] = 0;
    yawOffset[i] = 0;
    is_manualControl[i] = false;
    is_imageControl[i] = false;

  }
  isStitching = false;
  is_cameraControl = false;


  y_Offset_21_init = 1.5;
  y_Offset_23_init = -1.5;
  y_Offset_21 = y_Offset_21_init;
  y_Offset_23 = y_Offset_23_init;
  yawOffset_21 = 0;
  yawOffset_23 = 0;
  setYawOffset_ok = false;

  currentPosition[1].y = y_Offset_21_init;
  currentPosition[2].y = 0;
  currentPosition[3].y = y_Offset_23_init;

  uav2targetPose.position.x = 0;
  uav2targetPose.position.y = 0;
  uav2targetPose.position.z = 1.5;

  rotationalAngle_left = -6;
  rotationalAngle_right = 6;

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
  while (autoFlyThreadStatue)
  {
    if(isAutoFly)
    {
      autoFly_mutex_.lock();
      // 发送控制信号 （接受者在main_window.cpp)
      uav_FBcontrol();
      // 控制左右
      if(isStitching)
        uav_LRcontrol();
      if(is_cameraControl)
        uav_cameraControl();

      Q_EMIT uavTargetVelocitySignal(uav1TargetVelocity,uav2TargetVelocity,uav3TargetVelocity);

      autoFly_mutex_.unlock();
      msleep(100);  // 10Hz
    }
    else
    {
//      std::cout << "this is autoFlyThread" << std::endl;
      msleep(200);
    }
  }
}

void uav_control::deal_overlapRateSignal(double overlapRate_left,double overlapRate_right)
{
//  this->currentOverlap_left[10] = overlapRate_left;
//  this->currentOverlap_right[10] = overlapRate_right;
  filter(overlapRate_left,overlapRate_right);

}
void uav_control::filter(double left, double right)
{
  double sum_left =0,sum_right =0;
  int cnt = 4;
  currentOverlap_left[9] = left;
  currentOverlap_right[9] = right;
  sum_left = currentOverlap_left[cnt];
  sum_right = currentOverlap_right[cnt];
  for(int i=cnt;i<9;i++)
  {
    currentOverlap_left[i] = currentOverlap_left[i+1];
    sum_left = sum_left + currentOverlap_left[i];
    currentOverlap_right[i] = currentOverlap_right[i+1];
    sum_right = sum_right + currentOverlap_right[i];
  }
  currentOverlap_left[10] = sum_left / (10-cnt);
  currentOverlap_right[10] = sum_right / (10-cnt);
}

void uav_control::deal_uavodomDataSignal(int UAVx, nav_msgs::Odometry currentOdom)
{
  // 当前位置
  currentPosition[UAVx] = currentOdom.pose.pose.position;
  if(UAVx == 1)
  {
    currentPosition[UAVx].x = currentPosition[UAVx].x - y_Offset_21_init * sin(yawOffset[2]);
    currentPosition[UAVx].y = currentPosition[UAVx].y + y_Offset_21_init * cos(yawOffset[2]);
//    currentPosition[UAVx].y = currentPosition[UAVx].y + y_Offset_21_init;
  }
  if(UAVx == 3)
  {
    currentPosition[UAVx].x = currentPosition[UAVx].x - y_Offset_23_init * sin(yawOffset[2]);
    currentPosition[UAVx].y = currentPosition[UAVx].y + y_Offset_23_init * cos(yawOffset[2]);
  }

  // 当前偏航角  四元数 --> 欧拉角
  tf::Quaternion q;
  tf::quaternionMsgToTF(currentOdom.pose.pose.orientation, q);
  double roll,pitch, yaw;
  tf::Matrix3x3(q).getRPY(roll,pitch,yaw);
  currentYaw[UAVx] = yaw;
  // 当前速度
  currentVelocity[UAVx] = currentOdom.twist.twist;

  if(is_manualControl[1] == true)
  {
    // 计算相对距离
    y_Offset_21 = sqrt( (currentPosition[2].x - currentPosition[1].x) * (currentPosition[2].x - currentPosition[1].x)
                      + (currentPosition[2].y - currentPosition[1].y) * (currentPosition[2].y - currentPosition[1].y));
//    cout << "this is manualControl" << endl;
  }

  if(is_manualControl[3] == true)
  {
    // 计算相对距离
    y_Offset_23 = - sqrt( (currentPosition[2].x - currentPosition[3].x) * (currentPosition[2].x - currentPosition[3].x)
                        + (currentPosition[2].y - currentPosition[3].y) * (currentPosition[2].y - currentPosition[3].y));
  }

  if(is_manualControl[2] == true)
  {
    uav2targetPose.position = currentPosition[2];
//    cout << "uav2targetPose.position  " << uav2targetPose.position.x << "  " << uav2targetPose.position.y << endl;
  }

}

void uav_control::uav_cameraControl()
{
//    cout << "this is uav_cameraControl" << endl;
    if(currentOverlap_left[10] > targetOverlap_left + overlap_upper)
    {
      // 左
      rotationalAngle_left = rotationalAngle_left - 0.3;
    }
    else if(currentOverlap_left[10] < targetOverlap_left + overlap_lower)
    {
      // 右
      rotationalAngle_left = rotationalAngle_left + 0.3;
    }
    // ****************uav3****************
    if(currentOverlap_right[10] > targetOverlap_right + overlap_upper)
    {
      // 右
      //  无人机3正在根据图像调整镜头角度
      rotationalAngle_right = rotationalAngle_right+ 0.3;

    }
    else if(currentOverlap_right[10] < targetOverlap_right + overlap_lower)
    {
      // 左
      //  无人机3正在根据图像调整镜头角度
      rotationalAngle_right = rotationalAngle_right - 0.3;
    }
}
/*
 * 世界坐标系  x--北  y--西
 *                    0       1       2
 * position_global  uav1.x  uav2.x  uav3.x  0
 *                  uav1.y  uav2.y  uav3.y  1
*/

void uav_control::uav_FBcontrol()
{
  Eigen::Matrix<double, 2, 1> uavError_global,uavError_body;
  Eigen::Matrix<double, 2, 2> uav_R;

  uav1pid.x.kp = 0.1;
  uav1pid.x.kd = 0.1;
  uav1pid.y.kp = 0.1;
  uav1pid.y.kd = 0.1;
  uav1pid.z.kp = 0.2;
  uav1pid.yaw.kp = 0.3;

  uav3pid.x.kp = 0.1;
  uav3pid.x.kd = 0.1;
  uav3pid.y.kp = 0.1;
  uav3pid.y.kd = 0.1;
  uav3pid.z.kp = 0.2;
  uav3pid.yaw.kp = 0.3;

  uav2pid.x.kp = 0.1;
  uav2pid.y.kp = 0.1;
  uav2pid.z.kp = 0.15;

  if(is_manualControl[1] == false)
  {
    if(is_imageControl[1] == true)
    {
      // 计算相对距离
      y_Offset_21 = sqrt( (currentPosition[2].x - currentPosition[1].x) * (currentPosition[2].x - currentPosition[1].x)
                        + (currentPosition[2].y - currentPosition[1].y) * (currentPosition[2].y - currentPosition[1].y));
    }
    /* *************************** 计算 yaw 控制输出量 *************************** */
    uav1pid.yaw.error = currentYaw[2] - currentYaw[1] - yawOffset_21;
    if(abs(uav1pid.yaw.error) > PI)
    {
      if(uav1pid.yaw.error > 0)
        uav1pid.yaw.error = uav1pid.yaw.error - 2*PI;
      else
        uav1pid.yaw.error = uav1pid.yaw.error + 2*PI;
    }
    uav1TargetVelocity.angular.z = uav1pid.yaw.kp * uav1pid.yaw.error;

    /* *************************** 计算Z轴控制量 *************************** */
    uav1pid.z.error = currentPosition[2].z + 0.5 - currentPosition[1].z; // uav2.z - uav1.z
    uav1TargetVelocity.linear.z = uav1pid.z.kp * uav1pid.z.error;

    /* *************************** 计算X Y轴控制输出量 *************************** */
    uav1pid.x.error = currentPosition[2].x - y_Offset_21 * sin(currentYaw[2]) - currentPosition[1].x;
    uav1pid.y.error = currentPosition[2].y + y_Offset_21 * cos(currentYaw[2]) - currentPosition[1].y;
    uavError_global << uav1pid.x.error , uav1pid.y.error;
    uav_R << cos(currentYaw[1]), sin(currentYaw[1]), -sin(currentYaw[1]), cos(currentYaw[1]);
    uavError_body = uav_R * uavError_global;
    uav1TargetVelocity.linear.x = uav1pid.x.kp * uavError_body(0,0) + uav1pid.x.kd * (0 - currentVelocity[1].linear.x); //
    uav1TargetVelocity.linear.y = uav1pid.y.kp * uavError_body(1,0) + uav1pid.y.kd * (0 - currentVelocity[1].linear.y); //
  }



  if(is_manualControl[3] == false)
  {
    if(is_imageControl[3] == true)
    {
      // 计算相对距离
      y_Offset_23 = - sqrt( (currentPosition[2].x - currentPosition[3].x) * (currentPosition[2].x - currentPosition[3].x)
                          + (currentPosition[2].y - currentPosition[3].y) * (currentPosition[2].y - currentPosition[3].y));
    }
    /* *************************** 计算 yaw 控制输出量 *************************** */
    uav3pid.yaw.error = currentYaw[2] - currentYaw[3] - yawOffset_23;
    if(abs(uav3pid.yaw.error) > PI)
    {
      if(uav3pid.yaw.error > 0)
        uav3pid.yaw.error = uav3pid.yaw.error - 2*PI;
      else
        uav3pid.yaw.error = uav3pid.yaw.error + 2*PI;
    }
    uav3TargetVelocity.angular.z = uav3pid.yaw.kp * uav3pid.yaw.error;
    /* *************************** 计算Z轴控制量 *************************** */
    uav3pid.z.error = currentPosition[2].z + 0.5 - currentPosition[3].z; // uav2.z - uav3.z
    uav3TargetVelocity.linear.z = uav3pid.z.kp * uav3pid.z.error;

    /* *************************** 计算X Y轴控制输出量 *************************** */
    uav3pid.x.error = currentPosition[2].x - y_Offset_23 * sin(currentYaw[2]) - currentPosition[3].x;
    uav3pid.y.error = currentPosition[2].y + y_Offset_23 * cos(currentYaw[2]) - currentPosition[3].y;
    uavError_global << uav3pid.x.error , uav3pid.y.error;
    uav_R << cos(currentYaw[3]), sin(currentYaw[3]), -sin(currentYaw[3]), cos(currentYaw[3]);
    uavError_body = uav_R * uavError_global;
    uav3TargetVelocity.linear.x = uav3pid.x.kp * uavError_body(0,0) + uav3pid.x.kd * (0 - currentVelocity[3].linear.x); //
    uav3TargetVelocity.linear.y = uav3pid.y.kp * uavError_body(1,0) + uav3pid.y.kd * (0 - currentVelocity[3].linear.y); //
  }

//  if(is_manualControl[2] == false)
//  {
//    /* *************************** 计算Z轴控制量 *************************** */
//    uav2pid.z.error = uav2targetPose.position.z - currentPosition[2].z; // uav2.z - uav3.z
//    uav2TargetVelocity.linear.z = uav2pid.z.kp * uav2pid.z.error;

//    /* *************************** 计算X Y轴控制输出量 *************************** */
//    uav2pid.x.error = uav2targetPose.position.x - currentPosition[2].x; // uav2.z - uav3.z
//    uav2TargetVelocity.linear.x = -uav2pid.x.kp * uav2pid.x.error;
//    uav2pid.y.error = uav2targetPose.position.y - currentPosition[2].y; // uav2.z - uav3.z
//    uav2TargetVelocity.linear.y = -uav2pid.y.kp * uav2pid.y.error;
//  }

  // 限制幅值
  limiter(&uav1TargetVelocity.linear.x,0.2,-0.2);
  limiter(&uav1TargetVelocity.linear.y,0.2,-0.2);
  limiter(&uav1TargetVelocity.linear.z,0.2,-0.2);
  limiter(&uav1TargetVelocity.angular.z,0.2,-0.2);
  limiter(&uav3TargetVelocity.linear.x,0.2,-0.2);
  limiter(&uav3TargetVelocity.linear.y,0.2,-0.2);
  limiter(&uav3TargetVelocity.linear.z,0.2,-0.2);
  limiter(&uav3TargetVelocity.angular.z,0.2,-0.2);

  limiter(&uav2TargetVelocity.linear.x,0.2,-0.2);
  limiter(&uav2TargetVelocity.linear.y,0.2,-0.2);
  limiter(&uav2TargetVelocity.linear.z,0.2,-0.2);
  // 打印信息
//  cout << setprecision(6) << "currentPosition: ------------------------------------------  " << currentPosition[1].x << "  "<< currentPosition[1].y << "  "<< currentPosition[3].x << "  "<< currentPosition[3].y<< endl;
//  cout << setprecision(6) << "           out : " << uav1TargetVelocity.linear.x << "   "<< uav1TargetVelocity.linear.y
//                                                 << "  " << uav3TargetVelocity.linear.x << "   "<< uav3TargetVelocity.linear.y<< endl;

//  cout << setprecision(6) << "targetPosition : -------------------  " << currentPosition[2].x - y_Offset_21 * sin(currentYaw[2]) << " " << currentPosition[2].y + y_Offset_21 * cos(currentYaw[2]) << endl;
//  cout << setprecision(6) << "   yawOffset_2x : " << yawOffset_21*180/PI << "   "<< yawOffset_23*180/PI << endl;

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
//  cout << "this is uav_LRcontrol" << endl;
  if(currentOverlap_left[10] > targetOverlap_left + overlap_upper)
  {
    // 左飞
    is_imageControl[1] = true;  //  无人机1正在根据图像调整
    uav1TargetVelocity.linear.y = 0.03;
    uav1TargetVelocity.linear.z = 0;

  }
  else if(currentOverlap_left[10] < targetOverlap_left + overlap_lower)
  {
    // 右飞
    is_imageControl[1] = true;  //  无人机1正在根据图像调整
    uav1TargetVelocity.linear.y = -0.03;
    uav1TargetVelocity.linear.z = 0;
  }
  else
  {
    is_imageControl[1] = false;  //  无人机1 停止根据图像调整位置
  }

  // ****************uav3****************
  if(currentOverlap_right[10] > targetOverlap_right + overlap_upper)
  {
    // 右飞
    is_imageControl[3] = true;  //  无人机3正在根据图像调整
    uav3TargetVelocity.linear.y = -0.03;
    uav3TargetVelocity.linear.z = 0;

  }
  else if(currentOverlap_right[10] < targetOverlap_right + overlap_lower)
  {
    // 左飞
    is_imageControl[3] = true;  //  无人机3正在根据图像调整
    uav3TargetVelocity.linear.y = 0.03;
    uav3TargetVelocity.linear.z = 0;
  }
  else
  {
    is_imageControl[3] = false;  //  无人机3 停止根据图像调整位置
  }
  // 若调整距离太远，无法拼接，则向中间靠拢
  // 后期应加限制，防止撞向中间的无人机
  if(stitchingErr_left == true)
  {
    is_imageControl[1] = true;
    uav1TargetVelocity.linear.y = -0.03;
    uav1TargetVelocity.linear.z = 0;
  }
  if(stitchingErr_right == true)
  {
    is_imageControl[3] = true;
    uav3TargetVelocity.linear.y = 0.03;
    uav3TargetVelocity.linear.z = 0;
  }
}




/*
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
          start_point_lat += latitude * PI /180.0;
          start_point_lon += longitude * PI /180.0;
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

    double lat2_rad = lat2 * PI /180.0;
    double lon2_rad = lon2 * PI /180.0;
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
*/
}  // namespace image_stitching
