#include "../include/image_stitching/uav.hpp"

using namespace std;
uav::uav()
{
  receiveImageFlag = false;
  forward = false;
  backward = false;
  flayLeft = false;
  flayRight = false;
  flayUp = false;
  flayDown = false;
  turnLeft = false;
  turnRight = false;

  batteryData = 0; //

//  isRunning = false;
}

void uav::takeoff()
{
    takeoff_pub.publish(std_msgs::Empty());
    cout << "bebop  takeoff! " << endl;
//    ROS_INFO("bebop  takeoff! ");
}
void uav::land()
{
    land_pub.publish(std_msgs::Empty());
    cout << "bebop  land! " << endl;
//    ROS_INFO("bebop  land! ");
}
void uav::cmd(double x,double y,double z,double yaw)
{
    cmd_vel.linear.x = x;
    cmd_vel.linear.y = y;
    cmd_vel.linear.z = z;
    cmd_vel.angular.z = yaw;
    cmd_pub.publish(cmd_vel);
    cout << "bebop  send cmd! " << endl;
//    ROS_INFO("bebop  send cmd! ");
}

void uav::cameraControl(double vertical, double horizontal)
{
  cameraControl_vel.angular.y = vertical;
  cameraControl_vel.angular.z = horizontal;
  cameraControl_pub.publish(cameraControl_vel);
}

void uav::moveControl()
{
  if(forward == true)   //前进
    cmd(0.15, 0, 0, 0);
  if(backward == true)  //后退
    cmd(-0.15, 0, 0, 0);
  if(flayLeft == true)  //左飞
    cmd(0, 0.15, 0, 0);
  if(flayRight == true) //右飞
    cmd(0, -0.15, 0, 0);
  if(flayUp == true)    //上升
    cmd(0, 0, 0.15, 0);
  if(flayDown == true)  //下降
    cmd(0, 0, -0.15, 0);
  if(turnLeft == true)  //左转
    cmd(0, 0, 0, 0.1);
  if(turnRight == true) //右转
    cmd(0, 0, 0, -0.1);

  if(!forward && !backward && !flayLeft && !flayRight && !flayUp && !flayDown && !turnLeft && !turnRight )
    cmd(0, 0, 0, 0);
}
