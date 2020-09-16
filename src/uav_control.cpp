/**
 * @file offb_node.cpp
 * @brief offboard example node, written with mavros version 0.14.2, px4 flight
 * stack and tested in Gazebo SITL
 */

#include <ros/ros.h>
#include <iostream>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/TwistStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <image_stitching/uav_control.h>
using namespace std;



uavControl::~uavControl()
{
    delete rate;
}
/* 构造函数 初始化参数 */
uavControl::uavControl()
{
    offb_set_mode.request.custom_mode = "OFFBOARD";
    
    island = false;


    /*************************幅值限制*******************************/
    maxVelocity_x = 10.0;
    maxVelocity_y = 10.0;
    maxVelocity_z = 10.0;


    /*************************目标速度*******************************/
    setVelocity.twist.linear.x = 0;
    setVelocity.twist.linear.y = 0;
    setVelocity.twist.linear.z = 0;
    
}


void uavControl::waitConnect()
{
    ROS_INFO("wait for FCU connection...");
    while(ros::ok() && current_state.connected)
    {
        ros::spinOnce();
        rate->sleep();
    }
    ROS_INFO("FCU connection ok!");
}
//解锁
bool uavControl::arm()
{
    arm_cmd.request.value = true;
    if( arming_client.call(arm_cmd) &&
        arm_cmd.response.success)
    {
        return true;
    } 
    else
        return false;
}
//上锁
bool uavControl::disarm()
{
    arm_cmd.request.value = false;
    if( arming_client.call(arm_cmd) &&
        arm_cmd.response.success)
    {
        return true;
    } 
    else
        return false;
}
// 设置OFFBOARD 模式
bool uavControl::offboard()
{
    int i = 100;
    for(i = 100; ros::ok() && i > 0; --i)
    {
        setPosition_pub.publish(setPosition);
        /* 当前不是 OFFBOARD 模式 */
        if( current_state.mode != "OFFBOARD")
        {
            /*设置 OFFBOARD 模式*/
            if( set_mode_client.call(offb_set_mode) &&
                offb_set_mode.response.mode_sent)
            {
                ROS_INFO("Offboard enabled");
            }
        } 
        else 
        {
            /* 当前未解锁 */
            if( !current_state.armed)
            {
                if(this->arm())
                {
                    ROS_INFO("Vehicle armed");    
                }
            }
        }
        /*解锁成功提前退出*/
        if(current_state.mode == "OFFBOARD" && current_state.armed)
            return true;
        ros::spinOnce();
        rate->sleep();
    }
    return false;
}

float uavControl::satfunc(float data, float Max)
{

    if(abs(data)>Max)
        return (data>0)?Max:-Max;
    else
        return data;
}
void uavControl::pidVelocityControl()
{

    pid_x.err = tagetPose.position.x - currentPose.position.x;
    pid_y.err = tagetPose.position.y - currentPose.position.y;
    pid_z.err = tagetPose.position.z - currentPose.position.z;

    // setVelocity.twist.linear.x = pid_x.p * pid_x.err + pid_x.i * pid_x.err_last;
    // setVelocity.twist.linear.y = pid_y.p * pid_y.err + pid_y.i * pid_y.err_last;
    // setVelocity.twist.linear.z = pid_z.p * pid_z.err + pid_z.i * pid_z.err_last;

    setVelocity.twist.linear.x = pid_x.p * pid_x.err + pid_x.d * (0 - currentVelocity.x);
    setVelocity.twist.linear.y = pid_y.p * pid_y.err + pid_y.d * (0 - currentVelocity.y);
    setVelocity.twist.linear.z = pid_z.p * pid_z.err + pid_z.d * (0 - currentVelocity.z);
    

    // pid_x.err_last = pid_x.err;
    // pid_y.err_last = pid_y.err;
    // pid_z.err_last = pid_z.err;

    //限制幅值
    setVelocity.twist.linear.x = satfunc(setVelocity.twist.linear.x , maxVelocity_x);
    setVelocity.twist.linear.y = satfunc(setVelocity.twist.linear.y , maxVelocity_y);
    setVelocity.twist.linear.z = satfunc(setVelocity.twist.linear.z , maxVelocity_z);

    setVelocity_pub.publish(setVelocity);
    cout << "err_x: " << pid_x.err << endl;
    cout << "err_y: " << pid_y.err << endl;
    cout << "Velocity_x: " << setVelocity.twist.linear.x << endl;
    cout << "Velocity_y: " << setVelocity.twist.linear.y << endl;
}

void uavControl::positionControl()
{
    //转换为仿真世界参考坐标系 无人机位置参考原点在（0 -3 0）
    //只有在仿真环境中用得到，真实测试不需要    
    setPosition.pose.position.x = tagetPose.position.x - uav_offset.x;
    setPosition.pose.position.y = tagetPose.position.y - uav_offset.y;
    setPosition.pose.position.z = tagetPose.position.z - uav_offset.z;

    // setPosition.pose.position = tagetPose.position;

    setPosition_pub.publish(setPosition);    
}