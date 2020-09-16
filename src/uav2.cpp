#include <image_stitching/uav_control.h>
#include <iostream>
using namespace std;

uav2Node::uav2Node()
{
    offb_set_mode.request.custom_mode = "OFFBOARD";
    
    island = false;

    tagetPose.position.x = 3;
    tagetPose.position.y = 0;
    tagetPose.position.z = 3;

    setPosition.pose.position = tagetPose.position;
    /*************************初始位置偏差*******************************/
    //uav2 初始位置为（3 0 0） --> 无人机位置参考原点在（3 0 0）(仿真中的设置有关，现实中没有偏差)
    uav_offset.x = 3;
    uav_offset.y = 0;
    uav_offset.z = 0;

    /*************************PID参数*******************************/
    pid_x.p = 2.8;
    pid_x.i = 0;
    pid_x.d = 1.8;
    pid_x.err_last = 0;

    pid_y.p = 2.8;
    pid_y.i = 0;
    pid_y.d = 1.8;
    pid_y.err_last = 0;

    pid_z.p = 2.0;
    pid_z.i = 0;
    pid_z.d = 0.8;
    pid_z.err_last = 0;

    rate = new ros::Rate(10.0);
    /*详细参考 http://wiki.ros.org/mavros#Utility_commands */
    /*FCU state*/
    state_sub = nh.subscribe<mavros_msgs::State>("uav2/mavros/state", 10, &uav2Node::state_cb,this);
      
    /*Change Arming status. */
    arming_client = nh.serviceClient<mavros_msgs::CommandBool>("uav2/mavros/cmd/arming");
    /*Set FCU operation mode*/
    set_mode_client = nh.serviceClient<mavros_msgs::SetMode>("uav2/mavros/set_mode");
    /*Local position from FCU. NED坐标系(惯性系)*/ 
    // currentPose_sub = nh.subscribe<geometry_msgs::PoseStamped>("uav2/mavros/local_position/pose", 10,&uav2Node::currentPose_cb,this); 
    currentPose_sub = nh.subscribe<nav_msgs::Odometry>("uav2/gazeboPose", 10,&uav2Node::currentGazeboPose_cb,this); 

    /*订阅位置设置消息*/
    tagetPosition_sub = nh.subscribe<geometry_msgs::Point>("uav2/setTarget_position", 10, &uav2Node::tagetPosition_cb,this);

    currentVelocity_sub = nh.subscribe<geometry_msgs::TwistStamped>("uav2/mavros/local_position/velocity_local", 10, &uav2Node::currentVelocity_cb,this);

    setVelocity_pub =  nh.advertise<geometry_msgs::TwistStamped>("uav2/mavros/setpoint_velocity/cmd_vel",1);
    
    /*Local frame setpoint position. NED坐标系(惯性系)*/  
    setPosition_pub = nh.advertise<geometry_msgs::PoseStamped>("uav2/mavros/setpoint_position/local",10);

    
}


void uav2Node::state_cb(const mavros_msgs::State::ConstPtr& msg)
{
    current_state = *msg;
}
void uav2Node::currentVelocity_cb(const geometry_msgs::TwistStamped::ConstPtr &msg)
{
    // cout << "currentVelocity: " << msg->twist.linear.x << "  " << msg->twist.linear.y << "  " << msg->twist.linear.z << endl;
    currentVelocity.x = msg->twist.linear.x;
    currentVelocity.y = msg->twist.linear.y;
    currentVelocity.z = msg->twist.linear.z;
}
void uav2Node::currentPose_cb(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    //得到真实坐标 无人机位置参考原点在（0 -3 0）
    currentPose.position.x = msg->pose.position.x + uav_offset.x;
    currentPose.position.y = msg->pose.position.y + uav_offset.y;
    currentPose.position.z = msg->pose.position.z + uav_offset.z;
    currentPose.orientation = msg->pose.orientation; 
}
void uav2Node::currentGazeboPose_cb(const nav_msgs::Odometry::ConstPtr& msg)
{
    currentPose = msg->pose.pose;
}
/*处理收到的位置设置消息*/
void uav2Node::tagetPosition_cb(const geometry_msgs::Point::ConstPtr &msg)
{
    // cout << "target position: " << msg->x << "  "<< msg->y << "  " << msg->z << endl;
    tagetPose.position = *msg;
    cout << "target position: " << tagetPose.position.x << "  "<< tagetPose.position.y << "  " << tagetPose.position.z << endl;
}
// void uav2Node::setActivity_cb(const px4_offb::setActivity::ConstPtr &msg)
// {
//     ROS_INFO("Received Custom Activity:%s",msg->activity.c_str());
//     if (strcmp(msg->activity.c_str(),"land") ==0)
//     {
//         ROS_INFO("px4 is landing");
//         island = true;
//         pose.pose.position.z = 0.1;
//         arm_cmd.request.value = false;
//     }
// }

void uav2Node::start()
{
    //方便调参数
    double param_kp_x, param_ki_x, param_kp_y, param_ki_y, param_kp_z, param_ki_z;
    ros::Time last_request = ros::Time::now();

    if(!this->offboard())
    {
        while (ros::ok())
        {
            ROS_INFO("Offboard enabled ! Vehicle armed failed ! ");
            ros::spinOnce();
            rate->sleep();
        }
    }
    //等待到达安全高度
    while(ros::ok())
    {
        if(currentPose.position.z >= 2)
            break;
        setPosition_pub.publish(setPosition);
        ros::spinOnce();
        rate->sleep();
    }

    while(ros::ok())
    {
        // if(ros::Time::now() - last_request > ros::Duration(2.0))
        // {
        //     //方便调试参数 调试完成可以去掉
        //     nh.param("kp_x", param_kp_x, 1.0);
        //     nh.param("ki_x", param_ki_x, 1.0);
        //     nh.param("kp_y", param_kp_y, 1.0);
        //     nh.param("ki_y", param_ki_y, 1.0);
        //     nh.param("kp_z", param_kp_z, 1.0);
        //     nh.param("ki_z", param_ki_z, 1.0);

        //     pid_x.p = param_kp_x;
        //     pid_x.i = param_ki_x;
        //     pid_y.p = param_kp_y;
        //     pid_y.i = param_ki_y;
        //     pid_z.p = param_kp_z;
        //     pid_z.i = param_ki_z;


        //     last_request = ros::Time::now();
        // }
            // cout << "pid_x: " << pid_x.p << " " << pid_x.i << endl;
            // cout << "pid_y: " << pid_y.p << " " << pid_y.i << endl;        
            // cout << "pid_z: " << pid_z.p << " " << pid_z.i << endl;    
        
        //速度PID控制
        pidVelocityControl();

        // 位置控制
        // positionControl();


        //判断是否满足降落条件
        if (island == true && currentPose.position.z<=0.15)
        {    
            if( arming_client.call(arm_cmd) && arm_cmd.response.success)
            {
                ROS_INFO("Vehicle diarmed");
                break;
            }
        }

        ros::spinOnce();
        rate->sleep();
    }

}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "uav2_node");

    uav2Node uav2;
    uav2.start();

    return 0;
}