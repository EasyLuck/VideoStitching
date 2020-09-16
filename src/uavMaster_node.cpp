#include <ros/ros.h>  
#include <geometry_msgs/Point.h>
#include <nav_msgs/Odometry.h>
#include <iostream>
#include <cstdlib>
using namespace std;

class uavMaster
{
public:
    uavMaster();
    ~uavMaster();
    ros::NodeHandle nh;//创建句柄
    ros::Rate *rate;
    // ros::Publisher activityPub;

    //设置无人机的目标位置
    ros::Publisher setUav1_Position_Pub;
    ros::Publisher setUav2_Position_Pub;
    ros::Publisher setUav3_Position_Pub;
    geometry_msgs::Point setUav1_Position;
    geometry_msgs::Point setUav2_Position;
    geometry_msgs::Point setUav3_Position;

    //存储车辆位置
    geometry_msgs::Point getCar1_Position;
    geometry_msgs::Point getCar2_Position;
    geometry_msgs::Point getCar3_Position;

    //订阅获取车辆位置
    ros::Subscriber getCar1_Position_Sub;
    ros::Subscriber getCar2_Position_Sub;
    ros::Subscriber getCar3_Position_Sub;

    //订阅获取车身数字
    ros::Subscriber getCar_number_Sub;
    int car_number[3];
    bool isReceiveNumber;
    //获取数字后任务分配
    int uav_targetCar[3];
    int uav_isChange[3];

    void setUav_Position();

    void getCar1_Position_cb(const nav_msgs::Odometry::ConstPtr &msg);
    void getCar2_Position_cb(const nav_msgs::Odometry::ConstPtr &msg);
    void getCar3_Position_cb(const nav_msgs::Odometry::ConstPtr &msg);

    void getCar_number_cb(const geometry_msgs::Point::ConstPtr &msg);

    void start(void);

private:
    /* data */
};

uavMaster::~uavMaster()
{
    delete rate;
}
uavMaster::uavMaster()
{
    rate = new ros::Rate(10.0);

    isReceiveNumber = false;
    car_number[0] = 0;
    car_number[1] = 0;
    car_number[2] = 0;

    uav_targetCar[0] = 1;
    uav_targetCar[1] = 2;
    uav_targetCar[2] = 3;

    uav_isChange[0] = 0;
    uav_isChange[1] = 0;
    uav_isChange[2] = 0;

    // activity.activity = "land";
    setUav1_Position.x = 0;
    setUav1_Position.y = 0;
    setUav1_Position.z = 2.5;
    // positionPub = nh.advertise<px4_offb::setPosition>("position_info",1);
    setUav1_Position_Pub = nh.advertise<geometry_msgs::Point>("uav1/setTarget_position",1);
    setUav2_Position_Pub = nh.advertise<geometry_msgs::Point>("uav2/setTarget_position",1);
    setUav3_Position_Pub = nh.advertise<geometry_msgs::Point>("uav3/setTarget_position",1);

    getCar1_Position_Sub = nh.subscribe<nav_msgs::Odometry>("/car1/carpose", 10, &uavMaster::getCar1_Position_cb,this);
    getCar2_Position_Sub = nh.subscribe<nav_msgs::Odometry>("/car2/carpose", 10, &uavMaster::getCar2_Position_cb,this);
    getCar3_Position_Sub = nh.subscribe<nav_msgs::Odometry>("/car3/carpose", 10, &uavMaster::getCar3_Position_cb,this);

    getCar_number_Sub = nh.subscribe<geometry_msgs::Point>("/car/number", 10, &uavMaster::getCar_number_cb,this);


}
/*
 * 向三辆无人机发送目标位置
*/
void uavMaster::setUav_Position()
{

    geometry_msgs::Point car_Position[3]; 

    car_Position[0] = getCar1_Position;
    car_Position[1] = getCar2_Position;
    car_Position[2] = getCar3_Position;   

    setUav1_Position.x = car_Position[uav_targetCar[0]-1].x - 0;
    setUav1_Position.y = car_Position[uav_targetCar[0]-1].y - 2.5;
    setUav1_Position.z = 4;

    setUav2_Position.x = car_Position[uav_targetCar[1]-1].x;
    setUav2_Position.y = car_Position[uav_targetCar[1]-1].y - 3.5;
    setUav2_Position.z = 3;

    setUav3_Position.x = car_Position[uav_targetCar[2]-1].x + 0;
    setUav3_Position.y = car_Position[uav_targetCar[2]-1].y - 4.5;
    setUav3_Position.z = 2;

    // 测试使用
    // setUav1_Position.x = getCar1_Position.x - 1;
    // setUav1_Position.y = getCar1_Position.y - 2;
    // setUav1_Position.z = 4;

    // setUav2_Position.x = getCar1_Position.x;
    // setUav2_Position.y = getCar1_Position.y - 2;
    // setUav2_Position.z = 3;

    // setUav3_Position.x = getCar1_Position.x + 1;
    // setUav3_Position.y = getCar1_Position.y - 2;
    // setUav3_Position.z = 2;

    setUav1_Position_Pub.publish(setUav1_Position);
    setUav2_Position_Pub.publish(setUav2_Position);
    setUav3_Position_Pub.publish(setUav3_Position);

    cout << "car_number: " << car_number[0] << " " << car_number[1] << " " << car_number[2] << endl;
    cout << "setUav_Position: " << setUav1_Position.x << " " << setUav2_Position.x << " " << setUav3_Position.x << endl;
}
void uavMaster::getCar1_Position_cb(const nav_msgs::Odometry::ConstPtr &msg)
{
    getCar1_Position.x = msg->pose.pose.position.x;
    getCar1_Position.y = msg->pose.pose.position.y;
    getCar1_Position.z = msg->pose.pose.position.z;
}
void uavMaster::getCar2_Position_cb(const nav_msgs::Odometry::ConstPtr &msg)
{
    getCar2_Position.x = msg->pose.pose.position.x;
    getCar2_Position.y = msg->pose.pose.position.y;
    getCar2_Position.z = msg->pose.pose.position.z;
}
void uavMaster::getCar3_Position_cb(const nav_msgs::Odometry::ConstPtr &msg)
{
    getCar3_Position.x = msg->pose.pose.position.x;
    getCar3_Position.y = msg->pose.pose.position.y;
    getCar3_Position.z = msg->pose.pose.position.z;
}

void uavMaster::getCar_number_cb(const geometry_msgs::Point::ConstPtr &msg)
{
    car_number[0] = (int)msg->x;
    car_number[1] = (int)msg->y;
    car_number[2] = (int)msg->z;
    isReceiveNumber = true;
}
// void uavMaster::pubActivity(void)
// {
//     activityPub.publish(activity);
// }


void uavMaster::start(void)
{
    ros::Time last_request = ros::Time::now();
    while(ros::ok())
    {
        if(ros::Time::now() - last_request > ros::Duration(2.0))
        {
            std::cout << " uavMaster_node is running !" << std::endl;
            last_request = ros::Time::now();
        }
        setUav_Position();
        ros::spinOnce();
        rate->sleep();
    }
}
int main(int argc, char *argv[])
{
    /* code for main function */
    ros::init(argc, argv, "uavMaster_node");

    uavMaster posContral;
    posContral.start();

    return 0;
}