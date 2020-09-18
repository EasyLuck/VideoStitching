#include <ros/ros.h>  
#include<sensor_msgs/image_encodings.h> //ROS图象类型的编码函数
#include<image_transport/image_transport.h> //用来在ROS系统中的话题上发布和订阅图象消息

//OpenCV2标准头文件
#include<opencv2/core/core.hpp>
#include<opencv2/highgui/highgui.hpp>
#include<opencv2/imgproc/imgproc.hpp>
#include<cv_bridge/cv_bridge.h>     //cv_bridge中包含CvBridge库

#include <opencv2/opencv.hpp>   //getOptimalNewCameraMatrix()

#include <opencv2/features2d/features2d.hpp>
#include <iostream> //C++标准输入输出库
#include <chrono>

#include <image_stitching/imageStitching.h>
#include <image_stitching/myORBextractor.h>

using namespace std;
using namespace cv;

cv::Mat sub1RecImage ;
cv::Mat sub2RecImage ;
cv::Mat sub3RecImage ;
bool sub1RecFlag = false;
bool sub2RecFlag = false;
bool sub3RecFlag = false;

//消息订阅回调函数
void sub1Image_cb(const sensor_msgs::ImageConstPtr& msg)
{
    try
    {
        sub1RecImage = cv_bridge::toCvShare(msg,"bgr8")->image;
        sub1RecFlag = true ;
    }
    catch (cv_bridge::Exception& e)
    {
        cout << "sub1Image_cb could not convert from " << msg->encoding.c_str() << "to 'brg8'." << endl;
    }   
}
void sub2Image_cb(const sensor_msgs::ImageConstPtr& msg)
{
    try
    {
        sub2RecImage = cv_bridge::toCvShare(msg,"bgr8")->image;
        sub2RecFlag = true ;
    }
    catch (cv_bridge::Exception& e)
    {
        cout << "sub2Image_cb could not convert from " << msg->encoding.c_str() << "to 'brg8'." << endl;
    }   
}
void sub3Image_cb(const sensor_msgs::ImageConstPtr& msg)
{
    try
    {
        sub3RecImage = cv_bridge::toCvShare(msg,"bgr8")->image;
        sub3RecFlag = true ;
    }
    catch (cv_bridge::Exception& e)
    {
        cout << "sub3Image_cb could not convert from " << msg->encoding.c_str() << "to 'brg8'." << endl;
    }   
}


int main(int argc, char *argv[])
{
    ros::init(argc, argv, "subscribImage");
    
    ros::NodeHandle nh;//创建句柄
    image_transport::ImageTransport it(nh);

    image_transport::Subscriber sub1;
    image_transport::Subscriber sub2;
    image_transport::Subscriber sub3;
    sub1 = it.subscribe("/bebop3/image_raw",1,sub1Image_cb);
    sub2 = it.subscribe("/bebop4/image_raw",1,sub2Image_cb);
    sub3 = it.subscribe("/bebop5/image_raw",1,sub3Image_cb);

    //设置订阅主题 camera/image
    // sub1 = it.subscribe("iris_1/camera_Monocular/image_raw",1,sub1Image_cb);
    // sub1 = it.subscribe("iris_1/camera_Monocular/image_raw",1,sub1Image_cb);
    // sub2 = it.subscribe("iris_2/camera_Monocular/image_raw",1,sub2Image_cb);

    imageStitching imageStitch;
    cv::Mat newImage ;
    cv::Mat newImageSacle;
    ros::Rate loop_rate(30);

    while (ros::ok())
    {
        // rostopic 接受图像测试
        if (sub1RecFlag == true && sub2RecFlag == true && sub3RecFlag == true)
        {
            sub1RecFlag = false;
            sub2RecFlag = false;
            sub3RecFlag = false;
            cv::Mat leftimage = sub1RecImage;
            cv::Mat middleimage = sub2RecImage;
            cv::Mat rightimage = sub3RecImage;
            cv::imshow("leftimage",leftimage);
            cv::imshow("middleimage",middleimage);
            cv::imshow("rightimage",rightimage);
            newImage = imageStitch.stitchingThreeImage(leftimage, middleimage, rightimage);
            cv::imshow("imageStitching",newImage);
            cv::waitKey(5);
        }
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
