#include <ros/ros.h>  
#include<sensor_msgs/image_encodings.h> //ROS图象类型的编码函数
#include<image_transport/image_transport.h> //用来在ROS系统中的话题上发布和订阅图象消息

//OpenCV2标准头文件
#include<opencv2/core/core.hpp>
#include<opencv2/highgui/highgui.hpp>
#include<opencv2/imgproc/imgproc.hpp>
#include<cv_bridge/cv_bridge.h>     //cv_bridge中包含CvBridge库

#include <opencv2/features2d/features2d.hpp>
#include <iostream> //C++标准输入输出库
#include <chrono>

#include <image_stitching/imageStitching.h>
#include <image_stitching/myORBextractor.h>

using namespace std;


int main(int argc, char *argv[])
{
    ros::init(argc, argv, "subscribImage");
    
    ros::NodeHandle nh;//创建句柄
    image_transport::ImageTransport it(nh);

    imageStitching imageStitch;
    cv::Mat newImage ;

    ros::Rate loop_rate(30);

    // ********************* 读取图片进行测试 *********************
    // std::string img1path =  "/home/linux/work/stitchingImage/3.png";
    std::string img2path =  "/home/linux/work/stitchingImage/4.png";
    cv::Mat img1;
    cv::Mat img2;
    cv::Mat img3;
    cv::Mat img1Sacle;
    cv::Mat img2Sacle;
    cv::Mat img3Sacle;
    // img1 = cv::imread(img1path,CV_LOAD_IMAGE_COLOR);
    // cv::imshow("test1",img1);
    // img2 = cv::imread(img2path,CV_LOAD_IMAGE_COLOR);
    // cv::imshow("test2",img2);
    // newImage = imageStitch.imageProcessLeft(img1,img2);
    // cv::imshow("imageStitching",newImage);
    // cv::waitKey(5);

    // ********************* 读取电脑摄像头 *********************
    cv::VideoCapture cap1,cap2,cap3;
    cv::startWindowThread();
    cap1.open("/home/linux/data/dataset/3_2/left.mp4");
    cap2.open("/home/linux/data/dataset/3_2/middle.mp4");
    cap3.open("/home/linux/data/dataset/3_2/right.mp4");
    // cap1.open("/home/linux/data/dataset/saveVideo/video1/bebop3.avi");
    // cap2.open("/home/linux/data/dataset/saveVideo/video1/bebop4.avi");

    while (ros::ok())
    {
        // ******************************************************************
        if(cap1.isOpened())
        {
            cap1 >> img1;  
            resize(img1,img1Sacle,Size(img1.cols/4,img1.rows/4),0,0);
        }    
        if(cap2.isOpened())
        {
            cap2 >> img2;
            resize(img2,img2Sacle,Size(img2.cols/4,img2.rows/4),0,0);
        }  
        if(cap3.isOpened())
        {
            cap3 >> img3;
            resize(img3,img3Sacle,Size(img3.cols/4,img3.rows/4),0,0);
        }       
        cv::imshow("imgleft",img1Sacle);
        cv::waitKey(1);
        cv::imshow("imgmiddle",img2Sacle);
        cv::waitKey(1);
        cv::imshow("imgright",img3Sacle);
        cv::waitKey(1);
        newImage = imageStitch.stitchingThreeImage(img1,img2,img3);
        resize(newImage,newImage,Size(newImage.cols/4,newImage.rows/4),0,0);
        cv::imshow("imageStitching",newImage);
        cv::waitKey(1);

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
