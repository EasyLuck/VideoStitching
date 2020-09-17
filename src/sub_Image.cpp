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

    //内参矩阵
    double camera_matrix_data[3][3] = {531.942070, 0.000000, 429.943241, 0.000000, 522.386406, 236.978245, 0.000000, 0.000000, 1.000000};
    Mat cameraMatrix = Mat(3, 3, CV_64F, camera_matrix_data);
    // cameraMatrix.at<double>(0, 0) = 535.541655;
    // cameraMatrix.at<double>(0, 1) = 0;
    // cameraMatrix.at<double>(0, 2) = 439.725394;
    // cameraMatrix.at<double>(1, 0) = 0;
    // cameraMatrix.at<double>(1, 1) = 522.708593;
    // cameraMatrix.at<double>(1, 2) = 246.191841;

    //畸变参数
    double distCoeffs_data[5][1] = {-0.001109, -0.004412, -0.003832, 0.000871, 0.000000};
    cv::Mat distCoeffs = Mat(5, 1, CV_64F, distCoeffs_data);
    // distCoeffs.at<double>(0, 0) = 0.003443;
    // distCoeffs.at<double>(1, 0) = 0.013320;
    // distCoeffs.at<double>(2, 0) = 0.002249;
    // distCoeffs.at<double>(3, 0) = 0.004472;
    // distCoeffs.at<double>(4, 0) = 0;


cv::Mat sub1RecImage ;
cv::Mat sub2RecImage ;
bool sub1RecFlag = false;
bool sub2RecFlag = false;

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



int main(int argc, char *argv[])
{
    ros::init(argc, argv, "subscribImage");
    
    ros::NodeHandle nh;//创建句柄
    image_transport::ImageTransport it(nh);

    image_transport::Subscriber sub1;
    image_transport::Subscriber sub2;

    //设置订阅主题 camera/image
    // sub1 = it.subscribe("iris_1/camera_Monocular/image_raw",1,sub1Image_cb);
    // sub1 = it.subscribe("iris_1/camera_Monocular/image_raw",1,sub1Image_cb);
    // sub2 = it.subscribe("iris_2/camera_Monocular/image_raw",1,sub2Image_cb);

    sub1 = it.subscribe("/bebop3/image_raw",1,sub1Image_cb);
    sub2 = it.subscribe("/bebop4/image_raw",1,sub2Image_cb);

    imageStitching imageStitch;
    cv::Mat newImage ;
    cv::Mat newImageSacle;
    ros::Rate loop_rate(30);

    cout << "cameraMatrix:" << endl;
    cout << cameraMatrix << endl;
    cout << "distCoeffs:" << endl;
    cout << distCoeffs << endl;

    while (ros::ok())
    {
        // rostopic 接受图像测试
        if (sub1RecFlag == true )
        {
            sub1RecFlag = false;
            sub2RecFlag = false;

            cv::Mat leftimage = sub1RecImage;
            cv::Mat rightimage = sub2RecImage;
            // resize(img2,img2Sacle,Size(img2.cols/2,img2.rows/2),0,0);
            cv::imshow("leftimage",leftimage);
            cv::imshow("rightimage",rightimage);
            // cv::Mat  remapleftimage, map1, map2;
            // cv::Size imageSize;
            // imageSize = leftimage.size();
            // cv::Mat new_K = cv::getOptimalNewCameraMatrix(cameraMatrix, distCoeffs, imageSize, 1, imageSize, 0);
            // cv::initUndistortRectifyMap(cameraMatrix, distCoeffs, Mat(),new_K,imageSize, CV_16SC2, map1, map2);
            // cv::remap(leftimage, remapleftimage, map1, map2, INTER_LINEAR);
            // cv::imshow("remapleftimage",remapleftimage);    
            


            newImage = imageStitch.stitchingThreeImage(leftimage,leftimage,rightimage);
            resize(newImage,newImageSacle,Size(newImage.cols/2,newImage.rows/2),0,0);
            cv::imshow("imageStitching",newImageSacle);
            cv::waitKey(5);

        }
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
