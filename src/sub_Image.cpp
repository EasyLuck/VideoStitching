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

#define nFeatures       500    //图像金字塔上特征点的数量
#define nLevels         8       //图像金字塔层数
#define myfScaleFactor    1.2     //金字塔比例因子
#define myfIniThFAST      20      //检测fast角点阈值
#define myfMinThFAST      8       //最低阈值


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

int imagePress(cv::Mat img1,cv::Mat img2)
{
    cv::Mat outimg;
    bool try_use_gpu = false;
    

    // std::vector<cv::KeyPoint> keypoints1;
    // std::vector<cv::KeyPoint> keypoints2;
    // cv::Mat descriptors1;
    // cv::Mat descriptors2;

    // cv::Mat images;
    // if (img1.empty() || img2.empty())
    // {
    //     cout << "Can't read image" << endl;
    //     return -1;
    // }
    // images.push_back(img1);
    // images.push_back(img2);

    // Stitcher stitcher = Stitcher::createDefault(try_use_gpu);
    
    // cv::cvtColor(img,outimg,cv::COLOR_BGR2GRAY);

    cv::imshow("ORB features",outimg);
    cv::waitKey(5);
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
    sub1 = it.subscribe("iris_1/camera_Monocular/image_raw",1,sub1Image_cb);
    sub2 = it.subscribe("iris_2/camera_Monocular/image_raw",1,sub2Image_cb);

    // sub1 = it.subscribe("/bebop4/image_raw",1,sub1Image_cb);
    // sub2 = it.subscribe("/bebop5/image_raw",1,sub2Image_cb);

    imageStitching imageStitch;
    cv::Mat newImage ;

    ros::Rate loop_rate(30);

    // ********************* 读取图片进行测试 *********************
    // std::string img1path =  "/home/linux/work/stitchingImage/3.png";
    std::string img2path =  "/home/linux/work/stitchingImage/4.png";
    cv::Mat img1;
    cv::Mat img2;
    // cv::Mat img3;
    // img3 = cv::imread(img2path,CV_LOAD_IMAGE_COLOR);
    cv::Mat img1Sacle;
    cv::Mat img2Sacle;
    // img1 = cv::imread(img1path,CV_LOAD_IMAGE_COLOR);
    // cv::imshow("test1",img1);
    // img2 = cv::imread(img2path,CV_LOAD_IMAGE_COLOR);
    // cv::imshow("test2",img2);
    // newImage = imageStitch.imageProcessLeft(img1,img2);
    // cv::imshow("imageStitching",newImage);
    // cv::waitKey(5);

    // ********************* 读取电脑摄像头 *********************
    cv::VideoCapture cap1,cap2;
    cv::startWindowThread();
    // cap1.open("/home/linux/data/dataset/3_2/left.mp4");
    // cap2.open("/home/linux/data/dataset/3_2/middle.mp4");
    cap1.open("/home/linux/data/dataset/0/left.mp4");
    cap2.open("/home/linux/data/dataset/0/right.mp4");

    while (ros::ok())
    {
        // rostopic 接受图像测试
        // if (sub1RecFlag == true && sub2RecFlag == true)
        // {
        //     sub1RecFlag = false;
        //     sub2RecFlag = false;

        //     cv::Mat leftimage = sub1RecImage;
        //     cv::Mat rightimage = sub2RecImage;
        //     cv::imshow("leftimage",leftimage);
        //     cv::imshow("rightimage",rightimage);
        //     newImage = imageStitch.imageProcess(leftimage,rightimage);
        //     cv::imshow("imageStitching",newImage);
        //     cv::waitKey(5);

        // }

        // ******************************************************************
        chrono::steady_clock::time_point t1  = chrono::steady_clock::now();
        if(cap1.isOpened())
        {
            cap1 >> img1;  
            resize(img1,img1Sacle,Size(img1.cols/2,img1.rows/2),0,0);
            // img3 = img1;
        }    
        if(cap2.isOpened())
        {
            cap2 >> img2;
            resize(img2,img2Sacle,Size(img2.cols/2,img2.rows/2),0,0);
            // img3 = img2;
        }       
        cv::imshow("imgleft",img1Sacle);
        cv::waitKey(10);
        cv::imshow("imgright",img2Sacle);
        cv::waitKey(10);

        newImage = imageStitch.stitchingThreeImage(img1,img1,img2);
        // newImage = imageStitch.stitchingThreeImage(img1,img2,img2);
        // newImage = imageStitch.imageProcess(img1,img2);
        // newImage = imageStitch.imageProcessLeft(img1,img2);
        resize(newImage,newImage,Size(newImage.cols/4,newImage.rows/4),0,0);
        cv::imshow("imageStitching",newImage);
        cv::waitKey(10);

        // chrono::steady_clock::time_point t2 = chrono::steady_clock::now();
        // chrono::duration<double> time_used = chrono::duration_cast<chrono::duration<double>>(t2-t1);
        // cout << "traditional ORB cost : " << time_used.count() << " seconds." << endl;       
        // cout << "traditional ORB cost : " << 1.0/time_used.count() << " fps." << endl;     

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
