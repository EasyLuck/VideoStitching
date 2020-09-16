#include <opencv2/highgui/highgui.hpp>  
// #include <opencv2/nonfree/nonfree.hpp>
// #include <opencv2/legacy/legacy.hpp>   
// #include "opencv2/xfeatures2d/nonfree.hpp"
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/opencv_modules.hpp>
#include "opencv2/calib3d/calib3d.hpp"
#include <iostream>  
#include <chrono>

#include <opencv2/stitching.hpp>

#include <image_stitching/imageStitching.h>
#include <image_stitching/myORBextractor.h>

using namespace cv;
using namespace std;

#define nFeatures       500    //图像金字塔上特征点的数量
#define nLevels         8       //图像金字塔层数
#define myfScaleFactor    1.2     //金字塔比例因子
#define myfIniThFAST      20      //检测fast角点阈值
#define myfMinThFAST      8       //最低阈值

imageStitching::imageStitching()
{

}

void imageStitching::CalcCorners( Mat H,  Mat src)
{
    double v2[] = { 0, 0, 1 };//左上角
    double v1[3];//变换后的坐标值
    Mat V2 = Mat(3, 1, CV_64FC1, v2);  //列向量
    Mat V1 = Mat(3, 1, CV_64FC1, v1);  //列向量

    V1 = H * V2;

    corners.left_top.x = v1[0] / v1[2];
    corners.left_top.y = v1[1] / v1[2];

    //左下角(0,src.rows,1)
    v2[0] = 0;
    v2[1] = src.rows;
    v2[2] = 1;
    V2 = Mat(3, 1, CV_64FC1, v2);  //列向量
    V1 = Mat(3, 1, CV_64FC1, v1);  //列向量
    V1 = H * V2;
    corners.left_bottom.x = v1[0] / v1[2];
    corners.left_bottom.y = v1[1] / v1[2];

    //右上角(src.cols,0,1)
    v2[0] = src.cols;
    v2[1] = 0;
    v2[2] = 1;
    V2 = Mat(3, 1, CV_64FC1, v2);  //列向量
    V1 = Mat(3, 1, CV_64FC1, v1);  //列向量
    V1 = H * V2;
    corners.right_top.x = v1[0] / v1[2];
    corners.right_top.y = v1[1] / v1[2];

    //右下角(src.cols,src.rows,1)
    v2[0] = src.cols;
    v2[1] = src.rows;
    v2[2] = 1;
    V2 = Mat(3, 1, CV_64FC1, v2);  //列向量
    V1 = Mat(3, 1, CV_64FC1, v1);  //列向量
    V1 = H * V2;
    corners.right_bottom.x = v1[0] / v1[2];
    corners.right_bottom.y = v1[1] / v1[2];

}

cv::Mat imageStitching::stitchingThreeImage(Mat img1, Mat img2, Mat img3)
{
    //灰度图转换  
    Mat grayImage1, grayImage2, grayImage3;
    cvtColor(img1, grayImage1, CV_RGB2GRAY);
    cvtColor(img2, grayImage2, CV_RGB2GRAY);
    cvtColor(img3, grayImage3, CV_RGB2GRAY);

    Mat imageDesc1, imageDesc2,imageDesc3;
    vector<KeyPoint> keyPoint1, keyPoint2,keyPoint3;

    myORB::ORBextractor ORBextractor =  myORB::ORBextractor(nFeatures,myfScaleFactor,nLevels,myfIniThFAST,myfMinThFAST);
    ORBextractor(grayImage1, cv::Mat(), keyPoint1, imageDesc1);
    ORBextractor(grayImage2, cv::Mat(), keyPoint2, imageDesc2);
    ORBextractor(grayImage3, cv::Mat(), keyPoint3, imageDesc3);

    BFMatcher matcher(NORM_HAMMING);
    // matches_LM --> 左图和中图   matches_MR --> 中图和右图
    vector< vector<DMatch> > matches_LM,matches_MR;
    vector<DMatch> good_matches_LM,good_matches_MR;

    // 会为每个目标特征点返回两个最匹配的两个特征点
    matcher.knnMatch(imageDesc1,imageDesc2,matches_LM,2);
    matcher.knnMatch(imageDesc2,imageDesc3,matches_MR,2);
    // 筛选
    for (int i = 0; i < matches_LM.size(); i++)
    {
        // 比较欧氏距离差距    
        if(matches_LM[i][0].distance < 0.6 * matches_LM[i][1].distance)
            good_matches_LM.push_back(matches_LM[i][0]);
    }
    for (int i = 0; i < matches_MR.size(); i++)
    {
        // 比较欧氏距离差距    
        if(matches_MR[i][0].distance < 0.6 * matches_MR[i][1].distance)
            good_matches_MR.push_back(matches_MR[i][0]);
    }

    try{

    Mat homo_LM,homo_MR;
    Mat dst(img2.rows*2,img2.cols*3, CV_8UC3);
    dst.setTo(0);
    Mat  rightImageTransform;

    Point2f offfset;
    offfset.y = img2.rows/2;
    offfset.x = img2.cols;
    bool leftTransform_ok = false,rightTransform_ok = false;

    // if(good_matches_LM.size()>=8)
    // {
    //     vector<Point2f> imagePoints1, imagePoints2;
    //     for (int i = 0; i<good_matches_LM.size(); i++)
    //     {
    //         imagePoints1.push_back(keyPoint1[good_matches_LM[i].queryIdx].pt);  
    //         imagePoints2.push_back(keyPoint2[good_matches_LM[i].trainIdx].pt + offfset);
    //     }

    //     //获取图像2到图像1的投影映射矩阵 尺寸为3*3  
    //     homo_LM = findHomography(imagePoints1, imagePoints2, CV_RANSAC);
    //     //计算配准图的四个顶点坐标
    //     CalcCorners(homo_LM, img1);
    //     //图像配准  
    //     Mat  leftImageTransform;
    //     warpPerspective(img1, leftImageTransform, homo_LM,Size(MAX(corners.right_top.x,corners.right_bottom.x),MAX(corners.left_bottom.y,corners.right_bottom.y)));
    //     leftImageTransform.copyTo(dst(Rect(0, 0, leftImageTransform.cols, leftImageTransform.rows)));
    //     img2.copyTo(dst(Rect(offfset.x, offfset.y, img2.cols, img2.rows)));
    //     OptimizeSeamleft(img2, leftImageTransform, dst,offfset);
        // leftTransform_ok = true;
    // }
    // else//匹配点不足８个
    // {
    //     cout << " Insufficient feature points, unable to realize left image stitching " << endl;
    // }

    if(good_matches_MR.size()>=8)
    {
        vector<Point2f> imagePoints1, imagePoints2;
        for (int i = 0; i<good_matches_MR.size(); i++)
        {
            imagePoints1.push_back(keyPoint2[good_matches_MR[i].queryIdx].pt + offfset);  
            imagePoints2.push_back(keyPoint3[good_matches_MR[i].trainIdx].pt);
        }

        //获取图像2到图像1的投影映射矩阵 尺寸为3*3  
        homo_MR = findHomography(imagePoints2, imagePoints1, CV_RANSAC);
        //计算配准图的四个顶点坐标
        CalcCorners(homo_MR, img3);
        //图像配准  
        
        warpPerspective(img3, rightImageTransform, homo_MR, Size(MAX(corners.right_top.x,corners.right_bottom.x),MAX(corners.left_bottom.y,corners.right_bottom.y)));
        rightImageTransform.copyTo(dst(Rect(0, 0, rightImageTransform.cols, rightImageTransform.rows)));       
        rightTransform_ok = true;
    }
    else//匹配点不足８个
    {
        cout << " Insufficient feature points, unable to realize right image stitching " << endl;
    }

    img2.copyTo(dst(Rect(offfset.x, offfset.y, img2.cols, img2.rows)));
    cout << "1111111"<<endl;
    if(leftTransform_ok == true)
    {
        leftTransform_ok = false;
        // OptimizeSeamleft(img2, leftImageTransform, dst,offfset);
    }    
    if(rightTransform_ok == true)
    {
        rightTransform_ok = false;
        cout << "222222"<<endl;
        OptimizeSeamRight(img2, rightImageTransform, dst,offfset);
        cout << "333333"<<endl;
    }    

    return dst;

    }
    catch(...){
        cout << "error!!!!" << endl;
        return img1;
    }

}



//优化两图的连接处，使得拼接自然
void imageStitching::OptimizeSeamRight(Mat& leftImage, Mat& trans, Mat& dst, Point2f offfset)
{
    int start = MIN(corners.left_top.x, corners.left_bottom.x);//开始位置，即重叠区域的左边界  
    if(start < offfset.x)
        start = offfset.x;
    if(start < leftImage.cols  + offfset.x)
    {
    double processWidth = leftImage.cols + offfset.x - start;//重叠区域的宽度  
    cout << "processWidth " << processWidth << endl; 
    int top = offfset.y;
    int bottom = leftImage.rows + offfset.y;
    int cols = leftImage.cols + offfset.x; 
    double alpha = 1;//img1中像素的权重  

    for (int i = top; i < bottom; i++)
    {
        uchar* p = leftImage.ptr<uchar>(i - top);  //获取第i行的首地址
        uchar* t = trans.ptr<uchar>(i);
        uchar* d = dst.ptr<uchar>(i);
        for (int j = start; j < cols; j++)
        {
            //如果遇到图像trans中无像素的黑点，则完全拷贝img1中的数据
            if (t[j * 3] == 0 && t[j * 3 + 1] == 0 && t[j * 3 + 2] == 0)
            {
                alpha = 1;
            }
            else
            {
                //img1中像素的权重，与当前处理点距重叠区域左边界的距离成正比，实验证明，这种方法确实好  
                alpha = (processWidth - (j - start)) / processWidth;
            }

            d[j * 3] = p[j * 3] * alpha + t[j * 3] * (1 - alpha);
            d[j * 3 + 1] = p[j * 3 + 1] * alpha + t[j * 3 + 1] * (1 - alpha);
            d[j * 3 + 2] = p[j * 3 + 2] * alpha + t[j * 3 + 2] * (1 - alpha);

        }
    }
    }

}

//优化两图的连接处，使得拼接自然
void imageStitching::OptimizeSeamleft(Mat& rightImage, Mat& trans, Mat& dst,Point2f offfset)
{
    // offfset.x -- cols
    //开始位置，即重叠区域的右边界
    int start = MAX(corners.right_top.x, corners.right_bottom.x);  
    if(start > rightImage.cols + offfset.x)
        start = rightImage.cols+ offfset.x;

    if(start > offfset.x)
    {
    double processWidth = start - rightImage.cols;//重叠区域的宽度 
    int top = offfset.y;
    int bottom = rightImage.rows + offfset.y;
    int cols = offfset.x; 
    double alpha = 1;//img1中像素的权重  
    for (int i = top; i < bottom; i++)
    {
        uchar* p = rightImage.ptr<uchar>(i - top);  //获取第i行的首地址
        uchar* t = trans.ptr<uchar>(i);
        uchar* d = dst.ptr<uchar>(i);
        for (int j = start; j > cols; j--)
        {
            //如果遇到图像trans中无像素的黑点，则完全拷贝img1中的数据
            if (t[j * 3] == 0 && t[j * 3 + 1] == 0 && t[j * 3 + 2] == 0)
            {
                alpha = 1;
            }
            else
            {
                //img1中像素的权重，与当前处理点距重叠区域左边界的距离成正比，实验证明，这种方法确实好  
                alpha = (processWidth - (start - j)) / processWidth;
            }

            d[j * 3] = p[j * 3 - 0] * alpha + t[j * 3] * (1 - alpha);
            d[j * 3 + 1] = p[j * 3 - 0 + 1] * alpha + t[j * 3 + 1] * (1 - alpha);
            d[j * 3 + 2] = p[j * 3 - 0 + 2] * alpha + t[j * 3 + 2] * (1 - alpha);

        }
    }
    }

}

void imageStitching::drawText(Mat & image)
{
    putText(image, "Insufficient feature points, unable to realize image stitching",
            Point(0, image.rows/2),
            FONT_HERSHEY_COMPLEX, 1, // font face and scale
            Scalar(0, 0, 255), // red (bgr) 
            1, LINE_AA); // line thickness and type
}



// cv::Mat imageStitching::stitchingThreeImage(Mat img1, Mat img2, Mat img3)
// {
//     //灰度图转换  
//     Mat grayImage1, grayImage2, grayImage3;
//     cvtColor(img1, grayImage1, CV_RGB2GRAY);
//     cvtColor(img2, grayImage2, CV_RGB2GRAY);
//     cvtColor(img3, grayImage3, CV_RGB2GRAY);

//     Mat imageDesc1, imageDesc2,imageDesc3;
//     vector<KeyPoint> keyPoint1, keyPoint2,keyPoint3;

//     myORB::ORBextractor ORBextractor =  myORB::ORBextractor(nFeatures,myfScaleFactor,nLevels,myfIniThFAST,myfMinThFAST);
//     ORBextractor(grayImage1, cv::Mat(), keyPoint1, imageDesc1);
//     ORBextractor(grayImage2, cv::Mat(), keyPoint2, imageDesc2);
//     ORBextractor(grayImage3, cv::Mat(), keyPoint3, imageDesc3);

//     BFMatcher matcher(NORM_HAMMING);
//     // matches_LM --> 左图和中图   matches_MR --> 中图和右图
//     vector< vector<DMatch> > matches_LM,matches_MR;
//     vector<DMatch> good_matches_LM,good_matches_MR;

//     // 会为每个目标特征点返回两个最匹配的两个特征点
//     matcher.knnMatch(imageDesc1,imageDesc2,matches_LM,2);
//     matcher.knnMatch(imageDesc2,imageDesc3,matches_MR,2);
//     // 筛选
//     for (int i = 0; i < matches_LM.size(); i++)
//     {
//         // 比较欧氏距离差距    
//         if(matches_LM[i][0].distance < 0.6 * matches_LM[i][1].distance)
//             good_matches_LM.push_back(matches_LM[i][0]);
//     }
//     for (int i = 0; i < matches_MR.size(); i++)
//     {
//         // 比较欧氏距离差距    
//         if(matches_MR[i][0].distance < 0.6 * matches_MR[i][1].distance)
//             good_matches_MR.push_back(matches_MR[i][0]);
//     }

//     try{

//     Mat homo_LM,homo_MR;
//     Mat dst(img2.rows*2,img2.cols*3, CV_8UC3);
//     dst.setTo(0);

//     Point2f offfset;
//     offfset.y = img2.rows/2;
//     offfset.x = img2.cols;

//     if(good_matches_LM.size()>=8)
//     {
//         vector<Point2f> imagePoints1, imagePoints2;
//         for (int i = 0; i<good_matches_LM.size(); i++)
//         {
//             imagePoints1.push_back(keyPoint1[good_matches_LM[i].queryIdx].pt);  
//             imagePoints2.push_back(keyPoint2[good_matches_LM[i].trainIdx].pt + offfset);
//         }

//         //获取图像2到图像1的投影映射矩阵 尺寸为3*3  
//         homo_LM = findHomography(imagePoints1, imagePoints2, CV_RANSAC);
//         //计算配准图的四个顶点坐标
//         CalcCorners(homo_LM, img1);
//         //图像配准  
//         Mat  leftImageTransform;
//         warpPerspective(img1, leftImageTransform, homo_LM, Size(dst.cols, dst.rows));
//         // cv::imshow("leftImageTransform",leftImageTransform);
//         img2.copyTo(dst(Rect(offfset.x, offfset.y, img2.cols, img2.rows)));
//         leftImageTransform.copyTo(dst(Rect(0, 0, leftImageTransform.cols, leftImageTransform.rows)));
        
//         // img2ReSize.copyTo(dst(Rect(img1.cols, 0, img2.cols, img2.rows)));
//         // OptimizeSeamleft(img2, leftImageTransform, dst,offfset);

//     }
//     else//匹配点不足８个
//     {
//         // img2ReSize.copyTo(dst(Rect(0, 0, img2ReSize.cols, img2ReSize.rows)));
//         img2.copyTo(dst(Rect(img2.cols, 0, img2.cols, img2.rows)));
//         // img1.copyTo(dst(Rect(0, 0, img2.cols, img2.rows)));
//         drawText(dst);
//         cout << " Insufficient feature points, unable to realize left image stitching " << endl;
//     }

//     // if(good_matches_MR.size()>=8)
//     // {
//     //     vector<Point2f> imagePoints1, imagePoints2;
//     //     for (int i = 0; i<good_matches_MR.size(); i++)
//     //     {
//     //         imagePoints1.push_back(keyPoint2[good_matches_MR[i].queryIdx].pt + offfset);  
//     //         imagePoints2.push_back(keyPoint3[good_matches_MR[i].trainIdx].pt);
//     //     }

//     //     //获取图像2到图像1的投影映射矩阵 尺寸为3*3  
//     //     homo_MR = findHomography(imagePoints2, imagePoints1, CV_RANSAC);
//     //     //计算配准图的四个顶点坐标
//     //     CalcCorners(homo_MR, img3);
//     //     //图像配准  
//     //     Mat  rightImageTransform;
//     //     warpPerspective(img3, rightImageTransform, homo_MR, Size(dst.cols, dst.rows));
//     //     // cv::imshow("rightImageTransform",rightImageTransform);

//     //     rightImageTransform.copyTo(dst(Rect(0, 0, rightImageTransform.cols, rightImageTransform.rows)));
//     //     // img2.copyTo(dst(Rect(img1.cols, 0, img2.cols, img2.rows)));
//     //     OptimizeSeam(img2, rightImageTransform, dst,offfset);

//     // }
//     // else//匹配点不足８个
//     // {
//     //     // img1.copyTo(dst(Rect(0, 0, img1.cols, img1.rows)));
//     //     drawText(dst);
//     //     cout << " Insufficient feature points, unable to realize right image stitching " << endl;
//     // }

//     return dst;

//     }
//     catch(...){
//         cout << "error!!!!" << endl;
//         return img1;
//     }

// }