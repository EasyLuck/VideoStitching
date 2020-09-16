#include <opencv2/core/core.hpp>
#include <iostream>  
using namespace cv;
using namespace std;
class imageStitching
{
public:
    imageStitching();

    //优化两图的连接处，使得拼接自然
    void OptimizeSeamRight(Mat& leftImage, Mat& trans, Mat& dst,Point2f offfset);
    void OptimizeSeamleft(Mat& rightImage, Mat& trans, Mat& dst,Point2f offfset);
    void CalcCorners( Mat H,  Mat src);
    cv::Mat imageProcess(Mat img1, Mat img2);
    cv::Mat imageProcessLeft(Mat img1, Mat img2);
    cv::Mat stitchingThreeImage(Mat img1, Mat img2, Mat img3);
    void drawText(Mat & image);

    typedef struct
    {
        Point2f left_top;
        Point2f left_bottom;
        Point2f right_top;
        Point2f right_bottom;
    }four_corners_t;
    four_corners_t corners;

private:
    /* data */
};
