#include <opencv2/core/core.hpp>
#include <iostream>  
using namespace cv;
using namespace std;
class imageStitching
{
public:
    imageStitching();

    typedef struct
    {
        Point2f left_top;
        Point2f left_bottom;
        Point2f right_top;
        Point2f right_bottom;
    }four_corners_t;
    
    four_corners_t leftImg_corners, rightImg_corners;
    four_corners_t middleImg_corners;
    four_corners_t overlap; // 记录重叠区域的顶点
    float overlap_rate_left, overlap_rate_right;


    //优化两图的连接处，使得拼接自然
    void OptimizeSeam(Mat& sourceImage, Mat& transImage, Mat& dst,four_corners_t source_corners,four_corners_t trans_corners);
    void CalcCorners( Mat H,  Mat src, four_corners_t &corners);
    cv::Mat stitchingThreeImage(Mat img1, Mat img2, Mat img3);
    void drawText(Mat & image, float rate, Point centerpoint);

// "Insufficient feature points, unable to realize image stitching"

private:
    /* data */
};
