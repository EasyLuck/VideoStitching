#include "../../include/image_stitching/kcf/trackerThread.hpp"
#include <unistd.h>
#include "../../include/image_stitching/kcf/kcftracker.hpp"



namespace image_stitching {

using namespace std;
using namespace cv;


tracker_thread::tracker_thread(QWidget *parent)
  : QThread(parent)
  ,tracker(true, false, true, false)//初始化KCF追踪器
{

  mousePosition_start.setX(0);
  mousePosition_start.setY(0);
  mousePosition_current.setX(0);
  mousePosition_current.setY(0);

  mousePress = false;
  is_tracking = false;
  trackerInit_flag = false;
  trackerThreadStatus = true;
  recImage_flag = false;

  // stitchingImg  width: 856X3   height: 480X2
  // graphicsView width: 960      height: 480
  scale_x = 856*3.0/960.0;
  scale_y = 2;

}


tracker_thread::~tracker_thread() {

}

void tracker_thread::run()
{

  while(trackerThreadStatus)
  {
    if(recImage_flag)   // 接收到图像
    {
      recImage_flag = false;

      if(is_tracking){  // 点击了跟踪按钮

        if(trackerInit_flag == false) // 初始化跟踪器
        {
          tracker.init(trackRoi, srcImage);
          trackerInit_flag = true;
        }
        if (srcImage.rows == 0 || srcImage.cols == 0)
        {
          cout << "frame is error" << endl;
          continue;
        }
        else
        {
          trackResult = tracker.update(srcImage);
          rectangle(srcImage, Point(trackResult.x, trackResult.y), Point(trackResult.x + trackResult.width, trackResult.y + trackResult.height), Scalar(0, 0, 255), 3, 8);
        }
//        std::cout << "this is trackerThread " << std::endl;
      }
      //转换为QImage
      cvtColor(srcImage, rgbImage, CV_BGR2RGB);
      showImage = QImage(rgbImage.data,rgbImage.cols,rgbImage.rows,rgbImage.step[0],QImage::Format_RGB888);
      trackFinishSignal(true);

    }
    msleep(100);  // 33.3Hz
  }
}

void tracker_thread::deal_trackStitchingImageSignal(cv::Mat bgrImage)
{
  bgrImage.copyTo(srcImage);
  recImage_flag = true;
}

}
