#ifndef Q_MOC_RUN
#include <opencv2/core/core.hpp>
#include <opencv2/opencv.hpp>
#endif

#include <iostream>
#include <QThread>
#include <string>
#include <QGraphicsView>

#include "kcftracker.hpp"

namespace image_stitching {

class tracker_thread : public QThread
{
    Q_OBJECT
public:
  tracker_thread(QWidget *parent = 0);
  virtual ~tracker_thread();

  QPointF mousePosition_start,mousePosition_current;
  bool mousePress;

  cv::Rect2d trackRoi;
  cv::Rect trackResult;
  cv::Mat srcImage;
  KCFTracker tracker;
  bool recImage_flag;
  bool is_tracking;
  bool trackerInit_flag;
  bool trackerThreadStatus;
  cv::Mat rgbImage;
  QImage  showImage;

  double scale_x , scale_y;


  void run();

public Q_SLOTS:
//  void deal_showStitchingImageSignal(QImage image);
  void deal_trackStitchingImageSignal(cv::Mat bgrImage);

Q_SIGNALS://Qt信号
  void trackFinishSignal(bool);



};
}


