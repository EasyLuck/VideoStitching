#ifndef Q_MOC_RUN
#include <opencv2/core/core.hpp>
#include <opencv2/opencv.hpp>
#endif

#include <iostream>
#include <QThread>
#include <string>
#include <QGraphicsView>

namespace image_stitching {

class tracker_thread : public QThread
{
    Q_OBJECT
public:
  tracker_thread(QWidget *parent = 0);
  virtual ~tracker_thread();

  QPointF mousePosition_start,mousePosition_current;
  bool mousePress;
  void run();

public Q_SLOTS:

Q_SIGNALS://Qt信号



};
}


