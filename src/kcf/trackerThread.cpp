#include "../../include/image_stitching/kcf/trackerThread.hpp"
#include <unistd.h>


//using namespace std;
//using namespace cv;

namespace image_stitching {

tracker_thread::tracker_thread(QWidget *parent) : QThread(parent)
{

  mousePosition_start.setX(0);
  mousePosition_start.setY(0);
  mousePosition_current.setX(0);
  mousePosition_current.setY(0);

  mousePress = false;

}


tracker_thread::~tracker_thread() {

}

void tracker_thread::run()
{}

}
