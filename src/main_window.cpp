/**
 * @file /src/main_window.cpp
 *
 * @brief Implementation for the qt gui.
 *
 * @date February 2011
 **/
/*****************************************************************************
** Includes
*****************************************************************************/

#include <QtGui>
#include <QMessageBox>
#include <iostream>
#include "../include/image_stitching/main_window.hpp"
#include <QDebug>
#include <QMetaType>


/*****************************************************************************
** Namespaces
*****************************************************************************/

/*
 * 应用于 on_uav1Connect_pBtn_clicked()和on_uav2Connect_pBtn_clicked()
 * 0 --> 不运行bash命令
 * 1 --> 运行bash命令
 */
#define runCommand 1

namespace image_stitching {

using namespace Qt;

/*****************************************************************************
** Implementation [MainWindow]
*****************************************************************************/

MainWindow::MainWindow(int argc, char** argv, QWidget *parent)
	: QMainWindow(parent)
  , uav1Node(argc,argv)
  , uav2Node(argc,argv)
  , uav3Node(argc,argv)
{
	ui.setupUi(this); // Calling this incidentally connects all ui's triggers to on_...() callbacks in this class.

  moveUav1 = new moveUav(1);
  moveUav2 = new moveUav(2);
  moveUav3 = new moveUav(3);

  imageStitching = new stitching();
  uavControl = new uav_control();

  // 提示 connot queue arguments of type 'xxx'
  qRegisterMetaType< cv::Mat >("cv::Mat");
  qRegisterMetaType< geometry_msgs::Pose >("geometry_msgs::Pose");

  QObject::connect(ui.actionAbout_Qt, SIGNAL(triggered(bool)), qApp, SLOT(aboutQt())); // qApp is a global variable for the application

  QObject::connect(&uav1Node, SIGNAL(rosShutdown(int)), this, SLOT(deal_rosShutdown(int)));
  QObject::connect(&uav1Node,SIGNAL(showUav1ImageSignal(QImage)),this,SLOT(deal_showUav1ImageSignal(QImage)));
  QObject::connect(&uav1Node,SIGNAL(batteryDataSignal(int,bool)),this,SLOT(deal_uav1batteryDataSignal(int,bool)));
  QObject::connect(&uav1Node,SIGNAL(gpsDataSignal(int,double,double)),uavControl,SLOT(deal_uavgpsDataSignal(int,double,double)));
  QObject::connect(&uav1Node,SIGNAL(odomDataSignal(int,geometry_msgs::Pose)),uavControl,SLOT(deal_uavodomDataSignal(int,geometry_msgs::Pose)));

  QObject::connect(&uav2Node, SIGNAL(rosShutdown(int)), this, SLOT(deal_rosShutdown(int)));
  QObject::connect(&uav2Node,SIGNAL(showUav2ImageSignal(QImage)),this,SLOT(deal_showUav2ImageSignal(QImage)));
  QObject::connect(&uav2Node,SIGNAL(batteryDataSignal(int,bool)),this,SLOT(deal_uav2batteryDataSignal(int,bool)));
  QObject::connect(&uav2Node,SIGNAL(gpsDataSignal(int,double,double)),uavControl,SLOT(deal_uavgpsDataSignal(int,double,double)));
  QObject::connect(&uav2Node,SIGNAL(odomDataSignal(int,geometry_msgs::Pose)),uavControl,SLOT(deal_uavodomDataSignal(int,geometry_msgs::Pose)));

  QObject::connect(&uav3Node, SIGNAL(rosShutdown(int)), this, SLOT(deal_rosShutdown(int)));
  QObject::connect(&uav3Node,SIGNAL(showUav3ImageSignal(QImage)),this,SLOT(deal_showUav3ImageSignal(QImage)));
  QObject::connect(&uav3Node,SIGNAL(batteryDataSignal(int,bool)),this,SLOT(deal_uav3batteryDataSignal(int,bool)));
  QObject::connect(&uav3Node,SIGNAL(gpsDataSignal(int,double,double)),uavControl,SLOT(deal_uavgpsDataSignal(int,double,double)));
  QObject::connect(&uav3Node,SIGNAL(odomDataSignal(int,geometry_msgs::Pose)),uavControl,SLOT(deal_uavodomDataSignal(int,geometry_msgs::Pose)));

  QObject::connect(&uav1Node,SIGNAL(uav1RgbimageSignal(cv::Mat)),imageStitching,SLOT(deal_uav1RgbimageSignal(cv::Mat)));
  QObject::connect(&uav2Node,SIGNAL(uav2RgbimageSignal(cv::Mat)),imageStitching,SLOT(deal_uav2RgbimageSignal(cv::Mat)));
  QObject::connect(&uav3Node,SIGNAL(uav3RgbimageSignal(cv::Mat)),imageStitching,SLOT(deal_uav3RgbimageSignal(cv::Mat)));

  QObject::connect(imageStitching,SIGNAL(showStitchingImageSignal(QImage)),this,SLOT(deal_showStitchingImageSignal(QImage)));
  // 线程之间同步重合度
  QObject::connect(imageStitching,SIGNAL(overlapRateSignal(double,double)),uavControl,SLOT(deal_overlapRateSignal(double,double)));

  //uav1 飞行控制
  QObject::connect(moveUav1,SIGNAL(forwardSignal(int,bool)),this,SLOT(deal_forwardSignal(int,bool)));
  QObject::connect(moveUav1,SIGNAL(backwardSignal(int,bool)),this,SLOT(deal_backwardSignal(int,bool)));
  QObject::connect(moveUav1,SIGNAL(flayLeftSignal(int,bool)),this,SLOT(deal_flayLeftSignal(int,bool)));
  QObject::connect(moveUav1,SIGNAL(flayRightSignal(int,bool)),this,SLOT(deal_flayRightSignal(int,bool)));
  QObject::connect(moveUav1,SIGNAL(flayUpSignal(int,bool)),this,SLOT(deal_flayUpSignal(int,bool)));
  QObject::connect(moveUav1,SIGNAL(flayDownSignal(int,bool)),this,SLOT(deal_flayDownSignal(int,bool)));
  QObject::connect(moveUav1,SIGNAL(turnLeftSignal(int,bool)),this,SLOT(deal_turnLeftSignal(int,bool)));
  QObject::connect(moveUav1,SIGNAL(turnRightSignal(int,bool)),this,SLOT(deal_turnRightSignal(int,bool)));
  QObject::connect(moveUav1,SIGNAL(cameraControSignal(int,double,double)),this,SLOT(deal_cameraControSignal(int,double,double)));

  //uav2 飞行控制
  QObject::connect(moveUav2,SIGNAL(forwardSignal(int,bool)),this,SLOT(deal_forwardSignal(int,bool)));
  QObject::connect(moveUav2,SIGNAL(backwardSignal(int,bool)),this,SLOT(deal_backwardSignal(int,bool)));
  QObject::connect(moveUav2,SIGNAL(flayLeftSignal(int,bool)),this,SLOT(deal_flayLeftSignal(int,bool)));
  QObject::connect(moveUav2,SIGNAL(flayRightSignal(int,bool)),this,SLOT(deal_flayRightSignal(int,bool)));
  QObject::connect(moveUav2,SIGNAL(flayUpSignal(int,bool)),this,SLOT(deal_flayUpSignal(int,bool)));
  QObject::connect(moveUav2,SIGNAL(flayDownSignal(int,bool)),this,SLOT(deal_flayDownSignal(int,bool)));
  QObject::connect(moveUav2,SIGNAL(turnLeftSignal(int,bool)),this,SLOT(deal_turnLeftSignal(int,bool)));
  QObject::connect(moveUav2,SIGNAL(turnRightSignal(int,bool)),this,SLOT(deal_turnRightSignal(int,bool)));
  QObject::connect(moveUav2,SIGNAL(cameraControSignal(int,double,double)),this,SLOT(deal_cameraControSignal(int,double,double)));

  //uav3 飞行控制
  QObject::connect(moveUav3,SIGNAL(forwardSignal(int,bool)),this,SLOT(deal_forwardSignal(int,bool)));
  QObject::connect(moveUav3,SIGNAL(backwardSignal(int,bool)),this,SLOT(deal_backwardSignal(int,bool)));
  QObject::connect(moveUav3,SIGNAL(flayLeftSignal(int,bool)),this,SLOT(deal_flayLeftSignal(int,bool)));
  QObject::connect(moveUav3,SIGNAL(flayRightSignal(int,bool)),this,SLOT(deal_flayRightSignal(int,bool)));
  QObject::connect(moveUav3,SIGNAL(flayUpSignal(int,bool)),this,SLOT(deal_flayUpSignal(int,bool)));
  QObject::connect(moveUav3,SIGNAL(flayDownSignal(int,bool)),this,SLOT(deal_flayDownSignal(int,bool)));
  QObject::connect(moveUav3,SIGNAL(turnLeftSignal(int,bool)),this,SLOT(deal_turnLeftSignal(int,bool)));
  QObject::connect(moveUav3,SIGNAL(turnRightSignal(int,bool)),this,SLOT(deal_turnRightSignal(int,bool)));
  QObject::connect(moveUav3,SIGNAL(cameraControSignal(int,double,double)),this,SLOT(deal_cameraControSignal(int,double,double)));

  //自主飞行控制
  QObject::connect(uavControl,SIGNAL(forwardSignal(int,bool)),this,SLOT(deal_forwardSignal(int,bool)));
  QObject::connect(uavControl,SIGNAL(backwardSignal(int,bool)),this,SLOT(deal_backwardSignal(int,bool)));
  QObject::connect(uavControl,SIGNAL(flayLeftSignal(int,bool)),this,SLOT(deal_flayLeftSignal(int,bool)));
  QObject::connect(uavControl,SIGNAL(flayRightSignal(int,bool)),this,SLOT(deal_flayRightSignal(int,bool)));
  QObject::connect(uavControl,SIGNAL(flayUpSignal(int,bool)),this,SLOT(deal_flayUpSignal(int,bool)));
  QObject::connect(uavControl,SIGNAL(flayDownSignal(int,bool)),this,SLOT(deal_flayDownSignal(int,bool)));
  QObject::connect(uavControl,SIGNAL(turnLeftSignal(int,bool)),this,SLOT(deal_turnLeftSignal(int,bool)));
  QObject::connect(uavControl,SIGNAL(turnRightSignal(int,bool)),this,SLOT(deal_turnRightSignal(int,bool)));
  QObject::connect(uavControl,SIGNAL(uav_FBcontrolSignal(double, double, double, double)),this,SLOT(deal_uav_FBcontrolSignal(double, double, double, double)));


    uav1Name = "bebop3";//默认值
    ui.uav1Land_pBtn->setEnabled(false);
    ui.uav1Takeoff_pBtn->setEnabled(false);
    ui.uav1Move_pBtn->setEnabled(false);
    ui.uav1Battery_pBar->setEnabled(false);

    uav2Name = "bebop4";//默认值
    ui.uav2Land_pBtn->setEnabled(false);
    ui.uav2Takeoff_pBtn->setEnabled(false);
    ui.uav2Move_pBtn->setEnabled(false);
    ui.uav2Battery_pBar->setEnabled(false);

    uav3Name = "bebop5";//默认值
    ui.uav3Land_pBtn->setEnabled(false);
    ui.uav3Takeoff_pBtn->setEnabled(false);
    ui.uav3Move_pBtn->setEnabled(false);
    ui.uav3Battery_pBar->setEnabled(false);

    imageStitching->start();
    uavControl->start();

    // 加载图片
    ui.stitchingImage_label->setPixmap(QPixmap(":/images/load1.png"));
    ui.uav1Image_label->setPixmap(QPixmap(":/images/load.png"));
    ui.uav2Image_label->setPixmap(QPixmap(":/images/load.png"));
    ui.uav3Image_label->setPixmap(QPixmap(":/images/load.png"));
    ui.school_label->setPixmap(QPixmap(":/images/school.png"));
}

MainWindow::~MainWindow() {}

void MainWindow::closeEvent(QCloseEvent *event)
{

  uav1Node.isRun = false;
  uav2Node.isRun = false;
  uav3Node.isRun = false;

  moveUav2->close();
  moveUav1->close();
  moveUav3->close();

  /* 关闭图像拼接进程 */
  imageStitching->stitchingThreadStatue = false;
  imageStitching->isStitching = false;
  imageStitching->imageRecOK.wakeAll();
  imageStitching->quit();
  imageStitching->wait();

  uavControl->autoFlyThreadStatue = false;
  uavControl->isAutoFly = false;
  uavControl->quit();
  uavControl->wait();

  delete imageStitching;
  delete uavControl;
  imageStitching = NULL;
  uavControl = NULL;

  delete moveUav1;
  delete moveUav2;
  delete moveUav3;

  moveUav1 = NULL;
  moveUav2 = NULL;
  moveUav3 = NULL;
  QMainWindow::closeEvent(event);

}

/*****************************************************************************
** Implementation [Slots]
*****************************************************************************/

void MainWindow::showNoMasterMessage() {
	QMessageBox msgBox;
	msgBox.setText("Couldn't find the ros master.");
	msgBox.exec();
    close();
}


void MainWindow::deal_showStitchingImageSignal(QImage image)
{
  displayStitchingImage(image);
}
void MainWindow::displayStitchingImage(const QImage image)
{
  stitchingImage_mutex_.lock();
  stitchingImage = image.copy();
  ui.stitchingImage_label->setPixmap(QPixmap::fromImage(stitchingImage));
//  ui.uav1Image_label->resize(ui.uav1Image_label->pixmap()->size());
  stitchingImage_mutex_.unlock();

  //显示重合度
  QString overlap =QString("%1").arg(imageStitching->overlap_rate_left) ;
  ui.overlapRate_left_lineEdit->setText(overlap) ;
  overlap =QString("%1").arg(imageStitching->overlap_rate_right) ;
  ui.overlapRate_right_lineEdit->setText(overlap) ;
}

void MainWindow::deal_cameraControSignal(int UAVx, double vertical, double horizontal)
{
  if(UAVx == 1)
    uav1Node.cameraControl(vertical,horizontal);
  else if(UAVx == 2)
    uav2Node.cameraControl(vertical,horizontal);
  else if(UAVx == 3)
    uav3Node.cameraControl(vertical,horizontal);
}
void MainWindow::deal_uav_FBcontrolSignal(double controlUav1, double controlUav3, double yawUav1, double yawUav3)
{
  if(yawUav1 >= 0.02 || yawUav1 <= -0.02)
    uav1Node.cmd(0, 0, 0, yawUav1);
  else
    uav1Node.cmd(controlUav1, 0, 0, yawUav1);

  std::cout <<"controlUav1  " << controlUav1 << std::endl;
//  uav3Node.cmd(uav3Control, 0, 0, 0);
}
void  MainWindow::deal_rosShutdown(int UAVx)
{
  if(UAVx == 1)
  {
    std::cout << "11111111" << std::endl;
//    if(uav1Node.isRunning() == true)
//    {
////      uav1Node.st
//      std::cout << "22222222222" << std::endl;
//      uav1Node.quit();
//      uav1Node.wait();
//    }
    ui.uav1Land_pBtn->setEnabled(false);
    ui.uav1Takeoff_pBtn->setEnabled(false);
    ui.uav1Move_pBtn->setEnabled(false);
    ui.uav1Battery_pBar->setEnabled(false);
  }
  else if(UAVx == 2)
  {
    uav2Node.quit();
    uav2Node.wait();
    ui.uav2Land_pBtn->setEnabled(false);
    ui.uav2Takeoff_pBtn->setEnabled(false);
    ui.uav2Move_pBtn->setEnabled(false);
    ui.uav2Battery_pBar->setEnabled(false);
  }
  else if(UAVx == 3)
  {
    uav3Node.quit();
    uav3Node.wait();
    ui.uav3Land_pBtn->setEnabled(false);
    ui.uav3Takeoff_pBtn->setEnabled(false);
    ui.uav3Move_pBtn->setEnabled(false);
    ui.uav3Battery_pBar->setEnabled(false);
  }
//  uavControl->quit();
//  uavControl->wait();
}

/* ******************************************************  无人机1 ******************************************************* */

void MainWindow::displayUav1Image(const QImage image)
{
  uav1Image_mutex_.lock();
  uav1Image = image.copy();
  ui.uav1Image_label->setPixmap(QPixmap::fromImage(uav1Image));
//  ui.uav1Image_label->resize(ui.uav1Image_label->pixmap()->size());
  uav1Image_mutex_.unlock();

}

void MainWindow::deal_showUav1ImageSignal(QImage image)
{
//  static int i=0;
//  qDebug() << "deal_showUav1ImageSignal"<<i;
//  i++;
  displayUav1Image(image);
}
void MainWindow::deal_uav1batteryDataSignal(int batteryData, bool batteryState)
{
  if(batteryState == false)
    ui.uav1Battery_pBar->setEnabled(false);
  else
  {
    if(ui.uav1Battery_pBar->isEnabled() == false)
      ui.uav1Battery_pBar->setEnabled(true);
    ui.uav1Battery_pBar->setValue(batteryData);
  }
}
void MainWindow::on_uav1Takeoff_pBtn_clicked()
{
  uav1Node.takeoff();
}

void MainWindow::on_uav1Land_pBtn_clicked()
{
  uav1Node.land();
}

void MainWindow::on_uav1Connect_pBtn_clicked()
{
  uav1Name = ui.uav1Name_cBox->currentText().toStdString();
// window -e 'bash -c "roslaunch bebop_driver bebop_node.launch"'
#if runCommand

  std::string str = "gnome-terminal --window -e 'bash -c \"source /home/linux/work/driver/bebop_ws/devel/setup.bash; roslaunch bebop_driver "
                        + uav1Name
                        +"_node.launch; exec bash\"'&"; //'";
  const char *command = str.c_str();
  system(command);
#endif

}

void MainWindow::on_uav1Move_pBtn_clicked()
{
  moveUav1->show();
}

void MainWindow::on_uav1ShowImage_pBtn_clicked()
{
  if(uav1Node.isRun == false)
  {
    uav1Name = ui.uav1Name_cBox->currentText().toStdString();
    uav1Node.isRun = true;
    if ( !uav1Node.init(uav1Name) )
    {
      uav1Node.isRun = false;
      showNoMasterMessage();
    }
    else
    {
      ui.uav1Land_pBtn->setEnabled(true);
      ui.uav1Takeoff_pBtn->setEnabled(true);
      ui.uav1Move_pBtn->setEnabled(true);
      ui.uav1Battery_pBar->setEnabled(true);
      ui.uav1ShowImage_pBtn->setText(QString::fromUtf8("停止"));
    }
  }
  else if(uav1Node.isRun == true)
  {
    uav1Node.isRun = false;
    ui.uav1ShowImage_pBtn->setText(QString::fromUtf8("启动"));
//    ui.uav1Land_pBtn->setEnabled(false);
//    ui.uav1Takeoff_pBtn->setEnabled(false);
//    ui.uav1Move_pBtn->setEnabled(false);
//    ui.uav1Battery_pBar->setEnabled(false);
  }

}

/* *******************************************************  无人机2 ******************************************************* */

void MainWindow::deal_showUav2ImageSignal(QImage image)
{
//  static int i=0;
//  qDebug() << "deal_showUav1ImageSignal"<<i;
//  i++;
  displayUav2Image(image);
}
void MainWindow::displayUav2Image(const QImage image)
{
  uav2Image_mutex_.lock();
  uav2Image = image.copy();
  ui.uav2Image_label->setPixmap(QPixmap::fromImage(uav2Image));
//  ui.uav2Image_label->resize(ui.uav2Image_label->pixmap()->size());
  uav2Image_mutex_.unlock();
}
void MainWindow::deal_uav2batteryDataSignal(int batteryData, bool batteryState)
{
  if(batteryState == false)
    ui.uav2Battery_pBar->setEnabled(false);
  else
  {
    if(ui.uav2Battery_pBar->isEnabled() == false)
      ui.uav2Battery_pBar->setEnabled(true);
    ui.uav2Battery_pBar->setValue(batteryData);
  }
}

void MainWindow::on_uav2Takeoff_pBtn_clicked()
{
  uav2Node.takeoff();
}

void MainWindow::on_uav2Land_pBtn_clicked()
{
  uav2Node.land();
}

void MainWindow::on_uav2Connect_pBtn_clicked()
{
  uav2Name = ui.uav2Name_cBox->currentText().toStdString();

#if runCommand
  std::string str = "gnome-terminal --window -e 'bash -c \"source /home/linux/work/driver/bebop_ws/devel/setup.bash; roslaunch bebop_driver "
                        + uav2Name
                        +"_node.launch; exec bash\"'&";
  const char *command = str.c_str();
  system(command);
#endif
}

void MainWindow::on_uav2Move_pBtn_clicked()
{
  moveUav2->show();
}

void MainWindow::on_uav2ShowImage_pBtn_clicked()
{

  if(uav2Node.isRun == false)
  {
    uav2Name = ui.uav2Name_cBox->currentText().toStdString();
    uav2Node.isRun = true;
    if ( !uav2Node.init(uav2Name) )
    {
      uav2Node.isRun = false;
      showNoMasterMessage();
    }
    else
    {
      ui.uav2Land_pBtn->setEnabled(true);
      ui.uav2Takeoff_pBtn->setEnabled(true);
      ui.uav2Move_pBtn->setEnabled(true);
      ui.uav2Battery_pBar->setEnabled(true);
      ui.uav2ShowImage_pBtn->setText(QString::fromUtf8("停止"));
    }
  }
  else if(uav2Node.isRun == true)
  {
    uav2Node.isRun = false;
    ui.uav2ShowImage_pBtn->setText(QString::fromUtf8("启动"));
  }

}

/* *******************************************************  无人机3 ******************************************************* */

void MainWindow::deal_showUav3ImageSignal(QImage image)
{

  displayUav3Image(image);
}
void MainWindow::displayUav3Image(const QImage image)
{
  uav3Image_mutex_.lock();
  uav3Image = image.copy();
  ui.uav3Image_label->setPixmap(QPixmap::fromImage(uav3Image));
//  ui.uav3Image_label->resize(ui.uav3Image_label->pixmap()->size());
  uav3Image_mutex_.unlock();
}
void MainWindow::deal_uav3batteryDataSignal(int batteryData, bool batteryState)
{
  if(batteryState == false)
    ui.uav3Battery_pBar->setEnabled(false);
  else
  {
    if(ui.uav3Battery_pBar->isEnabled() == false)
      ui.uav3Battery_pBar->setEnabled(true);
    ui.uav3Battery_pBar->setValue(batteryData);
  }
}
void MainWindow::on_uav3Takeoff_pBtn_clicked()
{
  uav3Node.takeoff();
}

void MainWindow::on_uav3Land_pBtn_clicked()
{
  uav3Node.land();
}

void MainWindow::on_uav3Connect_pBtn_clicked()
{
  uav3Name = ui.uav3Name_cBox->currentText().toStdString();
//  std::string str = "gnome-terminal -x bash -c 'roslaunch bebop_driver "
//                        + uav3Name
//                        +"_node.launch'&";
#if runCommand
  std::string str = "gnome-terminal --window -e 'bash -c \"source /home/linux/work/driver/bebop_ws/devel/setup.bash; roslaunch bebop_driver "
                        + uav3Name
                        +"_node.launch; exec bash\"'&";
  const char *command = str.c_str();
  system(command);
#endif
}

void MainWindow::on_uav3Move_pBtn_clicked()
{
  moveUav3->show();
}

void MainWindow::on_uav3ShowImage_pBtn_clicked()
{
  if(uav3Node.isRun == false)
  {
    uav3Name = ui.uav3Name_cBox->currentText().toStdString();
    uav3Node.isRun = true;
    if ( !uav3Node.init(uav3Name) )
    {
      uav3Node.isRun = false;
      showNoMasterMessage();
    }
    else
    {
      ui.uav3Land_pBtn->setEnabled(true);
      ui.uav3Takeoff_pBtn->setEnabled(true);
      ui.uav3Move_pBtn->setEnabled(true);
      ui.uav3Battery_pBar->setEnabled(true);
      ui.uav3ShowImage_pBtn->setText(QString::fromUtf8("停止"));
    }
  }
  else if(uav3Node.isRun == true)
  {
    uav3Node.isRun = false;
    ui.uav3ShowImage_pBtn->setText(QString::fromUtf8("启动"));
  }
}

/* *******************************************************  控制无人机的移动 ******************************************************* */
void MainWindow::deal_forwardSignal(int UAVx, bool state)
{
  if(UAVx == 1)
    uav1Node.forward = state;
  else if(UAVx == 2)
    uav2Node.forward = state;
  else if(UAVx == 3)
    uav3Node.forward = state;
}
void MainWindow::deal_backwardSignal(int UAVx, bool state)
{
  if(UAVx == 1)
    uav1Node.backward = state;
  else if(UAVx == 2)
    uav2Node.backward = state;
  else if(UAVx == 3)
    uav3Node.backward = state;
}
void MainWindow::deal_flayLeftSignal(int UAVx, bool state)
{
  if(UAVx == 1)
    uav1Node.flayLeft = state;
  else if(UAVx == 2)
    uav2Node.flayLeft = state;
  else if(UAVx == 3)
    uav3Node.flayLeft = state;
}
void MainWindow::deal_flayRightSignal(int UAVx, bool state)
{
  if(UAVx == 1)
    uav1Node.flayRight = state;
  else if(UAVx == 2)
    uav2Node.flayRight = state;
  else if(UAVx == 3)
    uav3Node.flayRight = state;
}
void MainWindow::deal_flayUpSignal(int UAVx, bool state)
{
  if(UAVx == 1)
    uav1Node.flayUp = state;
  else if(UAVx == 2)
    uav2Node.flayUp = state;
  else if(UAVx == 3)
    uav3Node.flayUp = state;
}
void MainWindow::deal_flayDownSignal(int UAVx, bool state)
{
  if(UAVx == 1)
    uav1Node.flayDown = state;
  else if(UAVx == 2)
    uav2Node.flayDown = state;
  else if(UAVx == 3)
    uav3Node.flayDown = state;
}
void MainWindow::deal_turnLeftSignal(int UAVx, bool state)
{
  if(UAVx == 1)
    uav1Node.turnLeft = state;
  else if(UAVx == 2)
    uav2Node.turnLeft = state;
  else if(UAVx == 3)
    uav3Node.turnLeft = state;
}
void MainWindow::deal_turnRightSignal(int UAVx, bool state)
{
  if(UAVx == 1)
    uav1Node.turnRight = state;
  else if(UAVx == 2)
    uav2Node.turnRight = state;
  else if(UAVx == 3)
    uav3Node.turnRight = state;
}


// 是否开始图像拼接
void MainWindow::on_stitching_checkBox_stateChanged(int arg1)
{
    if(ui.stitching_checkBox->isChecked() == true)
    {
      imageStitching->isStitching = true;
    }
    else
    {
      imageStitching->isStitching = false;
    }
}
// 自动飞行
void MainWindow::on_autoFly_pBtn_clicked()
{
  if(uavControl->isAutoFly == false)
  {
//    uavControl->autoFlyThreadStatue = true;
    uavControl->isAutoFly = true;
    ui.autoFly_pBtn->setText(QString::fromUtf8("切换手动模式"));
//    uavControl->start();
  }
  else {
//    uavControl->autoFlyThreadStatue = false;
    uavControl->isAutoFly = false;
    ui.autoFly_pBtn->setText(QString::fromUtf8("自主飞行"));
//    uavControl->quit();
//    uavControl->wait();
  }
}

void MainWindow::on_setYawErr_pBtn_clicked()
{
  uavControl->yawOffset[1] = ui.yawOffset_uav1_lineEdit->text().toDouble() * 3.1415926 / 180.0;
  uavControl->yawOffset[2] = ui.yawOffset_uav2_lineEdit->text().toDouble() * 3.1415926 / 180.0;
  uavControl->yawOffset[3] = ui.yawOffset_uav3_lineEdit->text().toDouble() * 3.1415926 / 180.0;

  uavControl->yawOffset_21 = uavControl->yawOffset[2] - uavControl->yawOffset[1];

}

}  // namespace image_stitching







