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
  , moveUav1(1,parent)
  , moveUav2(2,parent)
  , moveUav3(3,parent)
  , imageStitching(parent)
  , uavControl(parent)
  , trackerThread(parent)
  , uavShow(parent)
//  , trackerWindow(parent)
{
	ui.setupUi(this); // Calling this incidentally connects all ui's triggers to on_...() callbacks in this class.

  MyTimer = new QTimer(this);
  uavShow.show();
  // 提示 connot queue arguments of type 'xxx'
  qRegisterMetaType< cv::Mat >("cv::Mat");
  qRegisterMetaType< nav_msgs::Odometry >("nav_msgs::Odometry");
  qRegisterMetaType< geometry_msgs::Twist >("geometry_msgs::Twist");

  QObject::connect(ui.actionAbout_Qt, SIGNAL(triggered(bool)), qApp, SLOT(aboutQt())); // qApp is a global variable for the application

  QObject::connect(MyTimer,SIGNAL(timeout()), this, SLOT(deal_timeout()));

  QObject::connect(&uav1Node, SIGNAL(rosShutdown(int)), this, SLOT(deal_rosShutdown(int)));
  QObject::connect(&uav1Node,SIGNAL(batteryDataSignal(int,bool)),this,SLOT(deal_uav1batteryDataSignal(int,bool)));
//  QObject::connect(&uav1Node,SIGNAL(gpsDataSignal(int,double,double)),&uavControl,SLOT(deal_uavgpsDataSignal(int,double,double)));
  QObject::connect(&uav1Node,SIGNAL(odomDataSignal(int,nav_msgs::Odometry)),&uavControl,SLOT(deal_uavodomDataSignal(int,nav_msgs::Odometry)));

  QObject::connect(&uav2Node, SIGNAL(rosShutdown(int)), this, SLOT(deal_rosShutdown(int)));
  QObject::connect(&uav2Node,SIGNAL(batteryDataSignal(int,bool)),this,SLOT(deal_uav2batteryDataSignal(int,bool)));
//  QObject::connect(&uav2Node,SIGNAL(gpsDataSignal(int,double,double)),&uavControl,SLOT(deal_uavgpsDataSignal(int,double,double)));
  QObject::connect(&uav2Node,SIGNAL(odomDataSignal(int,nav_msgs::Odometry)),&uavControl,SLOT(deal_uavodomDataSignal(int,nav_msgs::Odometry)));

  QObject::connect(&uav3Node, SIGNAL(rosShutdown(int)), this, SLOT(deal_rosShutdown(int)));
  QObject::connect(&uav3Node,SIGNAL(batteryDataSignal(int,bool)),this,SLOT(deal_uav3batteryDataSignal(int,bool)));
//  QObject::connect(&uav3Node,SIGNAL(gpsDataSignal(int,double,double)),&uavControl,SLOT(deal_uavgpsDataSignal(int,double,double)));
  QObject::connect(&uav3Node,SIGNAL(odomDataSignal(int,nav_msgs::Odometry)),&uavControl,SLOT(deal_uavodomDataSignal(int,nav_msgs::Odometry)));

  QObject::connect(&uav1Node,SIGNAL(uav1RgbimageSignal(cv::Mat)),this,SLOT(deal_uav1RgbimageSignal(cv::Mat)));
  QObject::connect(&uav2Node,SIGNAL(uav2RgbimageSignal(cv::Mat)),this,SLOT(deal_uav2RgbimageSignal(cv::Mat)));
  QObject::connect(&uav3Node,SIGNAL(uav3RgbimageSignal(cv::Mat)),this,SLOT(deal_uav3RgbimageSignal(cv::Mat)));

//  QObject::connect(&imageStitching,SIGNAL(showStitchingImageSignal(QImage)),this,SLOT(deal_showStitchingImageSignal(QImage)));
  // 线程之间同步重合度
  QObject::connect(&imageStitching,SIGNAL(overlapRateSignal(double,double)),&uavControl,SLOT(deal_overlapRateSignal(double,double)));

  //uav1 飞行控制
  QObject::connect(&moveUav1,SIGNAL(forwardSignal(int,bool)),this,SLOT(deal_forwardSignal(int,bool)));
  QObject::connect(&moveUav1,SIGNAL(backwardSignal(int,bool)),this,SLOT(deal_backwardSignal(int,bool)));
  QObject::connect(&moveUav1,SIGNAL(flayLeftSignal(int,bool)),this,SLOT(deal_flayLeftSignal(int,bool)));
  QObject::connect(&moveUav1,SIGNAL(flayRightSignal(int,bool)),this,SLOT(deal_flayRightSignal(int,bool)));
  QObject::connect(&moveUav1,SIGNAL(flayUpSignal(int,bool)),this,SLOT(deal_flayUpSignal(int,bool)));
  QObject::connect(&moveUav1,SIGNAL(flayDownSignal(int,bool)),this,SLOT(deal_flayDownSignal(int,bool)));
  QObject::connect(&moveUav1,SIGNAL(turnLeftSignal(int,bool)),this,SLOT(deal_turnLeftSignal(int,bool)));
  QObject::connect(&moveUav1,SIGNAL(turnRightSignal(int,bool)),this,SLOT(deal_turnRightSignal(int,bool)));
  QObject::connect(&moveUav1,SIGNAL(cameraControSignal(int,double,double)),this,SLOT(deal_cameraControSignal(int,double,double)));

  //uav2 飞行控制
  QObject::connect(&moveUav2,SIGNAL(forwardSignal(int,bool)),this,SLOT(deal_forwardSignal(int,bool)));
  QObject::connect(&moveUav2,SIGNAL(backwardSignal(int,bool)),this,SLOT(deal_backwardSignal(int,bool)));
  QObject::connect(&moveUav2,SIGNAL(flayLeftSignal(int,bool)),this,SLOT(deal_flayLeftSignal(int,bool)));
  QObject::connect(&moveUav2,SIGNAL(flayRightSignal(int,bool)),this,SLOT(deal_flayRightSignal(int,bool)));
  QObject::connect(&moveUav2,SIGNAL(flayUpSignal(int,bool)),this,SLOT(deal_flayUpSignal(int,bool)));
  QObject::connect(&moveUav2,SIGNAL(flayDownSignal(int,bool)),this,SLOT(deal_flayDownSignal(int,bool)));
  QObject::connect(&moveUav2,SIGNAL(turnLeftSignal(int,bool)),this,SLOT(deal_turnLeftSignal(int,bool)));
  QObject::connect(&moveUav2,SIGNAL(turnRightSignal(int,bool)),this,SLOT(deal_turnRightSignal(int,bool)));
  QObject::connect(&moveUav2,SIGNAL(cameraControSignal(int,double,double)),this,SLOT(deal_cameraControSignal(int,double,double)));

  //uav3 飞行控制
  QObject::connect(&moveUav3,SIGNAL(forwardSignal(int,bool)),this,SLOT(deal_forwardSignal(int,bool)));
  QObject::connect(&moveUav3,SIGNAL(backwardSignal(int,bool)),this,SLOT(deal_backwardSignal(int,bool)));
  QObject::connect(&moveUav3,SIGNAL(flayLeftSignal(int,bool)),this,SLOT(deal_flayLeftSignal(int,bool)));
  QObject::connect(&moveUav3,SIGNAL(flayRightSignal(int,bool)),this,SLOT(deal_flayRightSignal(int,bool)));
  QObject::connect(&moveUav3,SIGNAL(flayUpSignal(int,bool)),this,SLOT(deal_flayUpSignal(int,bool)));
  QObject::connect(&moveUav3,SIGNAL(flayDownSignal(int,bool)),this,SLOT(deal_flayDownSignal(int,bool)));
  QObject::connect(&moveUav3,SIGNAL(turnLeftSignal(int,bool)),this,SLOT(deal_turnLeftSignal(int,bool)));
  QObject::connect(&moveUav3,SIGNAL(turnRightSignal(int,bool)),this,SLOT(deal_turnRightSignal(int,bool)));
  QObject::connect(&moveUav3,SIGNAL(cameraControSignal(int,double,double)),this,SLOT(deal_cameraControSignal(int,double,double)));

  //自主飞行控制
  QObject::connect(&uavControl,SIGNAL(forwardSignal(int,bool)),this,SLOT(deal_forwardSignal(int,bool)));
  QObject::connect(&uavControl,SIGNAL(backwardSignal(int,bool)),this,SLOT(deal_backwardSignal(int,bool)));
  QObject::connect(&uavControl,SIGNAL(flayLeftSignal(int,bool)),this,SLOT(deal_flayLeftSignal(int,bool)));
  QObject::connect(&uavControl,SIGNAL(flayRightSignal(int,bool)),this,SLOT(deal_flayRightSignal(int,bool)));
  QObject::connect(&uavControl,SIGNAL(flayUpSignal(int,bool)),this,SLOT(deal_flayUpSignal(int,bool)));
  QObject::connect(&uavControl,SIGNAL(flayDownSignal(int,bool)),this,SLOT(deal_flayDownSignal(int,bool)));
  QObject::connect(&uavControl,SIGNAL(turnLeftSignal(int,bool)),this,SLOT(deal_turnLeftSignal(int,bool)));
  QObject::connect(&uavControl,SIGNAL(turnRightSignal(int,bool)),this,SLOT(deal_turnRightSignal(int,bool)));
  QObject::connect(&uavControl,SIGNAL(uavTargetVelocitySignal(geometry_msgs::Twist,geometry_msgs::Twist, geometry_msgs::Twist)),this,SLOT(deal_uavTargetVelocitySignal(geometry_msgs::Twist,geometry_msgs::Twist, geometry_msgs::Twist)));
  // KCF追踪
//  QObject::connect(this,SIGNAL(trackerImageSignal(QImage)),this,SLOT(deal_trackerImageSignal(QImage)));
  QObject::connect(ui.graphicsView,SIGNAL(mouseMove_signal(QPoint)), this, SLOT(deal_mouseMove_signal(QPoint)));
  QObject::connect(ui.graphicsView,SIGNAL(mousePress_signal(QPoint)), this, SLOT(deal_mousePress_signal(QPoint)));
  QObject::connect(ui.graphicsView,SIGNAL(mouseRelease_signal(QPoint)), this, SLOT(deal_mouseRelease_signal(QPoint)));
  QObject::connect(&imageStitching,SIGNAL(trackStitchingImageSignal(cv::Mat)), &trackerThread, SLOT(deal_trackStitchingImageSignal(cv::Mat)));

  QObject::connect(&trackerThread,SIGNAL(trackFinishSignal(bool)), this, SLOT(deal_trackFinishSignal(bool)));

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
    // 记录是否已经起飞
    takeoff_flag = false;

    // 开启线程
    imageStitching.start();
    uavControl.start();
    trackerThread.start();
    MyTimer->start(100);  //  100Ms定时器

    // 加载图片
    ui.stitchingImage_label->setPixmap(QPixmap(":/images/load1.png"));
    ui.uav1Image_label->setPixmap(QPixmap(":/images/load.png"));
    ui.uav2Image_label->setPixmap(QPixmap(":/images/load.png"));
    ui.uav3Image_label->setPixmap(QPixmap(":/images/load.png"));
    ui.school_label->setPixmap(QPixmap(":/images/school.png"));

    ui.graphicsView->setStyleSheet("background: transparent");
    // 关闭滑动条
    ui.graphicsView->setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
    ui.graphicsView->setVerticalScrollBarPolicy(Qt::ScrollBarAlwaysOff);

    scene = new QGraphicsScene(this);
    // 重置坐标系原点为左上角
    scene->setSceneRect(0,0,ui.graphicsView->width()-1,ui.graphicsView->height()-1);

    RectItem = new QGraphicsRectItem();

    QPen Polygonpen;
    Polygonpen.setWidth(1);
    Polygonpen.setColor(Qt::red);
    RectItem->setPen(Polygonpen);
    QBrush Polygon_Brush;
    Polygon_Brush.setColor(QColor(177,177,177));
    RectItem->setBrush(Polygon_Brush);
    RectItem->setPos(0,0);
    scene->addItem(RectItem);
    ui.graphicsView->setScene(scene);
    ui.graphicsView->show();

    ros::init(argc,argv,"main_node");
    if ( ! ros::master::check() )
    {
      std::string str = "gnome-terminal --window -e 'bash -c \"roscore; exec bash\"'&";
      const char *command = str.c_str();
      system(command);
    }


}

MainWindow::~MainWindow() {}

void MainWindow::closeEvent(QCloseEvent *event)
{

  uav1Node.isRun = false;
  uav2Node.isRun = false;
  uav3Node.isRun = false;

  moveUav2.close();
  moveUav1.close();
  moveUav3.close();

  uavShow.close();
//  trackerWindow.close();
  /* 关闭图像拼接进程 */
  imageStitching.stitchingThreadStatue = false;
  imageStitching.isStitching = false;
  imageStitching.imageRecOK.wakeAll();
  imageStitching.quit();
  imageStitching.wait();

  uavControl.autoFlyThreadStatue = false;
  uavControl.isAutoFly = false;
  uavControl.quit();
  uavControl.wait();

  trackerThread.trackerThreadStatus = false;
  trackerThread.quit();
  trackerThread.wait();
  MyTimer->stop();

}

/*****************************************************************************
** Implementation [Slots]
*****************************************************************************/

void MainWindow::showNoMasterMessage() {
	QMessageBox msgBox;
	msgBox.setText("Couldn't find the ros master.");
	msgBox.exec();


//    close();
}

void MainWindow::deal_timeout()
{
  QString currentYaw[3];
  if(uavControl.setYawOffset_ok)
  {
    currentYaw[0] = QString::number((uavControl.currentYaw[1] - uavControl.yawOffset[1]) * 180.0 / 3.1415926, 'g', 3);
    currentYaw[1] = QString::number((uavControl.currentYaw[2] - uavControl.yawOffset[2]) * 180.0 / 3.1415926, 'g', 3);
    currentYaw[2] = QString::number((uavControl.currentYaw[3] - uavControl.yawOffset[3]) * 180.0 / 3.1415926, 'g', 3);
  }
  else
  {
    currentYaw[0] = QString::number(uavControl.currentYaw[1] * 180.0 / 3.1415926,'g',3);
    currentYaw[1] = QString::number(uavControl.currentYaw[2] * 180.0 / 3.1415926,'g',3);
    currentYaw[2] = QString::number(uavControl.currentYaw[3] * 180.0 / 3.1415926,'g',3);
  }
  ui.yawOffset_uav1_lineEdit->setText(currentYaw[0]) ;
  ui.yawOffset_uav2_lineEdit->setText(currentYaw[1]) ;
  ui.yawOffset_uav3_lineEdit->setText(currentYaw[2]) ;

  // 仿真模拟界面
  Eigen::Matrix<double, 2, 3> uavPosition_global,uavPosition_body;
  Eigen::Matrix<double, 2, 2> uav_R;
  double offset;
  uavShow.uav1PolygonItem->setRotation(-(uavControl.currentYaw[1] - uavControl.yawOffset[1]) * 180 / uavControl.PI + 90);
  uavShow.uav2PolygonItem->setRotation(-(uavControl.currentYaw[2] - uavControl.yawOffset[2]) * 180 / uavControl.PI + 90);
  uavShow.uav3PolygonItem->setRotation(-(uavControl.currentYaw[3] - uavControl.yawOffset[3]) * 180 / uavControl.PI + 90);

  uavPosition_global << uavControl.currentPosition[1].x , uavControl.currentPosition[2].x , uavControl.currentPosition[3].x ,
                        uavControl.currentPosition[1].y , uavControl.currentPosition[2].y , uavControl.currentPosition[3].y;


  uav_R << cos(uavControl.yawOffset[2]), sin(uavControl.yawOffset[2]), -sin(uavControl.yawOffset[2]), cos(uavControl.yawOffset[2]);
  uavPosition_body = uav_R * uavPosition_global;

  uavShow.uav1PolygonItem->setPos(uavPosition_body(0,0)*50, -uavPosition_body(1,0)*50);
  uavShow.uav2PolygonItem->setPos(uavPosition_body(0,1)*50, -uavPosition_body(1,1)*50);
  uavShow.uav3PolygonItem->setPos(uavPosition_body(0,2)*50, -uavPosition_body(1,2)*50);


  QString  qstr;
  qstr = QString::number(uavPosition_body(0,0),'g',3) + " , " + QString::number(uavPosition_body(1,0),'g',3);
  uavShow.uav1PositionTextItem->setText(qstr);
  qstr = QString::number(uavControl.currentPosition[2].x,'g',3) + " , " + QString::number(uavControl.currentPosition[2].y,'g',3);
  uavShow.uav2PositionTextItem->setText(qstr);
  qstr = QString::number(uavPosition_body(0,2),'g',3) + " , " + QString::number(uavPosition_body(1,2),'g',3);
  uavShow.uav3PositionTextItem->setText(qstr);

  offset = uavShow.uav1PositionTextItem->text().length() * 2.5;
  uavShow.uav1PositionTextItem->setPos(uavPosition_body(0,0)*50 - 10, -uavPosition_body(1,0)*50 - offset);
  offset = uavShow.uav2PositionTextItem->text().length() * 2.5;
  uavShow.uav2PositionTextItem->setPos(uavPosition_body(0,1)*50 - 10, -uavPosition_body(1,1)*50 - offset);
  offset = uavShow.uav3PositionTextItem->text().length() * 2.5;
  uavShow.uav3PositionTextItem->setPos(uavPosition_body(0,2)*50 - 10, -uavPosition_body(1,2)*50 - offset);


//  uavShow.uav1PolygonItem->setPos(uavControl.currentPosition[1].x*50, -uavControl.currentPosition[1].y*50);
//  uavShow.uav1PolygonItem->setRotation((-uavControl.currentYaw[1] ) * 180 / uavControl.PI + 90);
//  uavShow.uav2PolygonItem->setPos(uavControl.currentPosition[2].x*50, -uavControl.currentPosition[2].y*50);
//  uavShow.uav2PolygonItem->setRotation((-uavControl.currentYaw[2]) * 180 / uavControl.PI + 90);
//  uavShow.uav3PolygonItem->setPos(uavControl.currentPosition[3].x*50, -uavControl.currentPosition[3].y*50);
//  uavShow.uav3PolygonItem->setRotation((-uavControl.currentYaw[3]) * 180 / uavControl.PI + 90);

//  QString  qstr;
//  qstr = QString::number(uavControl.currentPosition[1].x,'g',3) + " , " + QString::number(uavControl.currentPosition[1].y,'g',3);
//  uavShow.uav1PositionTextItem->setText(qstr);
//  qstr = QString::number(uavControl.currentPosition[2].x,'g',3) + " , " + QString::number(uavControl.currentPosition[2].y,'g',3);
//  uavShow.uav2PositionTextItem->setText(qstr);
//  qstr = QString::number(uavControl.currentPosition[3].x,'g',3) + " , " + QString::number(uavControl.currentPosition[3].y,'g',3);
//  uavShow.uav3PositionTextItem->setText(qstr);

//  uavShow.uav1PositionTextItem->setPos(uavControl.currentPosition[1].x*50 - 10, -uavControl.currentPosition[1].y*50 - 10);
//  uavShow.uav2PositionTextItem->setPos(uavControl.currentPosition[2].x*50 - 10, -uavControl.currentPosition[2].y*50 - 10);
//  uavShow.uav3PositionTextItem->setPos(uavControl.currentPosition[3].x*50 - 10, -uavControl.currentPosition[3].y*50 - 10);

}

void MainWindow::displayStitchingImage(const QImage image)
{
  stitchingImage_mutex_.lock();
  stitchingImage = image.copy();
  ui.stitchingImage_label->setPixmap(QPixmap::fromImage(stitchingImage));
  stitchingImage_mutex_.unlock();

  //显示重合度
//  QString overlap = QString::number(imageStitching.overlap_rate_left, 'g', 3);
  QString overlap = QString::number(uavControl.currentOverlap_left[10] , 'g', 3);
  overlap = overlap + "%";
  ui.overlapRate_left_lineEdit->setText(overlap) ;
//  overlap = QString::number(imageStitching.overlap_rate_right, 'g', 3);
  overlap = QString::number(uavControl.currentOverlap_right[10], 'g', 3);
  overlap = overlap + "%";
  ui.overlapRate_right_lineEdit->setText(overlap) ;

  uavControl.stitchingErr_left = imageStitching.stitchingErr_left;
  uavControl.stitchingErr_right = imageStitching.stitchingErr_right;

  //主界面 拼接状态指示
  if(imageStitching.stitchingErr_left)
    ui.stitchingState_left_label->setStyleSheet("background: rgb(255,0,0)");
  else
    ui.stitchingState_left_label->setStyleSheet("background: rgb(0,255,0)");

  if(imageStitching.stitchingErr_right)
    ui.stitchingState_right_label->setStyleSheet("background: rgb(255,0,0)");
  else
    ui.stitchingState_right_label->setStyleSheet("background: rgb(0,255,0)");

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

// 控制无人机的移动
void MainWindow::deal_uavTargetVelocitySignal(geometry_msgs::Twist uav1TargetVelocity, geometry_msgs::Twist uav2TargetVelocity,geometry_msgs::Twist uav3TargetVelocity)
{
  // UAV1
  if(uavControl.is_manualControl[1] == true)
     uav1Node.moveControl();
  else
  {
    if(uav1TargetVelocity.angular.z >= 0.02 || uav1TargetVelocity.angular.z <= -0.02)
      uav1Node.cmd(0, 0, 0, uav1TargetVelocity.angular.z);
    else
      uav1Node.cmd(uav1TargetVelocity.linear.x, uav1TargetVelocity.linear.y, uav1TargetVelocity.linear.z, uav1TargetVelocity.angular.z);
  }
  // UAV3
  if(uavControl.is_manualControl[3] == true)
     uav3Node.moveControl();
  else
  {
    if(uav3TargetVelocity.angular.z >= 0.02 || uav3TargetVelocity.angular.z <= -0.02)
      uav3Node.cmd(0, 0, 0, uav3TargetVelocity.angular.z);
    else
      uav3Node.cmd(uav3TargetVelocity.linear.x, uav3TargetVelocity.linear.y, uav3TargetVelocity.linear.z, uav3TargetVelocity.angular.z);
  }

  // UAV2
//  if(uavControl.is_manualControl[2] == true)
//     uav2Node.moveControl();
//  else
//  {
//      uav2Node.cmd(uav2TargetVelocity.linear.x, uav2TargetVelocity.linear.y, uav2TargetVelocity.linear.z, 0);
//  }

  // 移动相机
  if(uavControl.is_cameraControl == true)
  {
    uav1Node.cameraControl(20,uavControl.rotationalAngle_left);
    uav3Node.cameraControl(20,uavControl.rotationalAngle_right);
  }
}

void  MainWindow::deal_rosShutdown(int UAVx)
{
  if(UAVx == 1)
  {

    if(uav1Node.isRunning())
    {
      uav1Node.quit();
      uav1Node.wait();
    }
    ui.uav1Land_pBtn->setEnabled(false);
    ui.uav1Takeoff_pBtn->setEnabled(false);
    ui.uav1Move_pBtn->setEnabled(false);
    ui.uav1Battery_pBar->setEnabled(false);
  }
  else if(UAVx == 2)
  {
    if(uav2Node.isRunning())
    {
      uav2Node.quit();
      uav2Node.wait();
    }
    ui.uav2Land_pBtn->setEnabled(false);
    ui.uav2Takeoff_pBtn->setEnabled(false);
    ui.uav2Move_pBtn->setEnabled(false);
    ui.uav2Battery_pBar->setEnabled(false);
  }
  else if(UAVx == 3)
  {
    if(uav3Node.isRunning())
    {
      uav3Node.quit();
      uav3Node.wait();
    }
    ui.uav3Land_pBtn->setEnabled(false);
    ui.uav3Takeoff_pBtn->setEnabled(false);
    ui.uav3Move_pBtn->setEnabled(false);
    ui.uav3Battery_pBar->setEnabled(false);
  }
  if(uavControl.isRunning())
  {
    uavControl.autoFlyThreadStatue = false;
    uavControl.quit();
    uavControl.wait();
    ui.autoFly_pBtn->setText(QString::fromUtf8("自主飞行"));
  }

}

/* ******************************************************  无人机1 ******************************************************* */

void MainWindow::displayUav1Image(const QImage image)
{
  uav1Image_mutex_.lock();
//  uav1Image = image.copy();
  ui.uav1Image_label->setPixmap(QPixmap::fromImage(uav1Image));
//  ui.uav1Image_label->resize(ui.uav1Image_label->pixmap()->size());
  uav1Image_mutex_.unlock();

}
void MainWindow::deal_uav1RgbimageSignal(cv::Mat rgbimage)
{
  try
  {
      cvtColor(rgbimage, imageStitching.leftImage,CV_RGB2BGR);

//      rgbimage.copyTo(imageStitching.leftImage);// 深拷贝 完全复制一份
//      imageStitching.leftImage = rgbimage;        // 速度快 矩阵指针指向同一地址而实现 共享同一个矩阵
      imageStitching.leftImageRec_flag = true;
      uav1Image = QImage(rgbimage.data,rgbimage.cols,rgbimage.rows,rgbimage.step[0], QImage::Format_RGB888);
      displayUav1Image(uav1Image);
  }
  catch (cv_bridge::Exception& e)
  {
      std::cout << "deal_uav1RgbimageSignal err " << std::endl;
  }
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
  // 关闭自动控制功能
  uavControl.isAutoFly = false;
  if(uavControl.isRunning())
  {
    uavControl.autoFlyThreadStatue = false;
    uavControl.quit();
    uavControl.wait();
    ui.autoFly_pBtn->setText(QString::fromUtf8("自主飞行"));
  }
}

void MainWindow::on_uav1Connect_pBtn_clicked()
{
  uav1Name = ui.uav1Name_cBox->currentText().toStdString();
// window -e 'bash -c "roslaunch bebop_driver bebop_node.launch"'
#if runCommand
  std::string str = "gnome-terminal --window -e 'bash -c \"source /home/linux/work/driver/bebop_ws/devel/setup.bash; roslaunch bebop_driver "
                        + uav1Name
                        +"_node.launch; exec bash\"'&";
  const char *command = str.c_str();
  system(command);
#endif

}

void MainWindow::on_uav1Move_pBtn_clicked()
{
  moveUav1.show();
  moveUav1.move(1500,10);
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
  }

}

/* *******************************************************  无人机2 ******************************************************* */

void MainWindow::deal_uav2RgbimageSignal(cv::Mat rgbimage)
{
  try
  {
      cvtColor(rgbimage, imageStitching.middleImage,CV_RGB2BGR);
      imageStitching.middleImageRec_flag = true;
      uav2Image = QImage(rgbimage.data,rgbimage.cols,rgbimage.rows,rgbimage.step[0], QImage::Format_RGB888);
      displayUav2Image(uav2Image);
  }
  catch (cv_bridge::Exception& e)
  {
      std::cout << "deal_uav2RgbimageSignal error !!! " << std::endl;
  }
}

void MainWindow::displayUav2Image(const QImage image)
{
  uav2Image_mutex_.lock();
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
  // 关闭自动控制功能
  uavControl.isAutoFly = false;
  if(uavControl.isRunning())
  {
    uavControl.autoFlyThreadStatue = false;
    uavControl.quit();
    uavControl.wait();
    ui.autoFly_pBtn->setText(QString::fromUtf8("自主飞行"));
  }
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
  moveUav2.show();
  moveUav2.move(1500,380);

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

void MainWindow::deal_uav3RgbimageSignal(cv::Mat rgbimage)
{
  try
  {
      cvtColor(rgbimage, imageStitching.rightImage,CV_RGB2BGR);
      imageStitching.rightImageRec_flag = true;
      uav3Image = QImage(rgbimage.data,rgbimage.cols,rgbimage.rows,rgbimage.step[0], QImage::Format_RGB888);
      displayUav3Image(uav3Image);
  }
  catch (cv_bridge::Exception& e)
  {
      std::cout << "deal_uav3RgbimageSignal error !!! " << std::endl;
  }
}

void MainWindow::displayUav3Image(const QImage image)
{
  uav3Image_mutex_.lock();
//  uav3Image = image.copy();
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
  uavControl.isAutoFly = false;
  if(uavControl.isRunning())
  {
    uavControl.autoFlyThreadStatue = false;
    uavControl.quit();
    uavControl.wait();
    ui.autoFly_pBtn->setText(QString::fromUtf8("自主飞行"));
  }
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
  moveUav3.show();
  moveUav3.move(1500,720);
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
  if(state)
    uavControl.is_manualControl[UAVx] = true;
  else
    uavControl.is_manualControl[UAVx] = false;
  if(UAVx == 1)
    uav1Node.forward = state;
  else if(UAVx == 2)
    uav2Node.forward = state;
  else if(UAVx == 3)
    uav3Node.forward = state;
}
void MainWindow::deal_backwardSignal(int UAVx, bool state)
{
  if(state)
    uavControl.is_manualControl[UAVx] = true;
  else
    uavControl.is_manualControl[UAVx] = false;
  if(UAVx == 1)
    uav1Node.backward = state;
  else if(UAVx == 2)
    uav2Node.backward = state;
  else if(UAVx == 3)
    uav3Node.backward = state;
}
void MainWindow::deal_flayLeftSignal(int UAVx, bool state)
{
  if(state)
    uavControl.is_manualControl[UAVx] = true;
  else
    uavControl.is_manualControl[UAVx] = false;
  if(UAVx == 1)
    uav1Node.flayLeft = state;
  else if(UAVx == 2)
    uav2Node.flayLeft = state;
  else if(UAVx == 3)
    uav3Node.flayLeft = state;
}
void MainWindow::deal_flayRightSignal(int UAVx, bool state)
{
  if(state)
    uavControl.is_manualControl[UAVx] = true;
  else
    uavControl.is_manualControl[UAVx] = false;
  if(UAVx == 1)
    uav1Node.flayRight = state;
  else if(UAVx == 2)
    uav2Node.flayRight = state;
  else if(UAVx == 3)
    uav3Node.flayRight = state;
}
void MainWindow::deal_flayUpSignal(int UAVx, bool state)
{
  if(state)
    uavControl.is_manualControl[UAVx] = true;
  else
    uavControl.is_manualControl[UAVx] = false;
  if(UAVx == 1)
    uav1Node.flayUp = state;
  else if(UAVx == 2)
    uav2Node.flayUp = state;
  else if(UAVx == 3)
    uav3Node.flayUp = state;
}
void MainWindow::deal_flayDownSignal(int UAVx, bool state)
{
  if(state)
    uavControl.is_manualControl[UAVx] = true;
  else
    uavControl.is_manualControl[UAVx] = false;
  if(UAVx == 1)
    uav1Node.flayDown = state;
  else if(UAVx == 2)
    uav2Node.flayDown = state;
  else if(UAVx == 3)
    uav3Node.flayDown = state;
}
void MainWindow::deal_turnLeftSignal(int UAVx, bool state)
{
  if(state)
    uavControl.is_manualControl[UAVx] = true;
  else
    uavControl.is_manualControl[UAVx] = false;
  if(UAVx == 1)
    uav1Node.turnLeft = state;
  else if(UAVx == 2)
    uav2Node.turnLeft = state;
  else if(UAVx == 3)
    uav3Node.turnLeft = state;
}
void MainWindow::deal_turnRightSignal(int UAVx, bool state)
{
  if(state)
    uavControl.is_manualControl[UAVx] = true;
  else
    uavControl.is_manualControl[UAVx] = false;
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
      imageStitching.isStitching = true;
//      uavControl.isStitching = true;
    }
    else
    {
      imageStitching.isStitching = false;
      uavControl.isStitching = false;
      uavControl.is_cameraControl = false;
      ui.imageControl_checkBox->setCheckState(Unchecked);
      ui.cameraControl_checkBox->setCheckState(Unchecked);
    }
}
// 自动飞行
void MainWindow::on_autoFly_pBtn_clicked()
{
  if(uavControl.isAutoFly == false)
  {
    if(uavControl.setYawOffset_ok == false)
    {
      QMessageBox::warning(NULL , QString::fromUtf8("提示"), QString::fromUtf8("请先设定YAW偏差！ "));
      return;
    }
    uavControl.isAutoFly = true;
    ui.autoFly_pBtn->setText(QString::fromUtf8("切换手动模式"));

    if(uavControl.isFinished())
    {
      uavControl.autoFlyThreadStatue = true;
      uavControl.start();
    }
  }
  else
  {
    uavControl.isAutoFly = false;
    if(uavControl.isRunning())
    {
      uavControl.autoFlyThreadStatue = false;
      uavControl.quit();
      uavControl.wait();
      ui.autoFly_pBtn->setText(QString::fromUtf8("自主飞行"));
    }
  }
}

void MainWindow::on_setYawErr_pBtn_clicked()
{
  if(!uavControl.setYawOffset_ok)
  {
    uavControl.yawOffset[1] = uavControl.currentYaw[1];
    uavControl.yawOffset[2] = uavControl.currentYaw[2];
    uavControl.yawOffset[3] = uavControl.currentYaw[3];
    uavControl.yawOffset_21 = uavControl.yawOffset[2] - uavControl.yawOffset[1];
    if(abs(uavControl.yawOffset_21) > uavControl.PI)
    {
      if(uavControl.yawOffset_21 > 0)
        uavControl.yawOffset_21 = uavControl.yawOffset_21 - 2*uavControl.PI;
      else
        uavControl.yawOffset_21 = uavControl.yawOffset_21 + 2*uavControl.PI;
    }
    uavControl.yawOffset_23 = uavControl.yawOffset[2] - uavControl.yawOffset[3];
    if(abs(uavControl.yawOffset_23) > uavControl.PI)
    {
      if(uavControl.yawOffset_23 > 0)
        uavControl.yawOffset_23 = uavControl.yawOffset_23 - 2*uavControl.PI;
      else
        uavControl.yawOffset_23 = uavControl.yawOffset_23 + 2*uavControl.PI;
    }
    ui.setYawErr_pBtn->setText(QString::fromUtf8("已设定"));
//    ui.setYawErr_pBtn->setStyleSheet("color:rgb(255,0,0)");
//    背景色可以用函数setStyleSheet("background: rgb(0,255,0));
    uavControl.setYawOffset_ok = true;
  }
  else
  {
    ui.setYawErr_pBtn->setText(QString::fromUtf8("设定YAW偏差"));
    uavControl.setYawOffset_ok = false;
//    ui.setYawErr_pBtn->setStyleSheet("color:rgb(0, 255, 0)");

  }
}
void MainWindow::on_setOverlapRate_pBtn_clicked()
{
  uavControl.targetOverlap_left = ui.setOverlapRate_left_lineEdit->text().toDouble();
  uavControl.targetOverlap_right = ui.setOverlapRate_right_lineEdit->text().toDouble();
//  std::cout << "uavControl.targetOverlap_right  " <<  uavControl.targetOverlap_right << std::endl;
}
void MainWindow::on_track_pBtn_clicked()
{
  if(trackerThread.is_tracking == false)
  {
    if(trackerThread.trackRoi.width <= 1 || trackerThread.trackRoi.height <= 1)
    {
      QMessageBox::warning(NULL , QString::fromUtf8("提示"), QString::fromUtf8("选择区域太小！ "));
      return;
    }
    // 清除框选
    rect.setRect(0,0,1,1);
    RectItem->setRect(rect);
    trackerThread.is_tracking = true;
    trackerThread.trackerInit_flag = false;
    ui.track_pBtn->setText(QString::fromUtf8("停止追踪"));
    QObject::disconnect(ui.graphicsView,SIGNAL(mouseMove_signal(QPoint)), this, SLOT(deal_mouseMove_signal(QPoint)));
    QObject::disconnect(ui.graphicsView,SIGNAL(mousePress_signal(QPoint)), this, SLOT(deal_mousePress_signal(QPoint)));
    QObject::disconnect(ui.graphicsView,SIGNAL(mouseRelease_signal(QPoint)), this, SLOT(deal_mouseRelease_signal(QPoint)));

  }
  else {
    QObject::connect(ui.graphicsView,SIGNAL(mouseMove_signal(QPoint)), this, SLOT(deal_mouseMove_signal(QPoint)));
    QObject::connect(ui.graphicsView,SIGNAL(mousePress_signal(QPoint)), this, SLOT(deal_mousePress_signal(QPoint)));
    QObject::connect(ui.graphicsView,SIGNAL(mouseRelease_signal(QPoint)), this, SLOT(deal_mouseRelease_signal(QPoint)));
    trackerThread.is_tracking = false;
    trackerThread.trackRoi.width = 0;
    trackerThread.trackRoi.height = 0;
    ui.track_pBtn->setText(QString::fromUtf8("框选追踪"));

  }

}

void MainWindow::deal_trackFinishSignal(bool trackStatue)
{
  displayStitchingImage(trackerThread.showImage);
}

void MainWindow::deal_mouseMove_signal(QPoint point)
{
  //  显示view下的坐标
//  ui.view_lineEdit->setText(QString("%1, %2").arg(point.x()).arg(point.x()));
//  QPointF pointScene=ui.graphicsView ->mapToScene(point);
//  ui.scene_lineEdit->setText(QString("%1, %2").arg(pointScene.x()).arg(pointScene.x()));
  if(trackerThread.mousePress)
  {
     double err_x,err_y;
     err_x = point.x() - trackerThread.mousePosition_start.x();
     err_y = point.y() - trackerThread.mousePosition_start.y();
     rect.setRect(trackerThread.mousePosition_start.x(),trackerThread.mousePosition_start.y(),err_x,err_y);

    RectItem->setRect(rect);
    trackerThread.mousePosition_current = point;
  }
}
void MainWindow::deal_mousePress_signal(QPoint point)
{

  trackerThread.mousePosition_start = point;
  trackerThread.mousePress = true;

  trackerThread.trackRoi.x = point.x() * trackerThread.scale_x;
  trackerThread.trackRoi.y = point.y() * trackerThread.scale_y;
}
void MainWindow::deal_mouseRelease_signal(QPoint point)
{
  trackerThread.mousePosition_current = point;
  trackerThread.mousePress = false;
  trackerThread.trackRoi.width = (point.x() - trackerThread.mousePosition_start.x()) * trackerThread.scale_x;
  trackerThread.trackRoi.height = (point.y() - trackerThread.mousePosition_start.y()) * trackerThread.scale_y;

}

void MainWindow::on_takeoff_pBtn_clicked()
{
    if(takeoff_flag == false)
    {
      takeoff_flag = true;
      uav1Node.takeoff();
      uav2Node.takeoff();
      uav3Node.takeoff();
      ui.takeoff_pBtn->setText(QString::fromUtf8("一键降落"));
    }
    else
    {
      takeoff_flag = false;

      // 关闭自动控制功能
      uavControl.isAutoFly = false;
      if(uavControl.isRunning())
      {
        uavControl.autoFlyThreadStatue = false;
        uavControl.quit();
        uavControl.wait();
        ui.autoFly_pBtn->setText(QString::fromUtf8("自主飞行"));
      }

      uav1Node.land();
      uav2Node.land();
      uav3Node.land();
      ui.takeoff_pBtn->setText(QString::fromUtf8("一键起飞"));

    }
}

void MainWindow::on_imageControl_checkBox_stateChanged(int arg1)
{
  if(ui.imageControl_checkBox->isChecked() == true)
  {
    if(imageStitching.isStitching == true)
    {
      uavControl.isStitching = true;        //根据拼接效果调整队形
      uavControl.is_cameraControl = false;  //根据拼接效果调整相机角度
      ui.cameraControl_checkBox->setCheckState(Unchecked);

      //恢复相机原始角度
      uavControl.rotationalAngle_left = -0;
      uavControl.rotationalAngle_right = 0;
      uav1Node.cameraControl(20,-5);
      uav3Node.cameraControl(20,5);
    }
    else
      ui.imageControl_checkBox->setCheckState(Unchecked);
  }
  else
  {
    uavControl.isStitching = false;
  }
}
void MainWindow::on_cameraControl_checkBox_stateChanged(int arg1)
{
  if(ui.cameraControl_checkBox->isChecked() == true)
  {
    if(imageStitching.isStitching == true)
    {
      uavControl.is_cameraControl = true;
      uavControl.isStitching = false;
      ui.imageControl_checkBox->setCheckState(Unchecked);
    }
    else
      ui.cameraControl_checkBox->setCheckState(Unchecked);
  }
  else
  {
    uavControl.is_cameraControl = false;
  }
}

}  // namespace image_stitching

