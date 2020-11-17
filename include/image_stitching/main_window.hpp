/**
 * @file /include/image_stitching/main_window.hpp
 *
 * @brief Qt based gui for image_stitching.
 *
 * @date November 2010
 **/
#ifndef image_stitching_MAIN_WINDOW_H
#define image_stitching_MAIN_WINDOW_H

/*****************************************************************************
** Includes
*****************************************************************************/

#include <QtGui/QMainWindow>
#include "ui_main_window.h"
#include <QMutex>
#include <QWaitCondition>
#include <QTimer>
#include <QTime>

#include <QGraphicsView>
#include <QMouseEvent>
#include "uav1_node.hpp"
#include "uav2_node.hpp"
#include "uav3_node.hpp"
#include "moveuav_window.hpp"
#include "kcf/trackerThread.hpp"
#include "stitching.hpp"
#include "uav_control.hpp"
#include "uavpositionshow.hpp"

namespace image_stitching {

/*****************************************************************************
** Interface [MainWindow]
*****************************************************************************/
/**
 * @brief Qt central, all operations relating to the view part here.
 */
class MainWindow : public QMainWindow {
Q_OBJECT

public:
  MainWindow(int argc, char** argv, QWidget *parent = 0);
	~MainWindow();

  uav1 uav1Node;
  uav2 uav2Node;
  uav3 uav3Node;
  moveUav moveUav1;
  moveUav moveUav2;
  moveUav moveUav3;
  stitching imageStitching;
  uav_control uavControl;
  tracker_thread trackerThread;
  uavPositionshow  uavShow;

  QTimer *MyTimer;

  QImage uav1Image;
  QImage uav2Image;
  QImage uav3Image;

  QImage stitchingImage;
  std::string uav1Name;
  std::string uav2Name;
  std::string uav3Name;

  QGraphicsScene  *scene;
  QGraphicsRectItem *RectItem;   //多边形
  QRectF rect; //（x，y）为左上角并具有给定width和height的矩形

  // 一键起飞、降落标志位
  bool takeoff_flag;
  /*用来保护一个对象、数据结构、代码段、使得它们在同一一时刻，只有一个线程有访问权限*/
  mutable QMutex uav1Image_mutex_;
  mutable QMutex uav2Image_mutex_;
  mutable QMutex uav3Image_mutex_;
  mutable QMutex stitchingImage_mutex_;

	void closeEvent(QCloseEvent *event); // Overloaded function
	void showNoMasterMessage();

  void displayUav1Image(const QImage image);
  void displayUav2Image(const QImage image);
  void displayUav3Image(const QImage image);
  void displayStitchingImage(const QImage image);

Q_SIGNALS:
  void trackerImageSignal(QImage);

public Q_SLOTS:
	/******************************************
	** Auto-connections (connectSlotsByName())
	*******************************************/

  // 接受电池电量信息槽函数
  void deal_uav1batteryDataSignal(int batteryData, bool batteryState);
  void deal_uav2batteryDataSignal(int batteryData, bool batteryState);
  void deal_uav3batteryDataSignal(int batteryData, bool batteryState);
  // 接受图像槽函数
  void deal_uav1RgbimageSignal(cv::Mat rgbimage);
  void deal_uav2RgbimageSignal(cv::Mat rgbimage);
  void deal_uav3RgbimageSignal(cv::Mat rgbimage);
//  void deal_showStitchingImageSignal(QImage image);
  void deal_trackFinishSignal(bool trackStatue);


//  void deal_uav3gpsDataSignal(double latitude, double longitude);
//  void deal_uav3gpsDataSignal(double latitude, double longitude);
//  void deal_uav2gpsDataSignal(double latitude, double longitude);
//  void deal_uav3gpsDataSignal(double latitude, double longitude);

  void deal_forwardSignal(int UAVx, bool state);
  void deal_backwardSignal(int UAVx, bool state);
  void deal_flayLeftSignal(int UAVx, bool state);
  void deal_flayRightSignal(int UAVx, bool state);
  void deal_flayUpSignal(int UAVx, bool state);
  void deal_flayDownSignal(int UAVx, bool state);
  void deal_turnLeftSignal(int UAVx, bool state);
  void deal_turnRightSignal(int UAVx, bool state);

  void deal_uavTargetVelocitySignal(geometry_msgs::Twist uav1TargetVelocity,geometry_msgs::Twist uav2TargetVelocity, geometry_msgs::Twist uav3TargetVelocity);
  void deal_cameraControSignal(int UAVx, double vertical, double horizontal);

  void deal_rosShutdown(int UAVx);
  void deal_timeout();

  //  鼠标信号
  void deal_mouseMove_signal(QPoint point);
  void deal_mousePress_signal(QPoint point); //鼠标单击
  void deal_mouseRelease_signal(QPoint point);

  void on_uav1Takeoff_pBtn_clicked();
  void on_uav1Land_pBtn_clicked();
  void on_uav1Connect_pBtn_clicked();
  void on_uav1Move_pBtn_clicked();
  void on_uav1ShowImage_pBtn_clicked();

  void on_uav2Takeoff_pBtn_clicked();
  void on_uav2Land_pBtn_clicked();
  void on_uav2Connect_pBtn_clicked();
  void on_uav2Move_pBtn_clicked();
  void on_uav2ShowImage_pBtn_clicked();

  void on_uav3Takeoff_pBtn_clicked();
  void on_uav3Land_pBtn_clicked();
  void on_uav3Connect_pBtn_clicked();
  void on_uav3Move_pBtn_clicked();
  void on_uav3ShowImage_pBtn_clicked();

  void on_stitching_checkBox_stateChanged(int arg1);

  void on_autoFly_pBtn_clicked();

  void on_setYawErr_pBtn_clicked();

  void on_track_pBtn_clicked();

  void on_setOverlapRate_pBtn_clicked();

  void on_takeoff_pBtn_clicked();

  void on_imageControl_checkBox_stateChanged(int arg1);
private:
	Ui::MainWindowDesign ui;

protected:
//  void paintEvent(QPaintEvent *);


};

}  // namespace image_stitching

#endif // image_stitching_MAIN_WINDOW_H
