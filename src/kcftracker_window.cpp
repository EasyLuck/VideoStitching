#include "../include/image_stitching/kcftracker_window.hpp"
#include "ui_kcftracker_window.h"
#include <QString>
#include <QGraphicsEllipseItem>
#include <iostream>
kcfTracker_window::kcfTracker_window(QWidget *parent) :
  QWidget(parent),
  ui(new Ui::kcfTracker_window)
{
  ui->setupUi(this);

  QObject::connect(ui->graphicsView,SIGNAL(mousemove_signal(QPoint)), this, SLOT(deal_mousemove_signal(QPoint)));

  scene = new QGraphicsScene;
  ui->graphicsView->setScene(scene);

}

kcfTracker_window::~kcfTracker_window()
{
  delete ui;
}



/*
  * 处理鼠标移动槽函数
  * 显示鼠标坐标
*/
void kcfTracker_window::deal_mousemove_signal(QPoint point)
{
  //  显示view下的坐标
  ui->view_lineEdit->setText(QString("%1, %2").arg(point.x()).arg(point.x()));

}
void kcfTracker_window::deal_trackerImageSignal(QImage img)
{
//  img = img.scaled(ui->graphicsView->width(),ui->graphicsView->height());
//  QGraphicsScene *scene = new QGraphicsScene;
  scene->addPixmap(QPixmap::fromImage(img));

//  ui->graphicsView->show();

//  free(scene);
//  delete scene;
//  scene = NULL;
//  std::cout << "11111111" <<std::endl;
}
