#include "../include/image_stitching/uavpositionshow.hpp"
#include "ui_uavpositionshow.h"
#include <iostream>
uavPositionshow::uavPositionshow(QWidget *parent) :
  QWidget(parent),
  ui(new Ui::uavPositionshow)
{
  ui->setupUi(this);

  scene = new QGraphicsScene(this);

  ui->graphicsView->rotate(-90);

  ui->graphicsView->setScene(scene);

  uav1PolygonItem = new QGraphicsPolygonItem();
  uav2PolygonItem = new QGraphicsPolygonItem();
  uav3PolygonItem = new QGraphicsPolygonItem();

  QPolygonF polygon;
  polygon << QPointF(0, -10) << QPointF(-5*1.1732, 5)
          << QPointF(5*1.1732, 5);
  uav1PolygonItem->setPolygon(polygon);
  uav2PolygonItem->setPolygon(polygon);
  uav3PolygonItem->setPolygon(polygon);

  QPen Polygonpen;
  Polygonpen.setWidth(1);
  Polygonpen.setColor(Qt::green);
  uav1PolygonItem->setPen(Polygonpen);
  uav2PolygonItem->setPen(Polygonpen);
  uav3PolygonItem->setPen(Polygonpen);

  uav1PolygonItem->setPos(0,75);
  uav2PolygonItem->setPos(0,0);
  uav3PolygonItem->setPos(0,-75);
  uav1PolygonItem->setRotation(90);
  uav2PolygonItem->setRotation(90);
  uav3PolygonItem->setRotation(90);

  uav1PositionTextItem = new QGraphicsSimpleTextItem;
  uav2PositionTextItem = new QGraphicsSimpleTextItem;
  uav3PositionTextItem = new QGraphicsSimpleTextItem;

  uav1PositionTextItem->setBrush(Qt::green);
  uav1PositionTextItem->setText("-1.5 , 0");
  double offset = uav1PositionTextItem->text().length();
  uav1PositionTextItem->setFont(QFont("华文琥珀",9));
  uav1PositionTextItem->setPos(-10,75-offset*2.5);

  uav2PositionTextItem->setBrush(Qt::green);
  uav2PositionTextItem->setText("0 , 0");
  uav2PositionTextItem->setFont(QFont("华文琥珀",9));
  offset = uav2PositionTextItem->text().length();
  uav2PositionTextItem->setPos(-10,-offset*2.5);

  uav3PositionTextItem->setBrush(Qt::green);
  uav3PositionTextItem->setText("1.5 , 0");
  uav3PositionTextItem->setFont(QFont("华文琥珀",9));
  offset = uav3PositionTextItem->text().length();
  uav3PositionTextItem->setPos(-10,-75-offset*2.5);

  uav1PositionTextItem->setRotation(90);
  uav2PositionTextItem->setRotation(90);
  uav3PositionTextItem->setRotation(90);

  scene->addItem(uav1PolygonItem);
  scene->addItem(uav2PolygonItem);
  scene->addItem(uav3PolygonItem);

  scene->addItem(uav1PositionTextItem);
  scene->addItem(uav2PositionTextItem);
  scene->addItem(uav3PositionTextItem);

  this->setWindowTitle("UAV POSITION SHOW");

}

uavPositionshow::~uavPositionshow()
{
  delete ui;
}
