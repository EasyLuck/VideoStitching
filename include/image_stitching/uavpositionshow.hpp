#ifndef UAVPOSITIONSHOW_H
#define UAVPOSITIONSHOW_H

#include <QWidget>
#include <QGraphicsView>
#include <QPainter>
#include <QGraphicsEllipseItem>

namespace Ui {
class uavPositionshow;
}

class uavPositionshow : public QWidget
{
  Q_OBJECT

public:
  explicit uavPositionshow(QWidget *parent = nullptr);
  ~uavPositionshow();


  QGraphicsScene  *scene;
  QGraphicsPolygonItem *uav1PolygonItem;   //UAV1
  QGraphicsPolygonItem *uav2PolygonItem;   //UAV1
  QGraphicsPolygonItem *uav3PolygonItem;   //UAV1

  QGraphicsSimpleTextItem *uav1PositionTextItem;
  QGraphicsSimpleTextItem *uav2PositionTextItem;
  QGraphicsSimpleTextItem *uav3PositionTextItem;

  QRectF rect; //（x，y）为左上角并具有给定width和height的矩形



private:
  Ui::uavPositionshow *ui;
};

#endif // UAVPOSITIONSHOW_H
