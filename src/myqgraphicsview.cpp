#include "../include/image_stitching/myqgraphicsview.hpp"
#include <iostream>
myQGraphicsView::myQGraphicsView(QWidget *parent) : QGraphicsView(parent)
{

}

void myQGraphicsView::mouseMoveEvent(QMouseEvent *event)
{
    QPoint point = event->pos();
    Q_EMIT mouseMove_signal(point);
}

void myQGraphicsView::wheelEvent(QWheelEvent *event)
{
    QPoint point = event->pos();
    int delta = event->delta();
    Q_EMIT mouseWheel_signal(point,delta);
}
void myQGraphicsView::mousePressEvent(QMouseEvent *event)
{ //鼠标左键按下事件
    if (event->button()==Qt::LeftButton)
    {
        QPoint point=event->pos(); //QGraphicsView的坐标
        Q_EMIT mousePress_signal(point);//释放信号
//        std::cout << "3333333333333" << std::endl;
    }
    QGraphicsView::mousePressEvent(event);
}
void myQGraphicsView::mouseReleaseEvent(QMouseEvent *event)
{
  if (event->button()==Qt::LeftButton)
  {
      QPoint point=event->pos(); //QGraphicsView的坐标
      Q_EMIT mouseRelease_signal(point);//释放信号
//      std::cout << "444444444444444" << std::endl;
  }
  QGraphicsView::mouseReleaseEvent(event);
}
