#ifndef KCFTRACKER_WINDOW_H
#define KCFTRACKER_WINDOW_H

#include <QWidget>
#include <QGraphicsView>
#include <QMouseEvent>

namespace Ui {
class kcfTracker_window;
}

class kcfTracker_window : public QWidget
{
  Q_OBJECT

public:
  explicit kcfTracker_window(QWidget *parent = nullptr);
  ~kcfTracker_window();
  Ui::kcfTracker_window *ui;

QGraphicsScene  *scene;
QPointF mousePosition_start,mousePosition_current;
//bool is

public Q_SLOTS:
  void deal_mousemove_signal(QPoint point);
  void deal_trackerImageSignal(QImage img);
//  void deal_mousewheel_signal(QPoint point,int delta);
//  void deal_mouseClicked(QPoint point);

private:

};

#endif // KCFTRACKER_WINDOW_H
