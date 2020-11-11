#ifndef MYQGRAPHICSVIEW_H
#define MYQGRAPHICSVIEW_H

#include <QGraphicsView>
#include <QPoint>
#include <QMouseEvent>
#include <QWheelEvent>
#include <QScrollBar>

class myQGraphicsView : public QGraphicsView
{
    Q_OBJECT
public:
    explicit myQGraphicsView(QWidget *parent = nullptr);

protected:

Q_SIGNALS:
    void mouseMove_signal(QPoint point);
    void mouseWheel_signal(QPoint point, int delta);
    void mousePress_signal(QPoint point); //鼠标单击
    void mouseRelease_signal(QPoint point); //鼠标单击
public Q_SLOTS:
    void mouseMoveEvent(QMouseEvent *event);
    void wheelEvent(QWheelEvent *event);
    void mousePressEvent(QMouseEvent *event);
    void mouseReleaseEvent(QMouseEvent *event);
};

#endif // MYQGRAPHICSVIEW_H



