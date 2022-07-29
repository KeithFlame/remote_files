#pragma once
#ifndef MOUSE_MOVEMENT_H_
#define MOUSE_MOVEMENT_H_
#include <QLabel>
#include <QKeyEvent>
#include <iostream>

class MouseMovement : public QLabel
{
    Q_OBJECT
public:
    explicit MouseMovement(QWidget* parent = 0);

protected:
    //重写虚函数
    //鼠标移动事件
    void mouseMoveEvent(QMouseEvent* ev);
    //鼠标按下事件
    void mousePressEvent(QMouseEvent* ev);
    //鼠标释放事件
    void mouseReleaseEvent(QMouseEvent* ev);
    ////鼠标释放事件
    //void mouseReleaseRightEvent(QMouseEvent* ev);
    //
    //进入窗口区域
    void enterEvent(QEvent*);
    //离开窗口区域
    void leaveEvent(QEvent*);
signals:

public slots:

public:
    int mos_x, mos_y;
    bool left_clicked, right_clicked, is_range;
    

};

#endif // MOUSE_MOVEMENT_H_
