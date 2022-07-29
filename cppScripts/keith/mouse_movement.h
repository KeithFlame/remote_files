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
    //��д�麯��
    //����ƶ��¼�
    void mouseMoveEvent(QMouseEvent* ev);
    //��갴���¼�
    void mousePressEvent(QMouseEvent* ev);
    //����ͷ��¼�
    void mouseReleaseEvent(QMouseEvent* ev);
    ////����ͷ��¼�
    //void mouseReleaseRightEvent(QMouseEvent* ev);
    //
    //���봰������
    void enterEvent(QEvent*);
    //�뿪��������
    void leaveEvent(QEvent*);
signals:

public slots:

public:
    int mos_x, mos_y;
    bool left_clicked, right_clicked, is_range;
    

};

#endif // MOUSE_MOVEMENT_H_
