#include "mouse_movement.h"
#include <QMouseEvent>
#include <QLabel>
MouseMovement::MouseMovement(QWidget* parent) :
    QLabel(parent)
    , mos_x(0.f)
    , mos_y(0.f)
    , left_clicked(false)
    , right_clicked(false)
    , is_range(false)

{
    //����Ĭ��׷����꣬�����ڴ�������ƶ�ʱ�������ȵ�һ�²���Ч
    this->setMouseTracking(true);
}
//��д�麯��
//����ƶ�����
void MouseMovement::mouseMoveEvent(QMouseEvent* ev) {
    mos_x = ev->x();
    mos_y = ev->y();
    //QString text = QString("<center><h1>Mouse Move: (%1,%2)</h1></center>").arg(i).arg(j);
    //this->setText(text);
}
//��갴�´���
void MouseMovement::mousePressEvent(QMouseEvent* ev) {
    //��������������
    if (ev->button() == Qt::LeftButton) {
        int i = ev->x();
        int j = ev->y();
        QString text = QString("<center><h1>Mouse Left Press: (%1,%2)</h1></center>").arg(i).arg(j);
        this->setText(text);
        right_clicked = false;
        left_clicked = true;
    }
    else if(ev->button() == Qt::RightButton)
    {
        right_clicked = true;
        left_clicked = false;
        QString text = QString("<center><h1>Mouse  Left Release: </h1></center>");
        this->setText(text);
    }
    else
    {
        right_clicked = false;
        left_clicked = false;
    }
}

//����ͷŴ���
void MouseMovement::mouseReleaseEvent(QMouseEvent* ev) {
    
    right_clicked = false;
    left_clicked = false;
    if (ev->button() == Qt::LeftButton) {
        int i = ev->x();
        int j = ev->y();
        QString text = QString("<center><h1>Mouse Left release: (%1,%2)</h1></center>").arg(i).arg(j);
        this->setText(text);
        
    }
    else if (ev->button() == Qt::RightButton)
    {
        int i = ev->x();
        int j = ev->y();
        QString text = QString("<center><h1>Mouse  Right Release: (%1,%2)</h1></center>").arg(i).arg(j);
        this->setText(text);
    }

}

//void MouseMovement::keyPressEvent(QKeyEvent* event)
//{
//    if (event->key() == Qt::Key_W)
//        std::cout << "Asdasdasdas" << std::endl;
//}

//���봰������
void MouseMovement::enterEvent(QEvent*) {
    QString text = QString("<center><h1>Mouse enter</h1></center>");
    this->setText(text);
    is_range = true;
}

//�뿪��������
void MouseMovement::leaveEvent(QEvent*) {
    QString text = QString("<center><h1>Mouse leave</h1></center>");
    this->setText(text);
    is_range = false;
}
