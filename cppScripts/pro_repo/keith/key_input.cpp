#include "key_input.h"


KeyInput::KeyInput(QWidget* parent)
    : QWidget(parent)
    , w_clicked(false)
    , a_clicked(false)
    , s_clicked(false)
    , d_clicked(false)

{

}

KeyInput::~KeyInput()
{
}
void KeyInput::keyPressEvent(QKeyEvent* event)
{
    // ��ͨ��
    switch (event->key())
    {
        // ESC��
    case Qt::Key_W:
        w_clicked = true;
        a_clicked = false;
        s_clicked = false;
        d_clicked = false;
        break;
        // �س���
    case Qt::Key_A:
        w_clicked = false;
        a_clicked = true;
        s_clicked = false;
        d_clicked = false;
        break;
        // F1��
    case Qt::Key_S:
        w_clicked = false;
        a_clicked = false;
        s_clicked = true;
        d_clicked = false;
        break;
    case Qt::Key_D:
        w_clicked = false;
        a_clicked = false;
        s_clicked = false;
        d_clicked = true;
        break;
    }

    // �������
    if (event->modifiers() == Qt::ControlModifier) { // ���������CTRL��
        if (event->key() == Qt::Key_M) {
            
        }
    }

    if (event->modifiers() == Qt::AltModifier) { // ���������ALT��
        if (event->key() == Qt::Key_M)
        {
        }
    }

    if (event->modifiers() == Qt::ShiftModifier) { // ���������Shift��
        if (event->key() == Qt::Key_M)
        {
        }
    }

    // �������Shift + Ctrl + A��ʵ��
    if (event->modifiers() == (Qt::ShiftModifier | Qt::ControlModifier) && event->key() == Qt::Key_A) {
    }
}

// �����ͷ��¼�
void KeyInput::keyReleaseEvent(QKeyEvent* event)
{
    // ����UP��
    if (event->key() == Qt::Key_Up)
    {

    }
}