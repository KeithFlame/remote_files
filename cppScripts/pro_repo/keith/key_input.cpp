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
    // 普通键
    switch (event->key())
    {
        // ESC键
    case Qt::Key_W:
        w_clicked = true;
        a_clicked = false;
        s_clicked = false;
        d_clicked = false;
        break;
        // 回车键
    case Qt::Key_A:
        w_clicked = false;
        a_clicked = true;
        s_clicked = false;
        d_clicked = false;
        break;
        // F1键
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

    // 两键组合
    if (event->modifiers() == Qt::ControlModifier) { // 如果按下了CTRL键
        if (event->key() == Qt::Key_M) {
            
        }
    }

    if (event->modifiers() == Qt::AltModifier) { // 如果按下了ALT键
        if (event->key() == Qt::Key_M)
        {
        }
    }

    if (event->modifiers() == Qt::ShiftModifier) { // 如果按下了Shift键
        if (event->key() == Qt::Key_M)
        {
        }
    }

    // 三键组合Shift + Ctrl + A的实现
    if (event->modifiers() == (Qt::ShiftModifier | Qt::ControlModifier) && event->key() == Qt::Key_A) {
    }
}

// 键盘释放事件
void KeyInput::keyReleaseEvent(QKeyEvent* event)
{
    // 方向UP键
    if (event->key() == Qt::Key_Up)
    {

    }
}