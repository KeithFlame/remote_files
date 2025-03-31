#pragma once
#ifndef KEY_INPUT_H_
#define KEY_INPUT_H_

#include <QKeyEvent>  //键盘模块
#include <QWidget>

class KeyInput : public QWidget
{
    Q_OBJECT

public:
    KeyInput(QWidget* parent = nullptr);
    ~KeyInput();

protected:

    void keyPressEvent(QKeyEvent* ev); //键盘按下事件
    void keyReleaseEvent(QKeyEvent* ev); //键盘释放事件

public:
    bool w_clicked, a_clicked, s_clicked, d_clicked;
};
#endif // KEY_INPUT_H_