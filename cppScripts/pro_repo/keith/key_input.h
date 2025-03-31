#pragma once
#ifndef KEY_INPUT_H_
#define KEY_INPUT_H_

#include <QKeyEvent>  //����ģ��
#include <QWidget>

class KeyInput : public QWidget
{
    Q_OBJECT

public:
    KeyInput(QWidget* parent = nullptr);
    ~KeyInput();

protected:

    void keyPressEvent(QKeyEvent* ev); //���̰����¼�
    void keyReleaseEvent(QKeyEvent* ev); //�����ͷ��¼�

public:
    bool w_clicked, a_clicked, s_clicked, d_clicked;
};
#endif // KEY_INPUT_H_