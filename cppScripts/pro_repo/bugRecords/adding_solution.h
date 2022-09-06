#pragma once
#ifndef ADDING_SOLUTION_H_
#define ADDING_SOLUTION_H_

#include <QtWidgets/QWidget>
#include <QtWidgets/QPushButton>

class SubWidget : public QWidget
{
    Q_OBJECT

public:
    explicit SubWidget(QWidget* parent = 0);
    ~SubWidget();

    void sendSlot();

signals:
    /**
     * 信号必须要signals关键字来声明
     * 信号没有返回值, 但可以有参数
     * 信号就是函数的声明, 无需定义
     * 使用: emit mySignal();
     * 信号可以被重载
     */
    void mySignal();
    void mySignalParm(int, QString);

public slots:

private:
    QPushButton* bt;
};


#endif // ADDING_SOLUTION_H_