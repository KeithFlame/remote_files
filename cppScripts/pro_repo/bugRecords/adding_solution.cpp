#include "adding_solution.h"


SubWidget::SubWidget(QWidget* parent)
    : QWidget(parent)
{
    this->setParent(parent);
    this->setWindowTitle(QString::fromLocal8Bit("小弟"));
    this->resize(QSize(500, 500));

    bt = new QPushButton(this);
    bt->setText(QString::fromLocal8Bit("切换到主窗口"));
    bt->move(QPoint(50, 50));

    connect(bt, &QPushButton::clicked, this, &SubWidget::sendSlot);
}


SubWidget::~SubWidget()
{
}

void SubWidget::sendSlot()
{
    emit mySignal();
    emit mySignalParm(300, QString::fromLocal8Bit("已经切换到主窗口"));
}