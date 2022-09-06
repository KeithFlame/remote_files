#pragma once

#include <QtWidgets/QMainWindow>
#include "ui_QT_opengl_test.h"

class QT_opengl_test : public QMainWindow
{
    Q_OBJECT

public:
    QT_opengl_test(QWidget *parent = nullptr);
    ~QT_opengl_test();

private:
    Ui::QT_opengl_testClass ui;
};
