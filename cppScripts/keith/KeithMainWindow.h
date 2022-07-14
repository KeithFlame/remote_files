#pragma once

#include <QMainWindow>
#include "ui_QtWidgetsClass.h"

class QtKeithMainWindow : public QMainWindow
{
	Q_OBJECT

public:
	QtKeithMainWindow(QWidget *parent = nullptr);
	~QtKeithMainWindow();

private:
	Ui::QtWidgetsClassClass ui;
};
