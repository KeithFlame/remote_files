#pragma once

#include <QMainWindow>
#include "ui_contentWindow.h"

class contentWindow : public QMainWindow
{
	Q_OBJECT

public:
	contentWindow(QWidget *parent = nullptr);
	~contentWindow();

private:
	Ui::contentWindowClass ui;
};
