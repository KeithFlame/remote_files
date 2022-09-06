#pragma once

#include <QMainWindow>
#include "ui_newUserUI.h"
#include "userReader.h"

class newUserUI : public QMainWindow
{
	Q_OBJECT

public:
	newUserUI(QWidget *parent = nullptr);
	~newUserUI();

private:
	void setLoginBackground();
	void setDefaultValue();
	void showXmlFrame(FileProcess,int);

private slots:
	void confirmNewUserSlots();

private:
	Ui::newUserUIClass ui;
	user::User new_user;
};
