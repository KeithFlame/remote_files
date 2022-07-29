#pragma once

#include <QMainWindow>
#include "contentWindow.h"
#include "newUserUI.h"
#include "ui_QtWidgetsClass.h"
#include "userReader.h"
#include "windows.h"

class QtKeithMainWindow : public QMainWindow
{
	Q_OBJECT

public:
	QtKeithMainWindow(QWidget *parent = nullptr);
	~QtKeithMainWindow();

private:				//通用函数
	void setLoginBackground();
	void showXmlFrame(FileProcess);

private slots:				//slot函数
	void loginConfirm();
	void goToContentWindowSlot();
	void addNewUserUISlot();

private:
	Ui::QtWidgetsClassClass ui;
	contentWindow content_ui;
	newUserUI new_user_ui;
	user::User users;

signals:
	void logSuccessSig();
	void logSuccessAdminSig();
};
