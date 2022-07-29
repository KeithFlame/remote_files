#include "newUserUI.h"
#include <qpainter.h>
#pragma execution_character_set("utf-8")

newUserUI::newUserUI(QWidget *parent)
	: QMainWindow(parent)
{
	ui.setupUi(this);
	setDefaultValue();
	//setLoginBackground();
	
	connect(ui.confirm_pushButton, SIGNAL(clicked()), this, SLOT(confirmNewUserSlots()));
	setFocusPolicy(Qt::TabFocus);
}

newUserUI::~newUserUI()
{

}

void newUserUI::confirmNewUserSlots()
{
	std::string name = ui.lineEdit->text().toStdString();
	std::string password = ui.lineEdit_2->text().toStdString();
	std::string password2 = ui.lineEdit_3->text().toStdString();
	std::string password_tips = ui.lineEdit_4->text().toStdString();
	std::string password_protection = ui.lineEdit_5->text().toStdString();
	std::string password_protection_password = ui.lineEdit_6->text().toStdString();
	if (name == "" || password == "")
	{
		ui.label_8->setText("账号或者密码没有输入");
		return;
	}	
	if (password != password2)
	{
		ui.label_8->setText("两次密码不一致");
		return;
	}
		
	if (name.length() < 3 || password.length() < 3)
	{
		ui.label_8->setText("账号或者密码长度需要大于3位");
		return;
	}
		
	user::UserReader user_info;
	user_info.name = name;
	user_info.password = password;
	user_info.password_tips = password_tips == "" ? password : password_tips;
	user_info.password_protection = password_protection ==""? "最快乐的人是？": password_protection;
	user_info.password_protection_password = password_protection_password == "" ? "Keith W." : password_protection_password;
	user_info.serials = 0;
	int flag = 1;
	FileProcess fp;
	fp = new_user.addUserXml(user_info, flag);
	showXmlFrame(fp,flag);
}

void newUserUI::setLoginBackground()
{
	//加入了背景后，就无法填写lineEdit
#if 0
	QPixmap pixmap("./conf/pic/delta.png");

	QPixmap temp(pixmap.size());
	temp.fill(Qt::transparent);

	//利用painter重新画一个出来
	QPainter p1(&temp);
	p1.setCompositionMode(QPainter::CompositionMode_Source);
	p1.drawPixmap(0, 0, pixmap);
	p1.setCompositionMode(QPainter::CompositionMode_DestinationIn);

	//根据QColor中第四个参数设置透明度，0～255
	p1.fillRect(temp.rect(), QColor(0, 0, 0, 75));
	p1.end();

	pixmap = temp;
	pixmap.scaled(ui.label_7->size(), Qt::KeepAspectRatio);
	ui.label_7->setScaledContents(true);
	ui.label_7->setPixmap(pixmap);

	ui.label_7->show();
#endif 
}

void newUserUI::setDefaultValue()
{
	ui.lineEdit_4->setPlaceholderText("默认是密码重复");
	ui.lineEdit_5->setPlaceholderText("最快乐的人是？");
	ui.lineEdit_6->setPlaceholderText("Keith W.");
}

void newUserUI::showXmlFrame(FileProcess fp,int flag)
{
	switch (fp)
	{
	case FileProcess::FILE_PROCESS_SUCCESS:
	{
		if(flag == -2)
			ui.label_8->setText("用户数量超限");
		else if(flag == -1)
			ui.label_8->setText("该用户已存在");
		else if(flag == 1)
			ui.label_8->setText("用户信息保存失败");
		else
		{
			ui.label_8->setText("新建成功");
			this->close();
		}
			
		break;
	}
	case FileProcess::CAN_NOT_OPEN_FILE:
	{
		ui.label_8->setText("关键用户文件缺失");
		break;
	}
	case FileProcess::FILE_IS_EMPTY:
	{
		ui.label_8->setText("关键用户文件被篡改");
		break;
	}
	case FileProcess::FILE_PATTERN_ERROR:
	{
		ui.label_8->setText("关键用户文件异常");
		break;
	}
	case FileProcess::FILE_OPEN_UNNECCESARY:
	{
		ui.label_8->setText("非外部程序错误");
		break;
	}
	default:
	{

	}
	}
	return;
}