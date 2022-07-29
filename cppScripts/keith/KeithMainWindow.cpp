#include "KeithMainWindow.h"
#include "userReader.h"
#include <qpainter.h>


#pragma execution_character_set("utf-8")

QtKeithMainWindow::QtKeithMainWindow(QWidget *parent)
	: QMainWindow(parent)
{
	ui.setupUi(this);

	//user::User user_reader;
	//user_reader.readUserXml();
	//���õ�¼���汳��
	setLoginBackground();
	connect(ui.load_command,SIGNAL(clicked()),this,SLOT(loginConfirm()));
	connect(this, SIGNAL(logSuccessSig()), this, SLOT(goToContentWindowSlot()));
	connect(ui.registration_putton, SIGNAL(clicked()), this, SLOT(addNewUserUISlot()));
}

QtKeithMainWindow::~QtKeithMainWindow()
{

}

void QtKeithMainWindow::setLoginBackground()
{
	
	QPixmap pixmap("./conf/pic/bg.png");

	QPixmap temp(pixmap.size());
	temp.fill(Qt::transparent);

	//����painter���»�һ������
	QPainter p1(&temp);
	p1.setCompositionMode(QPainter::CompositionMode_Source);
	p1.drawPixmap(0, 0, pixmap);
	p1.setCompositionMode(QPainter::CompositionMode_DestinationIn);

	//����QColor�е��ĸ���������͸���ȣ�0��255
	p1.fillRect(temp.rect(), QColor(0, 0, 0, 75));
	p1.end();

	pixmap = temp;
	pixmap.scaled(ui.backgroud->size(), Qt::KeepAspectRatio);
	ui.backgroud->setScaledContents(true);
	ui.backgroud->setPixmap(pixmap);
	
	ui.backgroud->show();
}

void QtKeithMainWindow::showXmlFrame(FileProcess fp)
{
	switch (fp)
	{
	case FileProcess::FILE_PROCESS_SUCCESS:
	{
		if (users.is_admin)
			;
		else if (users.is_authentication)
		{
			content_ui.show();
			this->close();
		}
		else if(users.password_tip != "")
		{
			ui.warning_label->setText("�������");
			ui.password_value->setToolTip(QString::fromStdString(users.password_tip));
		}
		else
			ui.warning_label->setText("�޴��˺�");

		break;
	}
	case FileProcess::CAN_NOT_OPEN_FILE:
	{
		ui.warning_label->setText("�ؼ��û��ļ�ȱʧ");
		break;
	}
	case FileProcess::FILE_IS_EMPTY:
	{
		ui.warning_label->setText("�ؼ��û��ļ����۸�");
		break;
	}
	case FileProcess::FILE_PATTERN_ERROR:
	{
		ui.warning_label->setText("�ؼ��û��ļ��쳣");
		break;
	}
	case FileProcess::FILE_OPEN_UNNECCESARY:
	{
		ui.warning_label->setText("���ⲿ�������");
		break;
	}
	default:
	{
		
	}
	}
}

void QtKeithMainWindow::loginConfirm()
{
	user::UserReader user_current;
	FileProcess fp;

	user_current.name = ui.account_value->text().toStdString();
	user_current.password = ui.password_value->text().toStdString();

	fp = users.checkUserXml(user_current);
	showXmlFrame(fp);
}

void QtKeithMainWindow::goToContentWindowSlot()
{
	content_ui.show();
	this->close();
}

void QtKeithMainWindow::addNewUserUISlot()
{
	new_user_ui.show();
}



