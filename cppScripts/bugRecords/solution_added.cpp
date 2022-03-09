#include "solution_added.h"
#include <QMessageBox>
#include <fstream>
#include <iostream>


#pragma execution_character_set("utf-8");  

solution_added::solution_added(QWidget *parent)
	: QWidget(parent)
{
	ui.setupUi(this);

	bool re = connect(ui.close_window, SIGNAL(clicked()), this, SLOT(pushCloseSlots()), Qt::ConnectionType::QueuedConnection);
	bool rc =connect(ui.confirm_add_solution, SIGNAL(clicked()), this, SLOT(pushConfirmSlots()), Qt::ConnectionType::QueuedConnection);
	
}

solution_added::~solution_added()
{
	
}

void solution_added::acceptQuestionNum(int question_num)
{
	sub_question_num = question_num;
	setManualSlots();
}

void solution_added::pushCloseSlots()
{
	QMessageBox::StandardButton reply;
	QString solution_cur = ui.lineEdit->text();
	if (solution != solution_cur)
		reply = QMessageBox::question(this, "��ʾ", "��δ�����Ƿ����������\n\tYes��������\n\tNo��������", QMessageBox::Yes | QMessageBox::No);
	else
		reply = QMessageBox::Yes;
	if (reply == QMessageBox::Yes)
	{

	}
	else
	{
		return;
	}
	ui.lineEdit->setText("");
	this->close();
}

void solution_added::setManualSlots()
{
	QString str = "��" + QString::number(sub_question_num + 1) + "�������������ӽ������������";
	ui.add_manual->setText(str);
}

void solution_added::pushConfirmSlots()
{
	QString Qsave_file = ui.lineEdit->text();
	std::string save_file = Qsave_file.toStdString();
	
	std::string save_file_path = "./bugList/solution_" + std::to_string(sub_question_num + 1) + ".log";

	std::ofstream resultWrite;
	resultWrite.open(save_file_path, std::ios::out | std::ios::app);
	if (!resultWrite.is_open())
	{
		QString error_info = "��" + QString::number(sub_question_num + 1) + "�������Ӧ���ĵ��޷���";
		QMessageBox::information(this,
			"����",
			error_info.toStdString().c_str());
		return ;
	}
	resultWrite << save_file<<"\n";
	resultWrite.close();
	solution = Qsave_file;
}
