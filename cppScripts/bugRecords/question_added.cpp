
#include "question_added.h"
#include <QMessageBox>
#include <fstream>
#include <iostream>
#include <QFileDialog>
#include <opencv2/opencv.hpp>

#pragma execution_character_set("utf-8");  

question_added::question_added(QWidget* parent)
	: QWidget(parent)
{
	ui.setupUi(this);

	bool re = connect(ui.quit_added, SIGNAL(clicked()), this, SLOT(pushQuitSlots()));
	bool rc = connect(ui.confirm_added, SIGNAL(clicked()), this, SLOT(pushConfirmSlots()));
	bool rb = connect(ui.screen_print, SIGNAL(clicked()), this, SLOT(printPartialScreenSlots()));
	bool ra = connect(ui.open_file, SIGNAL(clicked()), this, SLOT(openFilesSlots()));


	frame = cv::Mat(3, 3,CV_8UC1);
}

question_added::~question_added()
{
	emit sendUpdate();
}

void question_added::acceptTotalQuestionNum(int question_num)
{
	sub_total_question_num = question_num;
	QString str = "��" + QString::number(sub_total_question_num + 1) + "������������ӡ�����";
	ui.add_manual->setText(str);
}

void question_added::pushQuitSlots()
{
	QMessageBox::StandardButton reply;
	QString solution_cur = ui.question_discription->text();
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
	ui.question_discription->setText("");
	emit sendUpdate();
	this->close();
}

void question_added::pushConfirmSlots()
{
	//Ԥ��
	QMessageBox::StandardButton reply;
	if (frame.cols < 10)
	{
		QMessageBox::information(this,
			"����",
			"δѡȡͼ��");
		return;
	}

	QString Qsave_file = ui.question_discription->text();
	std::string save_file = Qsave_file.toStdString();

	std::string save_file_path = "./bugList/question_" + std::to_string(sub_total_question_num + 1) + ".log";

	std::ofstream resultWrite;
	resultWrite.open(save_file_path, std::ios::out);
	if (!resultWrite.is_open())
	{
		QString error_info = "��" + QString::number(sub_total_question_num + 1) + "�������Ӧ���ĵ��޷���";
		QMessageBox::information(this,
			"����",
			error_info.toStdString().c_str());
		return;
	}
	resultWrite << save_file << "\n";
	resultWrite.close();
	save_file_path = "./bugList/solution_" + std::to_string(sub_total_question_num + 1) + ".log";
	resultWrite.open(save_file_path, std::ios::out);
	resultWrite.close();
	solution = Qsave_file;


	//pictures
	std::string pic_name = "./bugList/question_" + std::to_string(sub_total_question_num + 1) + ".jpg";
	if (!(cv::imwrite(pic_name, frame)))
	{
		QString error_info = "��" + QString::number(sub_total_question_num + 1) + "�������Ӧ��ͼ�񱣴�ʧ��";
		QMessageBox::information(this,
			"����",
			error_info.toStdString().c_str());


	}
	else
	{
		QString error_info = "��" + QString::number(sub_total_question_num + 1) + "�������Ӧ��ͼ�񱣴�ɹ���";
		QMessageBox::information(this,
			"��ʾ",
			error_info.toStdString().c_str());

	}

}

void question_added::openFilesSlots()
{
	QFileDialog* f = new QFileDialog(this);
	f->setWindowTitle("ѡ��ͼ���ļ�*.jpg");
	f->setNameFilter("*.jpg");
	f->setViewMode(QFileDialog::Detail);

	QString Qfile_path;
	if (f->exec() == QDialog::Accepted)
		Qfile_path = f->selectedFiles()[0];


	std::string file_path = Qfile_path.toStdString();
	if (file_path == "")
		return;
	cv::Mat	img = cv::imread(file_path);
	frame = img;
	cv::imshow("", img);
	cv::waitKey(10);
	///�ļ�����//

	
	


}

void question_added::printPartialScreenSlots()
{
}
