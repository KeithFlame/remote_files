#pragma once

#include <QWidget>
#include "ui_question_added.h"
#include <opencv2/opencv.hpp>

class question_added : public QWidget
{
	Q_OBJECT

public:
	question_added(QWidget* parent = 0);
	~question_added();
	void acceptTotalQuestionNum(int question_num);

private:
	Ui::question_added ui;
	QString solution = "";
	int sub_total_question_num;
	cv::Mat frame;

private slots:
	void pushConfirmSlots();
	void pushQuitSlots();
	void openFilesSlots();
	void printPartialScreenSlots();

signals:
	void sendUpdate();
};
