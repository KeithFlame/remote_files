#pragma once

#include <QWidget>
#include "ui_solution_added.h"

class solution_added : public QWidget
{
	Q_OBJECT

public:
	solution_added(QWidget* parent=0);
	~solution_added();
	void acceptQuestionNum(int question_num);

private:
	Ui::solution_added ui;
	QString solution = "";
	int sub_question_num;
	
private slots:
	void pushConfirmSlots();
	void pushCloseSlots();
	void setManualSlots();

	
};
