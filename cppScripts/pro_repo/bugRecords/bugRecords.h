#pragma once
#include <QtWidgets/QMainWindow>
#include "ui_bugRecords.h"
#include <qtimer.h>
#include <vector>
#include <qdir.h>
#include "solution_added.h"
#include "question_added.h"
#include "generation_report.h"

class bugRecords : public QMainWindow
{
    Q_OBJECT

public:
    bugRecords(QWidget *parent = Q_NULLPTR);

private:
    Ui::bugRecordsClass ui;
    QTimer* warning_windows_timer;
    solution_added* adding_solution;
    question_added* adding_question;
    //cv::Mat frame;
    int question_num;
    int total_question_num;
    //GenerationReport* gr;

    void addNewSolution();  //为了对已有问题添加新的解决方法
    void showInformation();
    void showWarning( int signal = 0);
    void getQuestionNumber();
    bool showPicture();
    void showQuestionInformation();
    void showSolutionInformation();
private slots:      //和按键无关
    void destroyWarningWindow();
    void updateRecord();
private slots:
    void pushNextQuestionSlots();
    void pushLastQuestionSlots();
    void pushGoToBottomSlots();
    void pushGoBackToTopSlots();
    void pushAddSolutionSlots();
    void pushAddQuestionSlots();

private slots:
    void setPdfCreatorSlot();
private:
    GenRep* GR;
signals:
    void sendQuestionNum(int);
};
