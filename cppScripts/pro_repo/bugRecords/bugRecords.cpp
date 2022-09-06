#include "bugRecords.h"
#include "typeDefine.h"
#include <QImage>
#include <QMessageBox>
#include <QDateTime>
#include <fstream>
#include <opencv2/opencv.hpp>
#include <opencv2\imgproc\types_c.h>

#pragma execution_character_set("utf-8")  

bugRecords::bugRecords(QWidget* parent)
    : QMainWindow(parent)
{
    

    ui.setupUi(this);
    QString label_stype_normal = "font-family:'新宋体';font-size:20px;background-color:#55FF7F;color:black;";

    setWindowTitle(QStringLiteral("Question Record"));

    warning_windows_timer = new QTimer(this);
    warning_windows_timer->setSingleShot(true);

    adding_solution = new solution_added();
    adding_question = new question_added();
    

    connect(ui.addQuestion, SIGNAL(clicked()), this, SLOT(pushAddQuestionSlots()));
    connect(ui.add_solution, SIGNAL(clicked()), this, SLOT(pushAddSolutionSlots()));
    connect(ui.back_to_top, SIGNAL(clicked()), this, SLOT(pushGoBackToTopSlots()));
    connect(ui.last_question, SIGNAL(clicked()), this, SLOT(pushLastQuestionSlots()));
    connect(ui.next_question, SIGNAL(clicked()), this, SLOT(pushNextQuestionSlots()));
    connect(ui.to_bottom, SIGNAL(clicked()), this, SLOT(pushGoToBottomSlots()));

    connect(warning_windows_timer, SIGNAL(timeout()), this, SLOT(destroyWarningWindow()));
    
    connect(adding_question, SIGNAL(sendUpdate()), this, SLOT(updateRecord()));

    question_num = 0;
    getQuestionNumber();
    showInformation();

    GR = new GenRep();
    QPushButton* gr = new QPushButton(this);
    gr->setObjectName(QStringLiteral("cs"));
    gr->setText("生成报告");
    gr->setGeometry(QRect(500, 500, 100, 40));
    connect(gr, SIGNAL(clicked()), this, SLOT(setPdfCreatorSlot()));

}

void bugRecords::destroyWarningWindow()
{
    ui.label->hide();
}

void bugRecords::addNewSolution()
{
    //adding_solution->acceptQuestionNum(question_num);
    //adding_solution->show();

    //connect(this, SIGNAL(sendQuestionNum(int)), adding_solution, SLOT(setManual()));
    //emit sendQuestionNum(question_num);
    //addNewSolution();
}

void bugRecords::showInformation()
{
    ui.textBrowser_question->clear();
    //ui.textBrowser_solution->clear();
    showPicture();
    showQuestionInformation();
    showSolutionInformation();
}

void bugRecords::showWarning(int signal)
{
    QString warning_word;
    TYPE_KEITH::WARNING_WORD type_word = (TYPE_KEITH::WARNING_WORD)signal;
    switch (type_word)
    {
    case TYPE_KEITH::WARNING_WORD::LAST_QUESTION:
        warning_word = "最后一个问题描述！";
        break;
    case TYPE_KEITH::WARNING_WORD::FIRST_QUESTION:
        warning_word = "第一个问题描述！";
        break;
    case TYPE_KEITH::WARNING_WORD::NUMBER_CONFLICT_BETWEEN_SOLUTION_QUESTION:
        warning_word = "问题数量和答案数量冲突！！";
        break;
    default:
        warning_word = "没有问题";
        break;
    }
    warning_windows_timer->start(1000);
    QRect rect_window=geometry();
    ui.label->setMaximumHeight(200);
    ui.label->setMinimumHeight(200);
    ui.label->setMaximumWidth(500);
    ui.label->setMidLineWidth(500);
    
    QFont font;
    font.setPointSize(25);
    ui.label->setFont(font);
    
    QPalette palette;
    palette.setColor(QPalette::Background,Qt::cyan);
    palette.setColor(QPalette::WindowText, Qt::red);
    ui.label->setPalette(palette);
    ui.label->setStyleSheet(QLatin1String("color::red;"));
    ui.label->setText(warning_word);
    ui.label->setGeometry(int((rect_window.width()-ui.label->width())/2), 
        int((rect_window.height() - ui.label->height()) / 3),ui.label->width(),ui.label->height());
    ui.label->show();
}

void bugRecords::getQuestionNumber()
{
    QDir* dir = new QDir("./bugList/");
    QStringList filter_jpg;
    filter_jpg << "*.jpg";
    dir->setNameFilters(filter_jpg);
    QList<QFileInfo>* file_info_jpg = new QList<QFileInfo>(dir->entryInfoList(filter_jpg));
    int count_jpg = file_info_jpg->count();
    count_jpg = count_jpg > 0 ? count_jpg : 0;

    QStringList filter_log;
    filter_log << "*.log";
    dir->setNameFilters(filter_log);
    QList<QFileInfo>* file_info_log = new QList<QFileInfo>(dir->entryInfoList(filter_log));
    int count_log = file_info_log->count();
    count_log = count_log > 0 ? (int)(count_log/2) : 0;
    if(count_log== count_jpg)
        total_question_num = count_log;
    else
    {
        total_question_num = count_log < count_jpg ? count_log : count_jpg;
        showWarning(2);
    }
    delete file_info_jpg;
    delete file_info_log;
    delete dir;
    total_question_num;
}

void bugRecords::pushLastQuestionSlots()
{
    question_num--;
    if (question_num<1)
    {
        
        ui.last_question->setEnabled(false);
        showWarning(1);  
        question_num = 0;
    }
    else
    {
          
    }
    showInformation();
    if(question_num< total_question_num)
        ui.next_question->setEnabled(true);
    //str = "\nquestion_num_after= " + QString::number(question_num);
    //ui.textBrowser_solution->append(str);
}

void bugRecords::pushNextQuestionSlots()
{
    question_num++;
    if (question_num > total_question_num - 2)
    {
        ui.next_question->setEnabled(false);
        showWarning(0);
        question_num = total_question_num - 1;
    }
        
    showInformation();
    
    if (question_num > 0)
        ui.last_question->setEnabled(true);
}

void bugRecords::pushGoToBottomSlots()
{
    question_num = total_question_num -1;
    ui.next_question->setEnabled(false);
    showWarning(0);
    showInformation();
    ui.last_question->setEnabled(true);
}

void bugRecords::pushGoBackToTopSlots()
{
    question_num = 0;
    ui.last_question->setEnabled(false);
    showWarning(1);
    showInformation();
    ui.next_question->setEnabled(true);
}

void bugRecords::pushAddSolutionSlots()
{
    adding_solution->acceptQuestionNum(question_num);
    adding_solution->show();


}

void bugRecords::pushAddQuestionSlots()
{
    adding_question->acceptTotalQuestionNum(total_question_num);
    adding_question->show();
}


bool bugRecords::showPicture()
{
    std::string image_path = "./bugList/question_" + std::to_string(question_num + 1) + ".jpg";
    QFileInfo fileInfo(QString::fromStdString(image_path));
    
    if (!(fileInfo.isFile())) //加载图像
    {
        QString error_info = "第" + QString::number(question_num + 1) + "个问题对应的图像失效或者不存在";
        //char err_info = error_info.toStdString().c_str();
        QMessageBox::information(this,
            "错误",
            error_info.toStdString().c_str());
        
        return 0;
    }
    cv::Mat showPict = cv::imread(image_path);
    cv::cvtColor(showPict, showPict, CV_BGR2RGB);//qt仅支持RGB顺序
    cv::resize(showPict, showPict, cv::Size(1700,1200));//#include <opencv2\imgproc\types_c.h>
    QImage srcQImage = QImage((uchar*)(showPict.data), showPict.cols, showPict.rows, QImage::Format_RGB888);

    QPixmap pix = QPixmap(QPixmap::fromImage(srcQImage));
    pix = pix.scaled(ui.figure_show->size(), Qt::KeepAspectRatio);
    ui.figure_show->setScaledContents(true);
    //ui.figure_show->setPixmap(pix);
    //ui.figure_show->resize(srcQImage.size());
    //ui.figure_show->show();
    ui.figure_show->setPixmap(pix.scaled(ui.figure_show->size())); 
    return 1;
}

void bugRecords::showSolutionInformation()
{
    std::fstream fread;
    std::string file_name = "./bugList/solution_"+std::to_string(question_num + 1) + ".log";
    fread.open(file_name);
    
    if (!fread.is_open())
    {
        QString error_info = "第" + QString::number(question_num + 1) + "个问题对应的解决方案不存在";
        QMessageBox::information(this,
            "错误",
            error_info.toStdString().c_str());
        
        return;
    }
    std::vector<std::string> result;
    char buf[5000] = { 0 };
    fread.getline(buf, sizeof(buf));
    std::string str(buf);
    while (fread.getline(buf, sizeof(buf)))
    {
        std::string str0(buf);
        str = str + "\n" + str0;
    }
    fread.close();
    QString string_temp;
    QString str2 = "针对第" + QString::number(question_num + 1) + "个问题的可能解决方法:\n";
    QDateTime current_date_time = QDateTime::currentDateTime();
    string_temp = current_date_time.toString("MM.dd hh:mm:ss.zzz: ") + str2 + QString::fromStdString(str);
    ui.textBrowser_solution->append(string_temp);
}

void bugRecords::updateRecord()
{
    getQuestionNumber();
}

void bugRecords::showQuestionInformation()
{
    std::fstream fread;
    std::string file_name = "./bugList/question_" + std::to_string(question_num + 1) + ".log";
    fread.open(file_name);
    if (!fread.is_open())
    {
        QString error_info = "第" + QString::number(question_num + 1) + "个问题对应的问题描述不存在";
        QMessageBox::information(this,
            "错误",
            error_info.toStdString().c_str());
        
        return;
    }

    std::vector<std::string> result;
    char buf[5000] = { 0 };
    fread.getline(buf, sizeof(buf));
    std::string str(buf);
    while (fread.getline(buf, sizeof(buf)))
    {
        std::string str0(buf);
        str = str + "\n" + str0;
    }
    fread.close();
    QString string_temp;
    QString str2 = "第" + QString::number(question_num + 1) + "个问题的描述:\n";
    string_temp = str2+  QString::fromStdString(str);
    ui.textBrowser_question->append(string_temp);
}



void bugRecords::setPdfCreatorSlot()
{
    GR->setFilePath("./data/");
    GR->setSystemProc("SR-ENS-600");
    GR->setSystemSerial("6002020023333");
    GR->setSystemBatch("20221222");
    GR->setToolProc("SR-ENT-SP0807S");
    GR->setToolSerial("6372022000089");
    GR->setToolBatch("20221222");
    GR->show();
}