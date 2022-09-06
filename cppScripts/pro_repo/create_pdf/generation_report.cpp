#include "generation_report.h"
#include <QPushButton>
#include <QLabel>
#include <QGridLayout>
#include <QFile>
#include <QtPrintSupport/QtPrintSupport>
#pragma execution_character_set("utf-8")

class GenRep::subUI
{
public:
    QLabel* label_1;
    QPushButton* button_1;
    QPushButton* button_2;
    QPushButton* button_3;
    QPushButton* button_4;
    QPushButton* button_5;
    QPushButton* button_6;

    void setupUi(QWidget* parent);
    void retranslate(QWidget* parent);
    void signalSlot(QWidget* parent);
};

void GenRep::subUI::setupUi(QWidget* parent)
{
    label_1 = new QLabel(parent);
    button_1 = new QPushButton(parent);
    button_2 = new QPushButton(parent);
    button_3 = new QPushButton(parent);
    button_4 = new QPushButton(parent);
    button_5 = new QPushButton(parent);
    button_6 = new QPushButton(parent);
    label_1->setGeometry(QRect(20, 20, 100, 80));
    label_1->setWordWrap(true);
    label_1->setAlignment(Qt::AlignCenter);
    button_1->setGeometry(QRect(20, 120, 100, 20));
    button_2->setGeometry(QRect(20, 160, 100, 20));
    button_3->setGeometry(QRect(20, 200, 100, 20));
    button_4->setGeometry(QRect(20, 240, 100, 20));
    button_5->setGeometry(QRect(20, 280, 100, 20));
    button_6->setGeometry(QRect(40, 360, 60, 20));

    retranslate(parent);
    signalSlot(parent);
}

void GenRep::subUI::retranslate(QWidget* parent)
{
    label_1->setText("./data/");
    button_1->setText("AccuRepeat");
    button_2->setText("Sensitivity");
    button_3->setText("PositionRepeat");
    button_4->setText("MaxWorkspace");
    button_5->setText("Delay");
    button_6->setText("È·¶¨");

    label_1->setMinimumWidth(10);
    button_1->setMinimumWidth(10);
    button_2->setMinimumWidth(10);
    button_3->setMinimumWidth(10);
    button_4->setMinimumWidth(10);
    button_5->setMinimumWidth(10);
    button_5->setMinimumWidth(10);
    parent->setFixedSize(140, 400);
}

void GenRep::subUI::signalSlot(QWidget* parent)
{
    connect(button_1, SIGNAL(clicked()), parent, SLOT(CreatePdfAccuRepeatSlot()));
    connect(button_2, SIGNAL(clicked()), parent, SLOT(CreatePdfSensitivitySlot()));
    connect(button_3, SIGNAL(clicked()), parent, SLOT(CreatePdfPositionRepeatSlot()));
    connect(button_4, SIGNAL(clicked()), parent, SLOT(CreatePdfMaxWorkspaceSlot()));
    connect(button_5, SIGNAL(clicked()), parent, SLOT(CreatePdfDelaySlot()));
    connect(button_6, SIGNAL(clicked()), parent, SLOT(setConfirmSlot()));
}

GenRep::GenRep(QWidget* parent)
	:QWidget(parent)
	, file_path("./data/")
    , m_subUI(new subUI)
    , system_proc("SR-ENS-600")
    , system_serial("6002020000003")
    , system_batch("20200003")
    , tool_proc("SR-ENT-SP0802S")
    , tool_serial("6322021000011")
    , tool_batch("20210101")
{
    m_subUI->setupUi(this);
    setTipWindow(file_path);
}

void GenRep::setFilePath(QString str)
{
    file_path = str;
    setTipWindow(str);
}

void GenRep::setSystemProc(QString system_proc)
{
    this->system_proc = system_proc;
}

void GenRep::setSystemSerial(QString system_serial)
{
    this->system_serial = system_serial;
}

void GenRep::setSystemBatch(QString system_batch)
{
    this->system_batch = system_batch;
}

void GenRep::setToolProc(QString tool_proc)
{
    this->tool_proc = tool_proc;
}

void GenRep::setToolSerial(QString tool_serial)
{
    this->tool_serial = tool_serial;
}

void GenRep::setToolBatch(QString tool_batch)
{
    this->tool_batch = tool_batch;
}

GenRep::~GenRep()
{

}

void GenRep::setTipWindow(QString str)
{
    m_subUI->label_1->setText(str);
}

void GenRep::setConfirmSlot()
{
    this->hide();
}
