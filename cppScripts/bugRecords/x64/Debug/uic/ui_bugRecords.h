/********************************************************************************
** Form generated from reading UI file 'bugRecords.ui'
**
** Created by: Qt User Interface Compiler version 5.11.1
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_BUGRECORDS_H
#define UI_BUGRECORDS_H

#include <QtCore/QVariant>
#include <QtWidgets/QAction>
#include <QtWidgets/QApplication>
#include <QtWidgets/QGroupBox>
#include <QtWidgets/QHBoxLayout>
#include <QtWidgets/QLabel>
#include <QtWidgets/QMainWindow>
#include <QtWidgets/QMenu>
#include <QtWidgets/QMenuBar>
#include <QtWidgets/QPushButton>
#include <QtWidgets/QStatusBar>
#include <QtWidgets/QTextBrowser>
#include <QtWidgets/QToolBar>
#include <QtWidgets/QVBoxLayout>
#include <QtWidgets/QWidget>

QT_BEGIN_NAMESPACE

class Ui_bugRecordsClass
{
public:
    QWidget *centralWidget;
    QLabel *figure_show;
    QGroupBox *solution;
    QTextBrowser *textBrowser_solution;
    QGroupBox *function;
    QWidget *layoutWidget;
    QHBoxLayout *horizontalLayout;
    QVBoxLayout *verticalLayout;
    QPushButton *next_question;
    QPushButton *to_bottom;
    QPushButton *addQuestion;
    QVBoxLayout *verticalLayout_2;
    QPushButton *last_question;
    QPushButton *back_to_top;
    QPushButton *add_solution;
    QGroupBox *question_discription;
    QTextBrowser *textBrowser_question;
    QLabel *label;
    QMenuBar *menuBar;
    QMenu *menuv0_1;
    QToolBar *mainToolBar;
    QStatusBar *statusBar;

    void setupUi(QMainWindow *bugRecordsClass)
    {
        if (bugRecordsClass->objectName().isEmpty())
            bugRecordsClass->setObjectName(QStringLiteral("bugRecordsClass"));
        bugRecordsClass->resize(1021, 711);
        centralWidget = new QWidget(bugRecordsClass);
        centralWidget->setObjectName(QStringLiteral("centralWidget"));
        figure_show = new QLabel(centralWidget);
        figure_show->setObjectName(QStringLiteral("figure_show"));
        figure_show->setGeometry(QRect(30, 20, 641, 411));
        solution = new QGroupBox(centralWidget);
        solution->setObjectName(QStringLiteral("solution"));
        solution->setGeometry(QRect(20, 490, 931, 141));
        textBrowser_solution = new QTextBrowser(solution);
        textBrowser_solution->setObjectName(QStringLiteral("textBrowser_solution"));
        textBrowser_solution->setGeometry(QRect(20, 21, 891, 111));
        function = new QGroupBox(centralWidget);
        function->setObjectName(QStringLiteral("function"));
        function->setGeometry(QRect(700, 300, 291, 121));
        layoutWidget = new QWidget(function);
        layoutWidget->setObjectName(QStringLiteral("layoutWidget"));
        layoutWidget->setGeometry(QRect(20, 20, 241, 91));
        horizontalLayout = new QHBoxLayout(layoutWidget);
        horizontalLayout->setSpacing(6);
        horizontalLayout->setContentsMargins(11, 11, 11, 11);
        horizontalLayout->setObjectName(QStringLiteral("horizontalLayout"));
        horizontalLayout->setContentsMargins(0, 0, 0, 0);
        verticalLayout = new QVBoxLayout();
        verticalLayout->setSpacing(6);
        verticalLayout->setObjectName(QStringLiteral("verticalLayout"));
        next_question = new QPushButton(layoutWidget);
        next_question->setObjectName(QStringLiteral("next_question"));

        verticalLayout->addWidget(next_question);

        to_bottom = new QPushButton(layoutWidget);
        to_bottom->setObjectName(QStringLiteral("to_bottom"));

        verticalLayout->addWidget(to_bottom);

        addQuestion = new QPushButton(layoutWidget);
        addQuestion->setObjectName(QStringLiteral("addQuestion"));

        verticalLayout->addWidget(addQuestion);


        horizontalLayout->addLayout(verticalLayout);

        verticalLayout_2 = new QVBoxLayout();
        verticalLayout_2->setSpacing(6);
        verticalLayout_2->setObjectName(QStringLiteral("verticalLayout_2"));
        last_question = new QPushButton(layoutWidget);
        last_question->setObjectName(QStringLiteral("last_question"));

        verticalLayout_2->addWidget(last_question);

        back_to_top = new QPushButton(layoutWidget);
        back_to_top->setObjectName(QStringLiteral("back_to_top"));

        verticalLayout_2->addWidget(back_to_top);

        add_solution = new QPushButton(layoutWidget);
        add_solution->setObjectName(QStringLiteral("add_solution"));

        verticalLayout_2->addWidget(add_solution);


        horizontalLayout->addLayout(verticalLayout_2);

        question_discription = new QGroupBox(centralWidget);
        question_discription->setObjectName(QStringLiteral("question_discription"));
        question_discription->setGeometry(QRect(690, 60, 301, 191));
        textBrowser_question = new QTextBrowser(question_discription);
        textBrowser_question->setObjectName(QStringLiteral("textBrowser_question"));
        textBrowser_question->setGeometry(QRect(20, 20, 261, 161));
        label = new QLabel(centralWidget);
        label->setObjectName(QStringLiteral("label"));
        label->setGeometry(QRect(703, 441, 231, 41));
        bugRecordsClass->setCentralWidget(centralWidget);
        menuBar = new QMenuBar(bugRecordsClass);
        menuBar->setObjectName(QStringLiteral("menuBar"));
        menuBar->setGeometry(QRect(0, 0, 1021, 23));
        menuv0_1 = new QMenu(menuBar);
        menuv0_1->setObjectName(QStringLiteral("menuv0_1"));
        bugRecordsClass->setMenuBar(menuBar);
        mainToolBar = new QToolBar(bugRecordsClass);
        mainToolBar->setObjectName(QStringLiteral("mainToolBar"));
        bugRecordsClass->addToolBar(Qt::TopToolBarArea, mainToolBar);
        statusBar = new QStatusBar(bugRecordsClass);
        statusBar->setObjectName(QStringLiteral("statusBar"));
        bugRecordsClass->setStatusBar(statusBar);

        menuBar->addAction(menuv0_1->menuAction());

        retranslateUi(bugRecordsClass);

        QMetaObject::connectSlotsByName(bugRecordsClass);
    } // setupUi

    void retranslateUi(QMainWindow *bugRecordsClass)
    {
        bugRecordsClass->setWindowTitle(QApplication::translate("bugRecordsClass", "bugRecords", nullptr));
        figure_show->setText(QApplication::translate("bugRecordsClass", "TextLabel", nullptr));
        solution->setTitle(QApplication::translate("bugRecordsClass", "\350\247\243\345\206\263\346\226\271\346\263\225", nullptr));
        function->setTitle(QApplication::translate("bugRecordsClass", "\345\212\237\350\203\275", nullptr));
        next_question->setText(QApplication::translate("bugRecordsClass", "\344\270\213\344\270\200\344\270\252", nullptr));
        to_bottom->setText(QApplication::translate("bugRecordsClass", "\347\233\264\350\207\263\345\272\225\351\203\250", nullptr));
        addQuestion->setText(QApplication::translate("bugRecordsClass", "\346\267\273\345\212\240\351\227\256\351\242\230", nullptr));
        last_question->setText(QApplication::translate("bugRecordsClass", "\344\270\212\344\270\200\344\270\252", nullptr));
        back_to_top->setText(QApplication::translate("bugRecordsClass", "\345\233\236\345\210\260\351\241\266\351\203\250", nullptr));
        add_solution->setText(QApplication::translate("bugRecordsClass", "\346\267\273\345\212\240\350\247\243\345\206\263\346\226\271\346\263\225", nullptr));
        question_discription->setTitle(QApplication::translate("bugRecordsClass", "\351\227\256\351\242\230\346\217\217\350\277\260", nullptr));
        label->setText(QString());
        menuv0_1->setTitle(QApplication::translate("bugRecordsClass", "v0.1", nullptr));
    } // retranslateUi

};

namespace Ui {
    class bugRecordsClass: public Ui_bugRecordsClass {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_BUGRECORDS_H
