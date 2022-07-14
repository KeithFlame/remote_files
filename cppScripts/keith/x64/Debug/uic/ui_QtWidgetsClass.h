/********************************************************************************
** Form generated from reading UI file 'QtWidgetsClass.ui'
**
** Created by: Qt User Interface Compiler version 5.11.1
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_QTWIDGETSCLASS_H
#define UI_QTWIDGETSCLASS_H

#include <QtCore/QVariant>
#include <QtWidgets/QApplication>
#include <QtWidgets/QHBoxLayout>
#include <QtWidgets/QLabel>
#include <QtWidgets/QMainWindow>
#include <QtWidgets/QMenuBar>
#include <QtWidgets/QPushButton>
#include <QtWidgets/QSpacerItem>
#include <QtWidgets/QStatusBar>
#include <QtWidgets/QTextBrowser>
#include <QtWidgets/QToolBar>
#include <QtWidgets/QVBoxLayout>
#include <QtWidgets/QWidget>

QT_BEGIN_NAMESPACE

class Ui_QtWidgetsClassClass
{
public:
    QWidget *centralWidget;
    QLabel *backgroud;
    QTextBrowser *textBrowser;
    QTextBrowser *textBrowser_2;
    QWidget *widget;
    QVBoxLayout *verticalLayout;
    QLabel *accont;
    QLabel *accont_2;
    QWidget *widget1;
    QHBoxLayout *horizontalLayout;
    QPushButton *load_command;
    QSpacerItem *horizontalSpacer;
    QPushButton *password_foggoten;
    QMenuBar *menuBar;
    QToolBar *mainToolBar;
    QStatusBar *statusBar;

    void setupUi(QMainWindow *QtWidgetsClassClass)
    {
        if (QtWidgetsClassClass->objectName().isEmpty())
            QtWidgetsClassClass->setObjectName(QStringLiteral("QtWidgetsClassClass"));
        QtWidgetsClassClass->resize(600, 400);
        centralWidget = new QWidget(QtWidgetsClassClass);
        centralWidget->setObjectName(QStringLiteral("centralWidget"));
        backgroud = new QLabel(centralWidget);
        backgroud->setObjectName(QStringLiteral("backgroud"));
        backgroud->setGeometry(QRect(10, 10, 581, 331));
        textBrowser = new QTextBrowser(centralWidget);
        textBrowser->setObjectName(QStringLiteral("textBrowser"));
        textBrowser->setGeometry(QRect(240, 90, 151, 31));
        textBrowser_2 = new QTextBrowser(centralWidget);
        textBrowser_2->setObjectName(QStringLiteral("textBrowser_2"));
        textBrowser_2->setGeometry(QRect(240, 130, 151, 31));
        widget = new QWidget(centralWidget);
        widget->setObjectName(QStringLiteral("widget"));
        widget->setGeometry(QRect(180, 90, 38, 71));
        verticalLayout = new QVBoxLayout(widget);
        verticalLayout->setSpacing(6);
        verticalLayout->setContentsMargins(11, 11, 11, 11);
        verticalLayout->setObjectName(QStringLiteral("verticalLayout"));
        verticalLayout->setContentsMargins(0, 0, 0, 0);
        accont = new QLabel(widget);
        accont->setObjectName(QStringLiteral("accont"));

        verticalLayout->addWidget(accont);

        accont_2 = new QLabel(widget);
        accont_2->setObjectName(QStringLiteral("accont_2"));

        verticalLayout->addWidget(accont_2);

        widget1 = new QWidget(centralWidget);
        widget1->setObjectName(QStringLiteral("widget1"));
        widget1->setGeometry(QRect(180, 180, 211, 25));
        horizontalLayout = new QHBoxLayout(widget1);
        horizontalLayout->setSpacing(6);
        horizontalLayout->setContentsMargins(11, 11, 11, 11);
        horizontalLayout->setObjectName(QStringLiteral("horizontalLayout"));
        horizontalLayout->setContentsMargins(0, 0, 0, 0);
        load_command = new QPushButton(widget1);
        load_command->setObjectName(QStringLiteral("load_command"));

        horizontalLayout->addWidget(load_command);

        horizontalSpacer = new QSpacerItem(40, 20, QSizePolicy::Expanding, QSizePolicy::Minimum);

        horizontalLayout->addItem(horizontalSpacer);

        password_foggoten = new QPushButton(widget1);
        password_foggoten->setObjectName(QStringLiteral("password_foggoten"));

        horizontalLayout->addWidget(password_foggoten);

        QtWidgetsClassClass->setCentralWidget(centralWidget);
        menuBar = new QMenuBar(QtWidgetsClassClass);
        menuBar->setObjectName(QStringLiteral("menuBar"));
        menuBar->setGeometry(QRect(0, 0, 600, 23));
        QtWidgetsClassClass->setMenuBar(menuBar);
        mainToolBar = new QToolBar(QtWidgetsClassClass);
        mainToolBar->setObjectName(QStringLiteral("mainToolBar"));
        QtWidgetsClassClass->addToolBar(Qt::TopToolBarArea, mainToolBar);
        statusBar = new QStatusBar(QtWidgetsClassClass);
        statusBar->setObjectName(QStringLiteral("statusBar"));
        QtWidgetsClassClass->setStatusBar(statusBar);

        retranslateUi(QtWidgetsClassClass);

        QMetaObject::connectSlotsByName(QtWidgetsClassClass);
    } // setupUi

    void retranslateUi(QMainWindow *QtWidgetsClassClass)
    {
        QtWidgetsClassClass->setWindowTitle(QApplication::translate("QtWidgetsClassClass", "QtWidgetsClass", nullptr));
        backgroud->setText(QApplication::translate("QtWidgetsClassClass", "TextLabel", nullptr));
        accont->setText(QApplication::translate("QtWidgetsClassClass", "\350\264\246\345\217\267\357\274\232", nullptr));
        accont_2->setText(QApplication::translate("QtWidgetsClassClass", "\345\257\206\347\240\201\357\274\232", nullptr));
        load_command->setText(QApplication::translate("QtWidgetsClassClass", "\347\241\256\350\256\244", nullptr));
        password_foggoten->setText(QApplication::translate("QtWidgetsClassClass", "\346\211\276\345\233\236\345\257\206\347\240\201", nullptr));
    } // retranslateUi

};

namespace Ui {
    class QtWidgetsClassClass: public Ui_QtWidgetsClassClass {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_QTWIDGETSCLASS_H
