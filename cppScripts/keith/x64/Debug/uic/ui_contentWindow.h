/********************************************************************************
** Form generated from reading UI file 'contentWindow.ui'
**
** Created by: Qt User Interface Compiler version 5.11.1
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_CONTENTWINDOW_H
#define UI_CONTENTWINDOW_H

#include <QtCore/QVariant>
#include <QtWidgets/QApplication>
#include <QtWidgets/QMainWindow>
#include <QtWidgets/QMenuBar>
#include <QtWidgets/QStatusBar>
#include <QtWidgets/QToolBar>
#include <QtWidgets/QWidget>

QT_BEGIN_NAMESPACE

class Ui_contentWindowClass
{
public:
    QMenuBar *menuBar;
    QToolBar *mainToolBar;
    QWidget *centralWidget;
    QStatusBar *statusBar;

    void setupUi(QMainWindow *contentWindowClass)
    {
        if (contentWindowClass->objectName().isEmpty())
            contentWindowClass->setObjectName(QStringLiteral("contentWindowClass"));
        contentWindowClass->resize(600, 400);
        menuBar = new QMenuBar(contentWindowClass);
        menuBar->setObjectName(QStringLiteral("menuBar"));
        contentWindowClass->setMenuBar(menuBar);
        mainToolBar = new QToolBar(contentWindowClass);
        mainToolBar->setObjectName(QStringLiteral("mainToolBar"));
        contentWindowClass->addToolBar(mainToolBar);
        centralWidget = new QWidget(contentWindowClass);
        centralWidget->setObjectName(QStringLiteral("centralWidget"));
        contentWindowClass->setCentralWidget(centralWidget);
        statusBar = new QStatusBar(contentWindowClass);
        statusBar->setObjectName(QStringLiteral("statusBar"));
        contentWindowClass->setStatusBar(statusBar);

        retranslateUi(contentWindowClass);

        QMetaObject::connectSlotsByName(contentWindowClass);
    } // setupUi

    void retranslateUi(QMainWindow *contentWindowClass)
    {
        contentWindowClass->setWindowTitle(QApplication::translate("contentWindowClass", "contentWindow", nullptr));
    } // retranslateUi

};

namespace Ui {
    class contentWindowClass: public Ui_contentWindowClass {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_CONTENTWINDOW_H
