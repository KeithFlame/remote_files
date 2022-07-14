/********************************************************************************
** Form generated from reading UI file 'keith.ui'
**
** Created by: Qt User Interface Compiler version 5.11.1
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_KEITH_H
#define UI_KEITH_H

#include <QtCore/QVariant>
#include <QtWidgets/QAction>
#include <QtWidgets/QApplication>
#include <QtWidgets/QLabel>
#include <QtWidgets/QMainWindow>
#include <QtWidgets/QMenu>
#include <QtWidgets/QMenuBar>
#include <QtWidgets/QPushButton>
#include <QtWidgets/QStatusBar>
#include <QtWidgets/QTabWidget>
#include <QtWidgets/QWidget>

QT_BEGIN_NAMESPACE

class Ui_MainWindow
{
public:
    QWidget *centralwidget;
    QTabWidget *tabWidget;
    QWidget *tab;
    QPushButton *pushButton;
    QLabel *label;
    QPushButton *down_pushbutton;
    QPushButton *up_pushbutton;
    QPushButton *right_pushbutton;
    QPushButton *is_rotation;
    QWidget *tab_2;
    QWidget *tab_3;
    QWidget *tab_4;
    QWidget *tab_5;
    QMenuBar *menubar;
    QMenu *menuv1_0;
    QStatusBar *statusbar;

    void setupUi(QMainWindow *MainWindow)
    {
        if (MainWindow->objectName().isEmpty())
            MainWindow->setObjectName(QStringLiteral("MainWindow"));
        MainWindow->resize(1369, 829);
        centralwidget = new QWidget(MainWindow);
        centralwidget->setObjectName(QStringLiteral("centralwidget"));
        tabWidget = new QTabWidget(centralwidget);
        tabWidget->setObjectName(QStringLiteral("tabWidget"));
        tabWidget->setGeometry(QRect(30, 10, 1401, 761));
        tab = new QWidget();
        tab->setObjectName(QStringLiteral("tab"));
        pushButton = new QPushButton(tab);
        pushButton->setObjectName(QStringLiteral("pushButton"));
        pushButton->setGeometry(QRect(20, 630, 71, 23));
        label = new QLabel(tab);
        label->setObjectName(QStringLiteral("label"));
        label->setGeometry(QRect(20, 20, 961, 541));
        down_pushbutton = new QPushButton(tab);
        down_pushbutton->setObjectName(QStringLiteral("down_pushbutton"));
        down_pushbutton->setGeometry(QRect(90, 660, 71, 23));
        up_pushbutton = new QPushButton(tab);
        up_pushbutton->setObjectName(QStringLiteral("up_pushbutton"));
        up_pushbutton->setGeometry(QRect(90, 600, 71, 23));
        right_pushbutton = new QPushButton(tab);
        right_pushbutton->setObjectName(QStringLiteral("right_pushbutton"));
        right_pushbutton->setGeometry(QRect(160, 630, 71, 23));
        is_rotation = new QPushButton(tab);
        is_rotation->setObjectName(QStringLiteral("is_rotation"));
        is_rotation->setGeometry(QRect(100, 630, 51, 23));
        tabWidget->addTab(tab, QString());
        tab_2 = new QWidget();
        tab_2->setObjectName(QStringLiteral("tab_2"));
        tabWidget->addTab(tab_2, QString());
        tab_3 = new QWidget();
        tab_3->setObjectName(QStringLiteral("tab_3"));
        tabWidget->addTab(tab_3, QString());
        tab_4 = new QWidget();
        tab_4->setObjectName(QStringLiteral("tab_4"));
        tabWidget->addTab(tab_4, QString());
        tab_5 = new QWidget();
        tab_5->setObjectName(QStringLiteral("tab_5"));
        tabWidget->addTab(tab_5, QString());
        MainWindow->setCentralWidget(centralwidget);
        menubar = new QMenuBar(MainWindow);
        menubar->setObjectName(QStringLiteral("menubar"));
        menubar->setGeometry(QRect(0, 0, 1369, 23));
        menuv1_0 = new QMenu(menubar);
        menuv1_0->setObjectName(QStringLiteral("menuv1_0"));
        MainWindow->setMenuBar(menubar);
        statusbar = new QStatusBar(MainWindow);
        statusbar->setObjectName(QStringLiteral("statusbar"));
        MainWindow->setStatusBar(statusbar);

        menubar->addAction(menuv1_0->menuAction());

        retranslateUi(MainWindow);

        tabWidget->setCurrentIndex(0);


        QMetaObject::connectSlotsByName(MainWindow);
    } // setupUi

    void retranslateUi(QMainWindow *MainWindow)
    {
        MainWindow->setWindowTitle(QApplication::translate("MainWindow", "MainWindow", nullptr));
        pushButton->setText(QApplication::translate("MainWindow", "\350\247\206\350\247\222\345\267\246\347\247\273", nullptr));
        label->setText(QApplication::translate("MainWindow", "frameflow", nullptr));
        down_pushbutton->setText(QApplication::translate("MainWindow", "\350\247\206\350\247\222\344\270\213\347\247\273", nullptr));
        up_pushbutton->setText(QApplication::translate("MainWindow", "\350\247\206\350\247\222\344\270\212\347\247\273", nullptr));
        right_pushbutton->setText(QApplication::translate("MainWindow", "\350\247\206\350\247\222\345\217\263\347\247\273", nullptr));
        is_rotation->setText(QApplication::translate("MainWindow", "\346\227\213\350\275\254", nullptr));
        tabWidget->setTabText(tabWidget->indexOf(tab), QApplication::translate("MainWindow", "V1", nullptr));
        tabWidget->setTabText(tabWidget->indexOf(tab_2), QApplication::translate("MainWindow", "V2", nullptr));
        tabWidget->setTabText(tabWidget->indexOf(tab_3), QApplication::translate("MainWindow", "V3", nullptr));
        tabWidget->setTabText(tabWidget->indexOf(tab_4), QApplication::translate("MainWindow", "V4", nullptr));
        tabWidget->setTabText(tabWidget->indexOf(tab_5), QApplication::translate("MainWindow", "V5", nullptr));
        menuv1_0->setTitle(QApplication::translate("MainWindow", "v1.0", nullptr));
    } // retranslateUi

};

namespace Ui {
    class MainWindow: public Ui_MainWindow {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_KEITH_H
