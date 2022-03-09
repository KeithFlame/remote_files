/********************************************************************************
** Form generated from reading UI file 'solution_added.ui'
**
** Created by: Qt User Interface Compiler version 5.11.1
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_SOLUTION_ADDED_H
#define UI_SOLUTION_ADDED_H

#include <QtCore/QVariant>
#include <QtWidgets/QApplication>
#include <QtWidgets/QGroupBox>
#include <QtWidgets/QLabel>
#include <QtWidgets/QLineEdit>
#include <QtWidgets/QPushButton>
#include <QtWidgets/QWidget>

QT_BEGIN_NAMESPACE

class Ui_solution_added
{
public:
    QPushButton *close_window;
    QPushButton *confirm_add_solution;
    QGroupBox *groupBox;
    QLineEdit *lineEdit;
    QLabel *add_manual;

    void setupUi(QWidget *solution_added)
    {
        if (solution_added->objectName().isEmpty())
            solution_added->setObjectName(QStringLiteral("solution_added"));
        solution_added->resize(482, 337);
        close_window = new QPushButton(solution_added);
        close_window->setObjectName(QStringLiteral("close_window"));
        close_window->setGeometry(QRect(370, 280, 75, 23));
        confirm_add_solution = new QPushButton(solution_added);
        confirm_add_solution->setObjectName(QStringLiteral("confirm_add_solution"));
        confirm_add_solution->setGeometry(QRect(370, 240, 75, 23));
        groupBox = new QGroupBox(solution_added);
        groupBox->setObjectName(QStringLiteral("groupBox"));
        groupBox->setGeometry(QRect(10, 20, 451, 201));
        lineEdit = new QLineEdit(groupBox);
        lineEdit->setObjectName(QStringLiteral("lineEdit"));
        lineEdit->setGeometry(QRect(20, 20, 391, 161));
        add_manual = new QLabel(solution_added);
        add_manual->setObjectName(QStringLiteral("add_manual"));
        add_manual->setGeometry(QRect(50, 240, 251, 61));

        retranslateUi(solution_added);

        QMetaObject::connectSlotsByName(solution_added);
    } // setupUi

    void retranslateUi(QWidget *solution_added)
    {
        solution_added->setWindowTitle(QApplication::translate("solution_added", "solution_added", nullptr));
        close_window->setText(QApplication::translate("solution_added", "\345\205\263\351\227\255", nullptr));
        confirm_add_solution->setText(QApplication::translate("solution_added", "\347\241\256\350\256\244", nullptr));
        groupBox->setTitle(QApplication::translate("solution_added", "\350\276\223\345\205\245\350\247\243\345\206\263\346\226\271\346\263\225", nullptr));
        add_manual->setText(QString());
    } // retranslateUi

};

namespace Ui {
    class solution_added: public Ui_solution_added {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_SOLUTION_ADDED_H
