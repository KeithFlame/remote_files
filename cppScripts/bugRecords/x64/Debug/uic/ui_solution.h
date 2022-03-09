/********************************************************************************
** Form generated from reading UI file 'solution.ui'
**
** Created by: Qt User Interface Compiler version 5.11.1
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_SOLUTION_H
#define UI_SOLUTION_H

#include <QtCore/QVariant>
#include <QtWidgets/QApplication>
#include <QtWidgets/QGroupBox>
#include <QtWidgets/QLabel>
#include <QtWidgets/QLineEdit>
#include <QtWidgets/QPushButton>
#include <QtWidgets/QWidget>

QT_BEGIN_NAMESPACE

class Ui_Form
{
public:
    QGroupBox *groupBox;
    QLineEdit *lineEdit;
    QPushButton *confirm_add_solution;
    QPushButton *close_window;
    QLabel *add_manual;

    void setupUi(QWidget *Form)
    {
        if (Form->objectName().isEmpty())
            Form->setObjectName(QStringLiteral("Form"));
        Form->resize(504, 345);
        groupBox = new QGroupBox(Form);
        groupBox->setObjectName(QStringLiteral("groupBox"));
        groupBox->setGeometry(QRect(20, 20, 451, 201));
        lineEdit = new QLineEdit(groupBox);
        lineEdit->setObjectName(QStringLiteral("lineEdit"));
        lineEdit->setGeometry(QRect(20, 20, 391, 161));
        confirm_add_solution = new QPushButton(Form);
        confirm_add_solution->setObjectName(QStringLiteral("confirm_add_solution"));
        confirm_add_solution->setGeometry(QRect(380, 240, 75, 23));
        close_window = new QPushButton(Form);
        close_window->setObjectName(QStringLiteral("close_window"));
        close_window->setGeometry(QRect(380, 280, 75, 23));
        add_manual = new QLabel(Form);
        add_manual->setObjectName(QStringLiteral("add_manual"));
        add_manual->setGeometry(QRect(60, 240, 251, 61));

        retranslateUi(Form);

        QMetaObject::connectSlotsByName(Form);
    } // setupUi

    void retranslateUi(QWidget *Form)
    {
        Form->setWindowTitle(QApplication::translate("Form", "Form", nullptr));
        groupBox->setTitle(QApplication::translate("Form", "\350\276\223\345\205\245\350\247\243\345\206\263\346\226\271\346\263\225", nullptr));
        confirm_add_solution->setText(QApplication::translate("Form", "\347\241\256\350\256\244", nullptr));
        close_window->setText(QApplication::translate("Form", "\345\205\263\351\227\255", nullptr));
        add_manual->setText(QApplication::translate("Form", "TextLabel", nullptr));
    } // retranslateUi

};

namespace Ui {
    class Form: public Ui_Form {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_SOLUTION_H
