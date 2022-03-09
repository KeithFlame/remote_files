/********************************************************************************
** Form generated from reading UI file 'question_added.ui'
**
** Created by: Qt User Interface Compiler version 5.11.1
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_QUESTION_ADDED_H
#define UI_QUESTION_ADDED_H

#include <QtCore/QVariant>
#include <QtWidgets/QApplication>
#include <QtWidgets/QGroupBox>
#include <QtWidgets/QHBoxLayout>
#include <QtWidgets/QLabel>
#include <QtWidgets/QLineEdit>
#include <QtWidgets/QPushButton>
#include <QtWidgets/QSpacerItem>
#include <QtWidgets/QVBoxLayout>
#include <QtWidgets/QWidget>

QT_BEGIN_NAMESPACE

class Ui_Form
{
public:
    QGroupBox *groupBox;
    QWidget *layoutWidget;
    QVBoxLayout *verticalLayout;
    QHBoxLayout *horizontalLayout;
    QHBoxLayout *horizontalLayout_2;
    QLabel *label_2;
    QSpacerItem *horizontalSpacer;
    QPushButton *screen_print;
    QSpacerItem *horizontalSpacer_2;
    QLabel *label_3;
    QSpacerItem *horizontalSpacer_3;
    QPushButton *open_file;
    QHBoxLayout *horizontalLayout_3;
    QLabel *label;
    QLineEdit *question_discription;
    QWidget *layoutWidget1;
    QHBoxLayout *horizontalLayout_4;
    QPushButton *confirm_added;
    QSpacerItem *horizontalSpacer_4;
    QPushButton *quit_added;
    QLabel *add_manual;

    void setupUi(QWidget *Form)
    {
        if (Form->objectName().isEmpty())
            Form->setObjectName(QStringLiteral("Form"));
        Form->resize(610, 252);
        groupBox = new QGroupBox(Form);
        groupBox->setObjectName(QStringLiteral("groupBox"));
        groupBox->setGeometry(QRect(60, 50, 521, 131));
        layoutWidget = new QWidget(groupBox);
        layoutWidget->setObjectName(QStringLiteral("layoutWidget"));
        layoutWidget->setGeometry(QRect(20, 30, 471, 91));
        verticalLayout = new QVBoxLayout(layoutWidget);
        verticalLayout->setObjectName(QStringLiteral("verticalLayout"));
        verticalLayout->setContentsMargins(0, 0, 0, 0);
        horizontalLayout = new QHBoxLayout();
        horizontalLayout->setObjectName(QStringLiteral("horizontalLayout"));
        horizontalLayout_2 = new QHBoxLayout();
        horizontalLayout_2->setObjectName(QStringLiteral("horizontalLayout_2"));
        label_2 = new QLabel(layoutWidget);
        label_2->setObjectName(QStringLiteral("label_2"));

        horizontalLayout_2->addWidget(label_2);

        horizontalSpacer = new QSpacerItem(40, 20, QSizePolicy::Expanding, QSizePolicy::Minimum);

        horizontalLayout_2->addItem(horizontalSpacer);

        screen_print = new QPushButton(layoutWidget);
        screen_print->setObjectName(QStringLiteral("screen_print"));

        horizontalLayout_2->addWidget(screen_print);

        horizontalSpacer_2 = new QSpacerItem(40, 20, QSizePolicy::Expanding, QSizePolicy::Minimum);

        horizontalLayout_2->addItem(horizontalSpacer_2);

        label_3 = new QLabel(layoutWidget);
        label_3->setObjectName(QStringLiteral("label_3"));

        horizontalLayout_2->addWidget(label_3);

        horizontalSpacer_3 = new QSpacerItem(40, 20, QSizePolicy::Expanding, QSizePolicy::Minimum);

        horizontalLayout_2->addItem(horizontalSpacer_3);

        open_file = new QPushButton(layoutWidget);
        open_file->setObjectName(QStringLiteral("open_file"));

        horizontalLayout_2->addWidget(open_file);


        horizontalLayout->addLayout(horizontalLayout_2);


        verticalLayout->addLayout(horizontalLayout);

        horizontalLayout_3 = new QHBoxLayout();
        horizontalLayout_3->setObjectName(QStringLiteral("horizontalLayout_3"));
        label = new QLabel(layoutWidget);
        label->setObjectName(QStringLiteral("label"));

        horizontalLayout_3->addWidget(label);

        question_discription = new QLineEdit(layoutWidget);
        question_discription->setObjectName(QStringLiteral("question_discription"));

        horizontalLayout_3->addWidget(question_discription);


        verticalLayout->addLayout(horizontalLayout_3);

        layoutWidget1 = new QWidget(Form);
        layoutWidget1->setObjectName(QStringLiteral("layoutWidget1"));
        layoutWidget1->setGeometry(QRect(160, 200, 301, 25));
        horizontalLayout_4 = new QHBoxLayout(layoutWidget1);
        horizontalLayout_4->setObjectName(QStringLiteral("horizontalLayout_4"));
        horizontalLayout_4->setContentsMargins(0, 0, 0, 0);
        confirm_added = new QPushButton(layoutWidget1);
        confirm_added->setObjectName(QStringLiteral("confirm_added"));

        horizontalLayout_4->addWidget(confirm_added);

        horizontalSpacer_4 = new QSpacerItem(40, 20, QSizePolicy::Expanding, QSizePolicy::Minimum);

        horizontalLayout_4->addItem(horizontalSpacer_4);

        quit_added = new QPushButton(layoutWidget1);
        quit_added->setObjectName(QStringLiteral("quit_added"));

        horizontalLayout_4->addWidget(quit_added);

        add_manual = new QLabel(Form);
        add_manual->setObjectName(QStringLiteral("add_manual"));
        add_manual->setGeometry(QRect(80, 20, 421, 31));

        retranslateUi(Form);

        QMetaObject::connectSlotsByName(Form);
    } // setupUi

    void retranslateUi(QWidget *Form)
    {
        Form->setWindowTitle(QApplication::translate("Form", "Form", nullptr));
        groupBox->setTitle(QApplication::translate("Form", "\346\267\273\345\212\240\351\227\256\351\242\230", nullptr));
        label_2->setText(QApplication::translate("Form", "\345\233\276\345\203\217\344\277\241\346\201\257", nullptr));
        screen_print->setText(QApplication::translate("Form", "\346\210\252\345\233\276", nullptr));
        label_3->setText(QApplication::translate("Form", "\346\210\226\350\200\205", nullptr));
        open_file->setText(QApplication::translate("Form", "\346\234\254\345\234\260\344\270\212\344\274\240", nullptr));
        label->setText(QApplication::translate("Form", "\351\227\256\351\242\230\346\217\217\350\277\260", nullptr));
        confirm_added->setText(QApplication::translate("Form", "\347\241\256\350\256\244", nullptr));
        quit_added->setText(QApplication::translate("Form", "\345\217\226\346\266\210", nullptr));
        add_manual->setText(QString());
    } // retranslateUi

};

namespace Ui {
    class question_added : public Ui_Form {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_QUESTION_ADDED_H