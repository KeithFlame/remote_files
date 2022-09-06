#pragma once

#include <QtWidgets/QMainWindow>
#include "ui_create_pdf.h"

class create_pdf : public QMainWindow
{
    Q_OBJECT

public:
    create_pdf(QWidget *parent = nullptr);
    ~create_pdf();

private:
    Ui::create_pdfClass ui;
};
