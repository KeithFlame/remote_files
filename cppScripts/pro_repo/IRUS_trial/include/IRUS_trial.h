#pragma once

#include <QtWidgets/QMainWindow>
#include "ui_IRUS_trial.h"

class IRUS_trial : public QMainWindow
{
    Q_OBJECT

public:
    IRUS_trial(QWidget *parent = nullptr);
    ~IRUS_trial();

private:
    Ui::IRUS_trialClass ui;
};
