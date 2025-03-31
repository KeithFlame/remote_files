#include "IRUS_trial.h"
#include <QtWidgets/QApplication>

int main(int argc, char *argv[])
{
    QApplication a(argc, argv);
    IRUS_trial w;
    w.show();
    return a.exec();
}
