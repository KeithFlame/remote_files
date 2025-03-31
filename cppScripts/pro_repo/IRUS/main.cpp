#include "DoM_pro.h"
#include <QtWidgets/QApplication>

int main(int argc, char *argv[])
{
    QApplication a(argc, argv);
    DoM_pm w;
    w.show();
    return a.exec();
}
