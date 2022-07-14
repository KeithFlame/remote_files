#include <QtCore/QCoreApplication>
#include "KeithMainWindow.h"

int main(int argc, char *argv[])
{
    QApplication a(argc, argv);
    QtKeithMainWindow w;
    w.show();

    return a.exec();
}
