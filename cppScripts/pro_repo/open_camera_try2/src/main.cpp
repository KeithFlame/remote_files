#include "open_camera_try2.h"
#include <QtWidgets/QApplication>

int main(int argc, char *argv[])
{
    QApplication a(argc, argv);
    CameraManipulation w;
    w.show();
    return a.exec();
}
