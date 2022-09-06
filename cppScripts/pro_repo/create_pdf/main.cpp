//#include "create_pdf.h"
#include "generation_report.h"
#include <QtWidgets/QApplication>

int main(int argc, char *argv[])
{
    QApplication a(argc, argv);
    GenRep w;
    w.show();
    return a.exec();
}