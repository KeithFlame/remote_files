#include "bugRecords.h"
#include <QtWidgets/QApplication>
#include <QTextCodeC>

int main(int argc, char *argv[])
{
    //QTextCodec* codec = QTextCodec::codecForName("utf-8");

    //QTextCodec::setCodecForTr(codec);

    //QTextCodec::setCodecForLocale(codec);

    //QTextCodec::setCodecForCStrings(codec);


    QApplication a(argc, argv);
    bugRecords w;
    w.show();
    return a.exec();
}
