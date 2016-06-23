#include "MultiClientWindow.h"
#include <QApplication>
#include <QFile>
int main(int argc, char *argv[])
{
    QApplication a(argc, argv);
    MultiClientWindow w;
    w.show();

    return a.exec();
}
