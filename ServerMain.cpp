#include <QtGui/QApplication>
#include "ServerWindow.h"

int main(int argc, char *argv[])
{
    QApplication a(argc, argv);

    ServerWindow w;
    w.showMaximized();

    return a.exec();
}
