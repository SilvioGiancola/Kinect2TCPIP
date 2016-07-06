#include <QtGlobal>
#if QT_VERSION >= 0x050000
#include <QApplication>
#else
#include <QtGui/QApplication>
#endif

#include "ServerWindow.h"

int main(int argc, char *argv[])
{
    QApplication a(argc, argv);

    ServerWindow w;
    w.showMaximized();

    return a.exec();
}
