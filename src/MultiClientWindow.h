#ifndef MultiClientWindow_H
#define MultiClientWindow_H

#include <QMainWindow>
#include <QTcpSocket>
#include <QSettings>

#include <ClientWindow.h>

#include <define.h>

namespace Ui {
class MultiClientWindow;
}

class MultiClientWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MultiClientWindow(QWidget *parent = 0);
    ~MultiClientWindow();

private:
    Ui::MultiClientWindow *ui;

    void writeSettings();
    void readSettings();
};

#endif // MultiClientWindow_H
