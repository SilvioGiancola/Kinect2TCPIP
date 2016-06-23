#ifndef MultiClientWindow_H
#define MultiClientWindow_H

#include <QMainWindow>
#include <QTcpSocket>
#include <QSettings>
#include <QCompleter>

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

private slots:
    void on_pushButton_ConnectALL_clicked();
    void on_pushButton_DisconnectALL_clicked();
    void on_pushButton_GrabALL_clicked();
    void on_pushButton_SendALL_clicked();

private:
    Ui::MultiClientWindow *ui;

    void writeSettings();
    void readSettings();
    QStringList *IPhistory;
 //   QCompleter *IPcompleter;
};

#endif // MultiClientWindow_H
