#ifndef CLIENTWINDOW_H
#define CLIENTWINDOW_H

#include <QMainWindow>
#include <QTcpSocket>
#include <QSettings>

namespace Ui {
class ClientWindow;
}

class ClientWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit ClientWindow(QWidget *parent = 0);
    ~ClientWindow();

private slots:
    void on_pushButton_Connect_clicked();
    void on_pushButton_Disconnect_clicked();
    void on_pushButton_Send_clicked();

    void on_pushButton_Connect_Devices_clicked();
    void on_pushButton_Disconnect_Devices_clicked();
    void on_pushButton_Grab_Devices_clicked();

private slots:
    void plotState(QAbstractSocket::SocketState);
    void newMessageReceived();
    void WriteMessage(QString message);

    void on_pushButton_Grab_Multiple_clicked();

private:
    Ui::ClientWindow *ui;
    QTcpSocket * mySocket;

    void writeSettings();
    void readSettings();
};

#endif // CLIENTWINDOW_H
