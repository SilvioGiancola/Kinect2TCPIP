#ifndef ClientWidget_H
#define ClientWidget_H

#include <QMainWindow>
#include <QTcpSocket>
#include <QSettings>
#include <QDateTime>
#include <QProcess>

#include <define.h>

namespace Ui {
class ClientWidget;
}

class ClientWidget : public QWidget
{
    Q_OBJECT

public:
    explicit ClientWidget(QWidget *parent = 0);
    ~ClientWidget();

    void setIP(QString str);
    void setPort(QString str);
    QString getIP();
    QString getPort();

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



    void on_checkBox_ExpertMode_toggled(bool checked);
    void on_pushButton_SSHReboot_clicked();
    void on_pushButton_SSHUpdate_clicked();
    void on_pushButton_SSHClientCompile_clicked();

private:
    Ui::ClientWidget *ui;
    QTcpSocket * mySocket;
};

#endif // ClientWidget_H
