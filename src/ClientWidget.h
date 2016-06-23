#ifndef ClientWidget_H
#define ClientWidget_H

#include <QMainWindow>
#include <QTcpSocket>
#include <QSettings>
#include <QDateTime>
#include <QProcess>
#include <QThread>
#include <QCompleter>
#include <QStringListModel>

#include <define.h>


/// TO DO:
/// ADD position matrix handle -> Can be saved in PC and can be opened / saved from 7 doubles
/// ADD Registration handle with fixed number of iteration and at every grab
/// ADD REgistration based on closest BRISK point ?
/// ADD opencv handling (check RTABMAP)
/// ADD Grab Multi index kinect in parallel


namespace Ui {
class ClientWidget;
}

class ClientWidget : public QWidget
{
    Q_OBJECT

public:
    explicit ClientWidget(QWidget *parent = 0);
    ~ClientWidget();

    void setIPCompletion(QStringList *strList);
    void setIP(QString str);
    void setPort(QString str);
    QString getIP();
    QString getPort();
    void setMessage(QString str);
    QString getLastMessage();
    QTcpSocket * mySocket;

public slots:
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

    void on_comboBox_activated(const QString &arg1);

    void on_checkBox_savePC_clicked(bool checked);

private:
    Ui::ClientWidget *ui;
    QStringList * IPhistory;

};

#endif // ClientWidget_H
