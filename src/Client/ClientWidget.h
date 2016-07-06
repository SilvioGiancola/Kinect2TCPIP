#ifndef ClientWidget_H
#define ClientWidget_H

#include <QMainWindow>
#include <QTcpSocket>
#include <QSettings>
#include <QDateTime>
#include <QProcess>
#include <QThread>
#include <QCompleter>
#include <QDir>
#include <QTimer>
#include <QStringListModel>

#include <define.h>
#include <Transform.h>


#include <string>       // std::string
#include <iostream>     // std::cout


#include <pcl/io/pcd_io.h>

/// TO DO:
/// ADD Registration handle with fixed number of iteration and at every grab
/// ADD REgistration based on closest BRISK point ?
/// ADD opencv handling (check RTABMAP)



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

    void on_pushButton_SendRepeated_clicked();


    void on_pushButton_SSHReboot_clicked();
    void on_pushButton_SSHUpdate_clicked();
    void SSHlog();
    void showProcState(QProcess::ProcessState newState);
    void on_comboBox_pipeline_activated(const QString &arg1);

    void on_checkBox_savePC_clicked(bool checked);

    void on_pushButton_Register_clicked();


    void on_pushButton_Save_Settings_clicked();

    void on_pushButton_GetPointCloud_clicked();

    void on_transformationWidget_Kin1_matrixchanged(Transform);

    void on_transformationWidget_Kin2_matrixchanged(Transform);

signals:
    void PCtransmitted(PointCloudT::Ptr);

private:
    Ui::ClientWidget *ui;
    QStringList * IPhistory;

    // SSH stuff
    QProcess proc;

    // repeat stuff
    QTimer* timer1;
};

#endif // ClientWidget_H
