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

#include <pcl/compression/octree_pointcloud_compression.h>

#include <string>       // std::string
#include <iostream>     // std::cout
#include <sstream>      // std::stringstream, std::stringbuf

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



    void on_pushButton_SSHReboot_clicked();
    void on_pushButton_SSHUpdate_clicked();
    void on_pushButton_SSHClientCompile_clicked();
    void SSHlog();
    void showProcState(QProcess::ProcessState newState);

    void on_comboBox_activated(const QString &arg1);

    void on_checkBox_savePC_clicked(bool checked);

    void on_pushButton_Register_clicked();


    void on_pushButton_Save_Settings_clicked();

    void on_pushButton_GrabAndTransmit_clicked();


    void on_pushButton_ShowArrived_clicked();

signals:
    void PCtransmitted(PointCloudT::Ptr);

private:
    Ui::ClientWidget *ui;
    QStringList * IPhistory;

    // SSH stuff
    QProcess proc;

    bool PCmode = false;
    QString compressedDataPart;
    pcl::io::OctreePointCloudCompression<PointT>* PointCloudDecoder;

};

#endif // ClientWidget_H