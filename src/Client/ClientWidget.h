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

    PointCloudT::Ptr getPointCloud(int index);
    void setPointCloud(int index, PointCloudT::Ptr PC);
    Transform getPointCloudPose(int index);
    void setPointCloudPose(int index, Transform T);



    Ui::ClientWidget *ui;

public slots:
    void on_pushButton_Connect_clicked();
    void on_pushButton_Disconnect_clicked();
    void on_pushButton_Send_clicked();

    void on_pushButton_Connect_Devices_clicked();
    void on_pushButton_Disconnect_Devices_clicked();
    void on_pushButton_Grab_Devices_clicked();

    void on_pushButton_GetPointCloud_clicked();

    void setOffsetCampata(Transform t)
    {
        offset_campata = t;

        offset_campata.print();;
    }

private slots:
    void plotState(QAbstractSocket::SocketState);
    void newMessageReceived();
    void WriteMessage(QString message);
    QString readAnswer();
    QString WriteMessageAndWaitForAnswer(QString message);



    void on_pushButton_SSHReboot_clicked();
    void on_pushButton_SSHUpdate_clicked();
    void SSHlog();
    void runProc(QString cmdline);
    void showProcState(QProcess::ProcessState newState);
    void on_comboBox_pipeline_activated(const QString &arg1);

    void on_checkBox_savePC_clicked(bool checked);

    void on_pushButton_Register_clicked();


    void on_pushButton_Save_Settings_clicked();


    void on_transformationWidget_Kin1_matrixchanged(Transform);

    void on_transformationWidget_Kin2_matrixchanged(Transform);


    void MovePointCloud1(Transform trans)
    {
        emit sig_PointCloudMoved(cloud0->header.frame_id, trans);
        return;
    }
    void MovePointCloud2(Transform trans)
    {
        emit sig_PointCloudMoved(cloud1->header.frame_id, trans);
        return;
    }



    void on_pushButton_SSHTime_clicked();

    void on_pushButton_SSHCustom_clicked();

signals:
    void PCtransmitted(PointCloudT::Ptr);
    void sig_PointCloudMoved(std::string, Transform);

private:
    QStringList * IPhistory;

    // SSH stuff
    QProcess proc;



    PointCloudT::Ptr cloud0;
    PointCloudT::Ptr cloud1;

   Transform offset_campata;
};

#endif // ClientWidget_H
