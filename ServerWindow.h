#ifndef SERVERWINDOW_H
#define SERVERWINDOW_H

#include <QMainWindow>
#include <QDebug>
#include <QTcpServer>
#include <QTcpSocket>
#include <QNetworkInterface>
#include <QDir>
#include <QThread>
#include <QSettings>
#include <QTime>
#include <QProcess>

#include <define.h>

#include <libfreenect2/libfreenect2.hpp>
#include <libfreenect2/frame_listener_impl.h>
#include <libfreenect2/registration.h>
#include <libfreenect2/packet_pipeline.h>
#include <libfreenect2/logger.h>

#include <pcl/visualization/pcl_visualizer.h> // viewer
#include <vtkRenderWindow.h> // qvtk

namespace Ui {
class ServerWindow;
}

class ServerWindow : public QMainWindow
{
    Q_OBJECT
    
public:
    explicit ServerWindow(QWidget *parent = 0);
    ~ServerWindow();


    int OpenKinect(int i);
    int GrabKinect(int i);
    int CloseKinect(int i);

private slots:

    void newTCPIPConnection();
    void newMessageReceived();
    void clientStateChanged(QAbstractSocket::SocketState);

    void showPC(PointCloudT::Ptr);


    void on_pushButton_Open_kin1_clicked();
    void on_pushButton_Close_kin1_clicked();
    void on_pushButton_Grab_kin1_clicked();

    void on_pushButton_Open_kin2_clicked();
    void on_pushButton_Close_kin2_clicked();
    void on_pushButton_Grab_kin2_clicked();


    void on_comboBox_KinectSerials_kin1_currentIndexChanged(const QString &arg1);
    void on_comboBox_KinectSerials_kin2_currentIndexChanged(const QString &arg1);

    //  void on_comboBox_pipeline_currentIndexChanged(const QString &arg1);

    void on_comboBox_pipeline_kin1_currentIndexChanged(const QString &arg1);
    void on_comboBox_pipeline_kin2_currentIndexChanged(const QString &arg1);

    void on_comboBox_log_currentIndexChanged(const QString &arg1);

signals:
    void PCGrabbedsignal(PointCloudT::Ptr);


private:
    Ui::ServerWindow *ui;

    // TCP IP stuff
    QTcpServer *server;
    QTcpSocket *socket;
    int _port;

    // pcl 3D Viewer
    pcl::visualization::PCLVisualizer::Ptr viewer;

    // Libfreenect2
    libfreenect2::Freenect2 freenect2;
    QList<libfreenect2::Freenect2Device*> dev;
    QList<libfreenect2::SyncMultiFrameListener*> listener;
    QList<libfreenect2::Registration*> registration;
    QList<libfreenect2::PacketPipeline*> pipeline;
    QList<QString> serials;



    void writeSettings();
    void readSettings();


};
#endif // SERVERWINDOW_H
