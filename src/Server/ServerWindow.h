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

#include <pcl/io/pcd_io.h>
#include <pcl/common/transforms.h>  // transform point cloud
#include <pcl/common/io.h> // copy point cloud

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/openni_grabber.h>
#include <pcl/visualization/cloud_viewer.h>


#include <stdio.h>
#include <sstream>
#include <stdlib.h>



namespace Ui {
class ServerWindow;
}

class ServerWindow : public QMainWindow
{
    Q_OBJECT
    
public:
    explicit ServerWindow(QWidget *parent = 0);
    ~ServerWindow();

private slots:
    void newTCPIPConnection();
    void newMessageReceived();
    void clientStateChanged(QAbstractSocket::SocketState);

    PointCloudT::Ptr getPointCloud(int i);
 //   QString savePC(PointCloudT::Ptr);

    void on_comboBox_log_currentIndexChanged(const QString &arg1);
    void on_checkBox_save_toggled(bool checked);
    void on_pushButton_registrer_clicked();


private:
    Ui::ServerWindow *ui;

    // TCP IP stuff
    QTcpServer *server;
    QTcpSocket *socket;
    int _port;


    void writeSettings();
    void readSettings();



};
#endif // SERVERWINDOW_H
