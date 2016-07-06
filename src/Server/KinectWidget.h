#ifndef KinectWidget_H
#define KinectWidget_H

#include <QWidget>
#include <QDebug>
#include <QTime>

#include "define.h"

#include <pcl/common/io.h> // copy point cloud

#include <libfreenect2/libfreenect2.hpp>
#include <libfreenect2/frame_listener_impl.h>
#include <libfreenect2/registration.h>
#include <libfreenect2/packet_pipeline.h>
#include <libfreenect2/logger.h>

namespace Ui {
class KinectWidget;
}

class KinectWidget : public QWidget
{
    Q_OBJECT

public:
    explicit KinectWidget(QWidget *parent = 0);
    ~KinectWidget();

    void setPipeline(QString Pipe);
    void setSerial(QString serial);
    void setTransform(Transform transf);

    QString getPipeline();
    QString getSerial();
    Transform getTransform();
    PointCloudT::Ptr getPointCloud();
    PointCloudNormalT::Ptr getPointCloudNormal();

    bool isClosed(){return (dev==0);}
    bool isOpened(){return !isClosed();}


signals:
    void PCGrabbedsignal(PointCloudT::Ptr);

public slots:
    int OpenKinect();
    int CloseKinect();
    int GrabKinect();
    void TransformationChanged(Transform);

private slots:
    void on_comboBox_pipeline_currentIndexChanged(const QString &arg1);
    void on_comboBox_KinectSerials_currentIndexChanged(const QString &arg1);

private:
    Ui::KinectWidget *ui;

    // Libfreenect2
    libfreenect2::Freenect2 freenect2;
    libfreenect2::Freenect2Device* dev;
    libfreenect2::SyncMultiFrameListener* listener;
    libfreenect2::Registration* registration;
    libfreenect2::PacketPipeline* pipeline;
    QString serial;

    PointCloudT::Ptr PC;
};

#endif // KinectWidget_H
