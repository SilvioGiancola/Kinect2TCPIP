#ifndef KinectWidget_H
#define KinectWidget_H

#include <QWidget>
#include <QDebug>
#include <QTime>
#include <QDir>

#include "define.h"

#include <pcl/io/pcd_io.h>
#include <pcl/common/io.h> // copy point cloud

#include <libfreenect2/libfreenect2.hpp>
#include <libfreenect2/frame_listener_impl.h>
#include <libfreenect2/registration.h>
#include <libfreenect2/packet_pipeline.h>
#include <libfreenect2/logger.h>



#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/core.hpp>
#include <opencv2/features2d.hpp>

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


    PointCloudT::Ptr TransformPointCloud(cv::Mat rgb, cv::Mat depth);

    bool save;

signals:
    void PCGrabbedsignal(PointCloudT::Ptr);

public slots:
    int OpenKinect();
    int CloseKinect();
    int GrabKinect();
    void TransformationChanged(Transform);
    QString savePC();

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



    cv::Mat mat_depth;
    cv::Mat mat_rgb;
    cv::Mat mat_undistorted;
    cv::Mat mat_registered;
    // libfreenect2::Frame undistorted(512, 424, 4), registered(512, 424, 4);
  /*  libfreenect2::Frame(512, 424, 4) undistorted;
    libfreenect2::Frame(512, 424, 4) registered;*/
    QDateTime timestamp;
    Transform pose;
    PointCloudT::Ptr PC;




};

#endif // KinectWidget_H
