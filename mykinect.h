#ifndef MYKINECT_H
#define MYKINECT_H



#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

#include <libfreenect2/libfreenect2.hpp>
#include <libfreenect2/frame_listener_impl.h>
#include <libfreenect2/registration.h>
#include <libfreenect2/packet_pipeline.h>
#include <libfreenect2/logger.h>

#include <QTime>
#include <QDir>
#include <QString>
#include <QDebug>

#include <define.h>


class MyKinect : public QObject
{
    Q_OBJECT

public:
    MyKinect(std::string serial = "");

    int Open();
    int Close();
    int Grab();

    bool _open;
    bool _play;
    bool _save;

    QString getSerial()    {        return QString::fromStdString(_serial);    }
    void setSerial(QString str)    {        _serial = str.toStdString();    }

    QStringList getSerialList()
    {
        QStringList list;
        for (int i = 0; i < freenect2.enumerateDevices(); i++)
            list.append(QString::fromStdString(freenect2.getDeviceSerialNumber(i)));

        return list;
    }

signals:
    void PCGrabbedsignal(PointCloudT::Ptr);

private:
    libfreenect2::Freenect2 freenect2;
    libfreenect2::Freenect2Device *dev;
    libfreenect2::SyncMultiFrameListener *listener;
    libfreenect2::Registration *registration;
    libfreenect2::PacketPipeline *pipeline;


    std::string _serial;



    PointCloudT::Ptr PC;

};

#endif // MYKINECT_H
