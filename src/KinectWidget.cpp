#include "KinectWidget.h"
#include "ui_KinectWidget.h"

KinectWidget::KinectWidget(QWidget *parent) :
    QWidget(parent),
    ui(new Ui::KinectWidget)
{
    ui->setupUi(this);


    serial = QString("");
    dev = 0;
    listener = new libfreenect2::SyncMultiFrameListener(libfreenect2::Frame::Color | libfreenect2::Frame::Ir | libfreenect2::Frame::Depth);
    registration = new libfreenect2::Registration(libfreenect2::Freenect2Device::IrCameraParams(), libfreenect2::Freenect2Device::ColorCameraParams());
    pipeline = new libfreenect2::CpuPacketPipeline();


    // define serial numbers
    int n = freenect2.enumerateDevices();
    for (int i = 0; i < n; i++)
        ui->comboBox_KinectSerials->addItem(QString::fromStdString(freenect2.getDeviceSerialNumber(i)));


#ifndef LIBFREENECT2_WITH_OPENGL_SUPPORT
    ui->comboBox_pipeline->model()->setData(ui->comboBox_pipeline->model()->index(1,0), 0, Qt::UserRole - 1);
#endif
#ifndef LIBFREENECT2_WITH_OPENCL_SUPPORT
    ui->comboBox_pipeline->model()->setData(ui->comboBox_pipeline->model()->index(2,0), 0, Qt::UserRole - 1);
#endif
#ifndef LIBFREENECT2_WITH_CUDA_SUPPORT
    ui->comboBox_pipeline->model()->setData(ui->comboBox_pipeline->model()->index(3,0), 0, Qt::UserRole - 1);
#endif

    // PointCloud
    PC.reset(new PointCloudT);
    PC->resize(512*424); // set the memory size to allocate
    PC->height = 424;        // set the height
    PC->width = 512;          // set the width
    PC->is_dense = false;                   // Kinect V2 returns organized and not dense point clouds
}

KinectWidget::~KinectWidget()
{
    CloseKinect();
    delete ui;
}



// KINECT

int KinectWidget::OpenKinect()
{
    if (isOpened())
    {
        qWarning() << "Kinect is already open";
        return ERROR;
    }


    if (serial.isEmpty())
    {
        qWarning() << tr("Error in opening Kinect, please define a serial number");
        return ERROR;
    }


    dev = freenect2.openDevice(serial.toStdString(), pipeline);



    if(isClosed())
    {
        qWarning() << tr("Error in opening Kinect, failure opening the default one");
        return ERROR;
    }


    dev->setColorFrameListener(listener);
    dev->setIrAndDepthFrameListener(listener);
    dev->start();


    // Color
    registration = new libfreenect2::Registration(dev->getIrCameraParams(), dev->getColorCameraParams());


    return SUCCESS;
}
int KinectWidget::GrabKinect()
{
    if (isClosed())
    {
        qWarning() << "Kinect stream is not open";
        return ERROR;
    }


    QTime t;
    t.start();



    // Acquire Frames
    libfreenect2::FrameMap frames;
    listener->waitForNewFrame(frames);

    QDateTime timestamp = QDateTime::currentDateTime();

    libfreenect2::Frame *rgb = frames[libfreenect2::Frame::Color];
    libfreenect2::Frame *depth = frames[libfreenect2::Frame::Depth];



    // Undistort and register frames
    libfreenect2::Frame undistorted(512, 424, 4), registered(512, 424, 4);
    registration->apply(rgb,depth,&undistorted,&registered);

    const float *undistorted_data = (float*)undistorted.data;
    const unsigned int *registered_data = (unsigned int*)registered.data;

    listener->release(frames);



    // Initialize my Point Cloud
  //  PC.reset(new PointCloudT());
    PC->header.stamp = timestamp.toMSecsSinceEpoch();                               // the stamp correspond to the acquisition time
    //  PC->header.frame_id = QString("%1/PointClouds/Kinect%2_%3.pcd").arg(QDir::homePath()).arg(_serial.c_str()).arg(timestamp.toString("yyyy-MM-dd-HH:mm:ss:zzz")).toStdString();
    PC->header.frame_id = serial.toStdString();




    libfreenect2::Freenect2Device::IrCameraParams IRparam = dev->getIrCameraParams();

    // Set data into my Point cloud
    for (unsigned int i = 0; i < undistorted.height ;i++)
    {
        for (unsigned int j = 0; j < undistorted.width ;j++)
        {
            int index = i * undistorted.width + j;

            float depth = undistorted_data[index] / 1000.0f;
            unsigned int rgba = registered_data[index];

            PointT P;

            if ( depth != 0 && rgba != 0)
            {
                P.x = -depth * (IRparam.cx - j) / IRparam.fx;
                P.y =  depth * (IRparam.cy - i) / IRparam.fy;
                P.z =  depth;

                P.a = (rgba >> 24) & 0xFF;
                P.r = (rgba >> 16) & 0xFF;
                P.g = (rgba >> 8)  & 0xFF;
                P.b =  rgba        & 0xFF;
            }
            else
            {
                P.x = P.y = P.z = std::numeric_limits<float>::quiet_NaN();
            }

            PC->at(j,i) = P;
        }
    }


    emit PCGrabbedsignal(PC);

    return SUCCESS;
}
int KinectWidget::CloseKinect()
{
    if (isClosed())
    {
        qWarning() << "Kinect already closed";
        return ERROR;
    }

    dev->stop();
    dev->close();
    dev = 0;

    return SUCCESS;
}

void  KinectWidget::TransformationChanged(Transform trans)
{
    PC->sensor_orientation_ = trans.getQuaternion();
    PC->sensor_origin_ = trans.getOrigin4();
    GrabKinect();
}


// get Set Parameters
void KinectWidget::setPipeline(QString string)      {ui->comboBox_pipeline->setCurrentIndex(ui->comboBox_pipeline->findText(string));}
void KinectWidget::setSerial(QString string)        {ui->comboBox_KinectSerials->setCurrentIndex(ui->comboBox_KinectSerials->findText(string));}
void KinectWidget::setTransform(Transform transf)   {ui->myTransformationWidget->setTransform(transf);}


QString KinectWidget::getPipeline()     {return ui->comboBox_pipeline->currentText();}
QString KinectWidget::getSerial()       {return ui->comboBox_KinectSerials->currentText();}
Transform KinectWidget::getTransform()  {return ui->myTransformationWidget->getTransform();}
PointCloudT::Ptr KinectWidget::getPointCloud(){ PointCloudT::Ptr _PC; pcl::copyPointCloud(*PC, *_PC); return _PC;}


// Event Handling
void KinectWidget::on_comboBox_pipeline_currentIndexChanged(const QString &arg1)
{
    if (arg1.compare("Cpu") == 0)           pipeline = new libfreenect2::CpuPacketPipeline();
#ifdef LIBFREENECT2_WITH_OPENGL_SUPPORT
    else if (arg1.compare("OpenGL") == 0)   pipeline = new libfreenect2::OpenGLPacketPipeline();
#endif
#ifdef LIBFREENECT2_WITH_OPENCL_SUPPORT
    else if (arg1.compare("OpenCL") == 0)   pipeline = new libfreenect2::OpenCLPacketPipeline();
#endif
    if (arg1.compare("Cpu") == 0)           pipeline = new libfreenect2::CpuPacketPipeline();
    else if (arg1.compare("OpenGL") == 0)   pipeline = new libfreenect2::OpenGLPacketPipeline();
    else if (arg1.compare("OpenCL") == 0)   pipeline = new libfreenect2::OpenCLPacketPipeline();
#ifdef LIBFREENECT2_WITH_CUDA_SUPPORT
    else if (arg1.compare("Cuda") == 0)     pipeline = new libfreenect2::CudaPacketPipeline();
#endif
}
void KinectWidget::on_comboBox_KinectSerials_currentIndexChanged(const QString &arg1)
{
    serial = arg1;
}



