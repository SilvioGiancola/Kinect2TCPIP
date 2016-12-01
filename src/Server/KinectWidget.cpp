#include "KinectWidget.h"
#include "ui_KinectWidget.h"

KinectWidget::KinectWidget(QWidget *parent) :
    QWidget(parent),
    ui(new Ui::KinectWidget)
{
    save = false;
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
    /*  PC.reset(new PointCloudT);
    PC->resize(512*424); // set the memory size to allocate
    PC->height = 424;        // set the height
    PC->width = 512;          // set the width
    PC->is_dense = false;                   // Kinect V2 returns organized and not dense point clouds*/


    //libfreenect2::Frame _undistorted(512, 424, 4), _registered(512, 424, 4);

    // undistorted =  libfreenect2::Frame(512, 424, 4);
    //  registered = libfreenect2::Frame(512, 424, 4);

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


    libfreenect2::Freenect2Device::Config config;
    config.EnableBilateralFilter = ui->checkBox_BilateralFilter->isChecked();
    config.EnableEdgeAwareFilter = ui->checkBox_EdgeAwareFiltering->isChecked();
    config.MinDepth = ui->doubleSpinBox_Min_Depth->value();
    config.MaxDepth = ui->doubleSpinBox_Max_Depth->value();
    dev->setConfiguration(config);
    //dev->get



    dev->setColorFrameListener(listener);
    dev->setIrAndDepthFrameListener(listener);
    dev->start();


    // Color
    registration = new libfreenect2::Registration(dev->getIrCameraParams(), dev->getColorCameraParams());

    qDebug() << "IR cx: " << dev->getIrCameraParams().cx;
    qDebug() << "IR cy: " << dev->getIrCameraParams().cy;
    qDebug() << "IR fx: " << dev->getIrCameraParams().fx;
    qDebug() << "IR fy: " << dev->getIrCameraParams().fy;
    qDebug() << "IR k1: " << dev->getIrCameraParams().k1;
    qDebug() << "IR k2: " << dev->getIrCameraParams().k2;
    qDebug() << "IR k3: " << dev->getIrCameraParams().k3;


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


    libfreenect2::Frame *rgb = frames[libfreenect2::Frame::Color];
    libfreenect2::Frame *depth = frames[libfreenect2::Frame::Depth];

  /*  libfreenect2::Frame rgb(1980, 1024, 4);
    libfreenect2::Frame depth(512, 424, 4);
    rgb = *frames[libfreenect2::Frame::Color];
    depth = *frames[libfreenect2::Frame::Depth];*/



  //  timestamp = QDateTime::fromMSecsSinceEpoch(rgb->timestamp);//QDateTime::currentDateTime();
    timestamp = QDateTime::currentDateTime();
   /* qDebug() << "sequence: " << rgb->sequence;
    qDebug() << "timestamp: " << rgb->timestamp;
    qDebug() << "exposure: " << rgb->exposure;
    qDebug() << "gain: " << rgb->gain;
    qDebug() << "status: " << rgb->status;*/

    // Undistort and register frames
    libfreenect2::Frame undistorted(512, 424, 4), registered(512, 424, 4);
   // undistorted = libfreenect2::Frame(512, 424, 4);
    registration->apply(rgb, depth, &undistorted, &registered);

   // qDebug() << "width" << undistorted.width;
  //  qDebug() << "height" << undistorted.height;
  //  qDebug() << "Grab" << undistorted.data;
    mat_rgb = cv::Mat((*rgb).height, (*rgb).width, CV_8UC4, (rgb)->data) ;
    mat_depth = cv::Mat((*depth).height, (*depth).width, CV_32FC1, (depth)->data) / 1000.0f;
    mat_registered = cv::Mat(registered.height, registered.width, CV_8UC4, registered.data) ;
    mat_undistorted = cv::Mat(undistorted.height, undistorted.width, CV_32FC1, undistorted.data) / 1000.0f;
   // mat_undistorted = cv::Mat(undistorted.height, undistorted.width, CV_16UC1, undistorted.data);


    listener->release(frames);

    /* std::string str = QString("/home/silvio/PointClouds/Kinect%1_%2.png").arg(serial.toStdString().c_str()).arg(timestamp.toString("yyyy_MM_dd_HH_mm_ss_zzz")).toStdString();
     cv::Mat rgbMatinv; //= cv::Mat(1080, 1920, CV_8UC4);
     cv::flip(rgbMat, rgbMatinv, 1);
     cv::imwrite(str,rgbMat);*/

    //PC

    emit PCGrabbedsignal(this->getPointCloud());
   /* emit RGBGrabbedsignal(this->getRGB());
    emit DepthGrabbedsignal(this->getDepth());*/


    if (save)
        savePC();
    return SUCCESS;
    /*   const float *undistorted_data = (float*)undistorted.data;
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

    return SUCCESS;*/
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
    pose = trans;
    //PC->sensor_orientation_ = trans.getQuaternion();
    //PC->sensor_origin_ = trans.getOrigin4();
    GrabKinect();
}



// get Set Parameters
void KinectWidget::setPipeline(QString string)      {ui->comboBox_pipeline->setCurrentIndex(ui->comboBox_pipeline->findText(string));}
void KinectWidget::setSerial(QString string)        {ui->comboBox_KinectSerials->setCurrentIndex(ui->comboBox_KinectSerials->findText(string));}
void KinectWidget::setTransform(Transform transf)   {ui->myTransformationWidget->setTransform(transf);  ui->myTransformationWidget->emitTransform();}


QString KinectWidget::getPipeline()     {return ui->comboBox_pipeline->currentText();}
QString KinectWidget::getSerial()       {return ui->comboBox_KinectSerials->currentText();}
Transform KinectWidget::getTransform()  {return ui->myTransformationWidget->getTransform();}

PointCloudT::Ptr KinectWidget::getPointCloud()
{
    PointCloudT::Ptr PC(new PointCloudT());
    //pcl::copyPointCloud(*PC, *_PC);
    PC = TransformPointCloud(mat_registered, mat_undistorted);
    return PC;
}
PointCloudNormalT::Ptr KinectWidget::getPointCloudNormal()
{
    PointCloudT::Ptr PC = getPointCloud();
    PointCloudNormalT::Ptr _PC(new PointCloudNormalT());
    // TODO : ADD NORMAL CALCULATION
    pcl::copyPointCloud(*PC, *_PC);
    return _PC;
}

PointCloudT::Ptr KinectWidget::TransformPointCloud(cv::Mat registered, cv::Mat undistorted)
{
   // qDebug() << "TransformPointCloud";
    // Undistort and register frames
    //    libfreenect2::Frame undistorted(512, 424, 4), registered(512, 424, 4);
    //  registration->apply(rgb,depth,&undistorted,&registered);

    const float *undistorted_data = (float*)undistorted.data;
    const unsigned int *registered_data = (unsigned int*)registered.data;

   // qDebug() << "TransformPointCloud2";
    //   listener->release(frames);



    // Initialize my Point Cloud

    // PointCloud

    PointCloudT::Ptr PC(new PointCloudT);
    //PC.reset(new PointCloudT);
    PC->resize(512*424); // set the memory size to allocate
    PC->height = 424;        // set the height
    PC->width = 512;          // set the width
    PC->is_dense = false;                   // Kinect V2 returns organized and not dense point clouds
    //  PC.reset(new PointCloudT());
    PC->header.stamp = timestamp.toMSecsSinceEpoch();                               // the stamp correspond to the acquisition time
    //  PC->header.frame_id = QString("%1/PointClouds/Kinect%2_%3.pcd").arg(QDir::homePath()).arg(_serial.c_str()).arg(timestamp.toString("yyyy-MM-dd-HH:mm:ss:zzz")).toStdString();
    PC->header.frame_id = serial.toStdString();

    PC->sensor_orientation_ = pose.getQuaternion();
    PC->sensor_origin_ = pose.getOrigin4();

 //   qDebug() << "PC done";


    libfreenect2::Freenect2Device::IrCameraParams IRparam = dev->getIrCameraParams();
//registration->getPointXYZRGB();

   // registration->get

  //  qDebug() << "IR Param";

    // Set data into my Point cloud
    for (unsigned int i = 0; i < undistorted.rows;i++)
    {
        for (unsigned int j = 0; j < undistorted.cols ;j++)
        {
            int index = i * undistorted.cols + j;

            float depth = undistorted_data[index] ;
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

  //  qDebug() << "Double Loop";

    //  emit PCGrabbedsignal(PC);

    return PC;
}


QString KinectWidget::savePC()
{
    QString DIR = QDir::homePath() + "/PointClouds/" + QDateTime::fromMSecsSinceEpoch(timestamp.toMSecsSinceEpoch()).toString(DATEFORMAT);
    QString NAME = QDateTime::fromMSecsSinceEpoch(timestamp.toMSecsSinceEpoch()).toString(TIMEFORMAT) + "_" + QString::fromStdString(serial.toStdString());



    QString path_PCD =  QString("%1/%2.pcd").arg(DIR).arg(NAME);
    QString path_RGB =  QString("%1/rgb/%2.png").arg(DIR).arg(NAME);
    QString path_DEPTH =  QString("%1/depth/%2.png").arg(DIR).arg(NAME);
    QString path_UNDISTORTED =  QString("%1/undistorted/%2.png").arg(DIR).arg(NAME);
    QString path_REGISTERED =  QString("%1/registered/%2.png").arg(DIR).arg(NAME);

    QDir().mkpath(QFileInfo(path_PCD).absolutePath());
    QDir().mkpath(QFileInfo(path_RGB).absolutePath());
    QDir().mkpath(QFileInfo(path_DEPTH).absolutePath());
    QDir().mkpath(QFileInfo(path_UNDISTORTED).absolutePath());
    QDir().mkpath(QFileInfo(path_REGISTERED).absolutePath());

    // SAVE PCD
    pcl::io::savePCDFileBinary(path_PCD.toStdString(), *getPointCloud());

    // SAve RGB
    cv::imwrite(path_RGB.toStdString(), mat_rgb);

    // Save DEPTH
    cv::imwrite(path_DEPTH.toStdString(), mat_depth);

    // Save UNDISTORTED
    cv::imwrite(path_UNDISTORTED.toStdString(), mat_undistorted);

    // Save REGISTERED
    cv::imwrite(path_REGISTERED.toStdString(), mat_registered);



 /*  std::vector<int> compression_params;
    compression_params.push_back(CV_IMWRITE_PNG_COMPRESSION);
    compression_params.push_back(0);

   cv::Mat m1(mat_undistorted.rows, mat_undistorted.cols, CV_8UC4, mat_undistorted.data);
    cv::imwrite(path_DEPTH.toStdString(), m1 ,compression_params);
*/
   // cv::imwrite(path_DEPTH.toStdString(), mat_undistorted);

//    qDebug() << "PC saved in: " << path_PCD ;
    return path_PCD;




    /* std::string str = QString("/home/silvio/PointClouds/Kinect%1_%2.png").arg(serial.toStdString().c_str()).arg(timestamp.toString("yyyy_MM_dd_HH_mm_ss_zzz")).toStdString();
     cv::Mat rgbMatinv; //= cv::Mat(1080, 1920, CV_8UC4);
     cv::flip(rgbMat, rgbMatinv, 1);
     cv::imwrite(str,rgbMat);*/
}

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
#ifdef LIBFREENECT2_WITH_CUDA_SUPPORT
    else if (arg1.compare("Cuda") == 0)     pipeline = new libfreenect2::CudaPacketPipeline();
#endif
    else
        pipeline = new libfreenect2::CpuPacketPipeline();
}
void KinectWidget::on_comboBox_KinectSerials_currentIndexChanged(const QString &arg1)
{
    serial = arg1;
}



