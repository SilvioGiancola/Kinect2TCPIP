#include "ServerWindow.h"
#include "ui_ServerWindow.h"


// Constructor
ServerWindow::ServerWindow(QWidget *parent) : QMainWindow(parent), ui(new Ui::ServerWindow)
{
    // Set the user interface from Qt Designer
    ui->setupUi(this);
    ui->centralWidget->setVisible(false);


    // Init intern parameters
    for (int i = 0; i < 2 ; i++)
    {
        serials.append(QString(""));
        dev.append(0);
        listener.append(new libfreenect2::SyncMultiFrameListener(libfreenect2::Frame::Color | libfreenect2::Frame::Ir | libfreenect2::Frame::Depth));
        registration.append(new libfreenect2::Registration(libfreenect2::Freenect2Device::IrCameraParams(), libfreenect2::Freenect2Device::ColorCameraParams()));
        pipeline.append(new libfreenect2::CpuPacketPipeline());
    }



    // define serial numbers
    int n = freenect2.enumerateDevices();
    for (int i = 0; i < n; i++)
    {
        ui->comboBox_KinectSerials_kin1->addItem(QString::fromStdString(freenect2.getDeviceSerialNumber(i)));
        ui->comboBox_KinectSerials_kin2->addItem(QString::fromStdString(freenect2.getDeviceSerialNumber(i)));
    }




    // Settings
    readSettings();




#ifndef LIBFREENECT2_WITH_OPENGL_SUPPORT
    ui->comboBox_pipeline_kin1->model()->setData(ui->comboBox_pipeline_kin1->model()->index(1,0), 0, Qt::UserRole - 1);
    ui->comboBox_pipeline_kin2->model()->setData(ui->comboBox_pipeline_kin2->model()->index(1,0), 0, Qt::UserRole - 1);
#endif
#ifndef LIBFREENECT2_WITH_OPENCL_SUPPORT
    ui->comboBox_pipeline_kin1->model()->setData(ui->comboBox_pipeline_kin1->model()->index(2,0), 0, Qt::UserRole - 1);
    ui->comboBox_pipeline_kin2->model()->setData(ui->comboBox_pipeline_kin2->model()->index(2,0), 0, Qt::UserRole - 1);
#endif
#ifndef LIBFREENECT2_WITH_CUDA_SUPPORT
    ui->comboBox_pipeline_kin1->model()->setData(ui->comboBox_pipeline_kin1->model()->index(3,0), 0, Qt::UserRole - 1);
    ui->comboBox_pipeline_kin2->model()->setData(ui->comboBox_pipeline_kin2->model()->index(3,0), 0, Qt::UserRole - 1);
#endif




    QObject::connect(this, SIGNAL(PCGrabbedsignal(PointCloudT::Ptr)), this, SLOT(showPC(PointCloudT::Ptr)));
    // QObject::connect(Kinect2, SIGNAL(PCGrabbedsignal(PointCloudT::Ptr)), this, SLOT(showPC(PointCloudT::Ptr)));











    // initializza il Server TCPIP
    _port = 1234;

    server = new QTcpServer(this);
    socket = new QTcpSocket(this);

    connect (server, SIGNAL(newConnection()), this, SLOT(newTCPIPConnection()));

    if (server->listen(QHostAddress::Any, _port))
        qDebug() << "Server started";
    else
        qDebug() << "Server could not start";



    bool connectionFind = false;
    while (!connectionFind)
    {
        foreach (const QHostAddress &address, QNetworkInterface::allAddresses())
            if(address.protocol() == QAbstractSocket::IPv4Protocol && address != QHostAddress(QHostAddress::LocalHost))
            {
                ui->label_IP->setText(QString("My IP address is : %1 (TCPIP server on port %2)").arg(address.toString()).arg(_port));
                connectionFind = true;
            }

        if (!connectionFind)
        {
            qDebug () << "waiting for a network connection";
            qApp->processEvents();
            usleep(1000*1000);
        }
    }









    // set Viewer
    viewer.reset(new pcl::visualization::PCLVisualizer("Viewer",false));
    ui->qvtkwidget->SetRenderWindow(viewer->getRenderWindow());

    viewer->addCoordinateSystem(1.0);
    // on_pushButton_CleanViewer_clicked();
    viewer->setCameraPosition(-3.5,1,1, // mi posiziono dietro ad un Kinect
                              0.5,0.5,0.5, // guardo un punto centrale
                              0,0,1);   // orientato con la z verso l'alto
    viewer->setCameraClipDistances(-10,10);
    viewer->setBackgroundColor (0.5, 0.5, 0.5);
    ui->qvtkwidget->update ();

    viewer->setupInteractor(ui->qvtkwidget->GetInteractor(),ui->qvtkwidget->GetRenderWindow());
    viewer->getInteractorStyle()->setKeyboardModifier(pcl::visualization::INTERACTOR_KB_MOD_SHIFT);




}


// Destructor
ServerWindow::~ServerWindow()
{
    writeSettings();
    for (int i = 0; i <dev.size();i++)
    {
        dev.at(i)->stop();
        dev.at(i)->close();
    }
    dev.clear();
    listener.clear();
    registration.clear();
    pipeline.clear();

    //  viewer.
    delete server;
    delete socket;
    delete ui;
}



void ServerWindow::writeSettings()
{
    QSettings settings("SilvioGiancola", "Kinect 2 TCPIP");

    settings.beginGroup("Kinect settings");
    for (int i = 0; i < serials.length(); i++)
        settings.setValue(QString("Serial Kinect %1").arg(i), serials.at(i));

    settings.setValue(QString("Pipeline %1").arg(1), ui->comboBox_pipeline_kin1->currentText());
    settings.setValue(QString("Pipeline %1").arg(2), ui->comboBox_pipeline_kin2->currentText());

    settings.setValue("LogLevel", ui->comboBox_log->currentText());

    settings.endGroup();
}

void ServerWindow::readSettings()
{
    QSettings settings("SilvioGiancola", "Kinect 2 TCPIP");

    settings.beginGroup("Kinect settings");
    for (int i = 0; i < serials.length(); i++)
        serials.replace(i, settings.value(QString("Serial Kinect %1").arg(i),"").toString());


    // define current Kinect
    ui->comboBox_KinectSerials_kin1->setCurrentIndex(ui->comboBox_KinectSerials_kin1->findText(serials.at(0)));
    ui->comboBox_KinectSerials_kin2->setCurrentIndex(ui->comboBox_KinectSerials_kin2->findText(serials.at(1)));

    ui->comboBox_pipeline_kin1->setCurrentIndex(ui->comboBox_pipeline_kin1->findText(settings.value(QString("Pipeline %1").arg(1),"").toString()));
    ui->comboBox_pipeline_kin2->setCurrentIndex(ui->comboBox_pipeline_kin2->findText(settings.value(QString("Pipeline %1").arg(1),"").toString()));

    ui->comboBox_log->setCurrentIndex(ui->comboBox_log->findText(settings.value("LogLevel","").toString()));

    settings.endGroup();
}




// TCPIP

void ServerWindow::newTCPIPConnection()
{
    socket = server->nextPendingConnection();

    connect (socket, SIGNAL(readyRead()), this, SLOT(newMessageReceived()));
    connect (socket, SIGNAL(stateChanged(QAbstractSocket::SocketState)), this, SLOT(clientStateChanged(QAbstractSocket::SocketState)));
    clientStateChanged(socket->state());

    socket->write("Hello client! :)");
}

void ServerWindow::clientStateChanged(QAbstractSocket::SocketState state)
{
    if (state == QAbstractSocket::UnconnectedState) ui->label_ClientState->setText(QString("Socket : UnconnectedState"));
    if (state == QAbstractSocket::HostLookupState)  ui->label_ClientState->setText(QString("Socket : HostLookupState"));
    if (state == QAbstractSocket::ConnectingState)  ui->label_ClientState->setText(QString("Socket : ConnectingState"));
    if (state == QAbstractSocket::ConnectedState)   ui->label_ClientState->setText(QString("Socket : ConnectedState"));
    if (state == QAbstractSocket::BoundState)       ui->label_ClientState->setText(QString("Socket : BoundState"));
    if (state == QAbstractSocket::ListeningState)   ui->label_ClientState->setText(QString("Socket : ListeningState"));
    if (state == QAbstractSocket::ClosingState)     ui->label_ClientState->setText(QString("Socket : ClosingState"));
}

void ServerWindow::newMessageReceived()
{
    QString message = QString(socket->readAll());
    socket->write("ok!");
    qDebug() << "Server received the following message : " << message;
    if (message == QString(PROTOCOL_OPEN))
    {
        on_pushButton_Open_kin1_clicked();
        on_pushButton_Open_kin2_clicked();
    }
    else if(message == QString(PROTOCOL_CLOSE))
    {
        on_pushButton_Close_kin1_clicked();
        on_pushButton_Close_kin2_clicked();
    }
    else if(message == QString(PROTOCOL_GRAB))
    {
        on_pushButton_Grab_kin1_clicked();
        on_pushButton_Grab_kin2_clicked();
    }
    else if(message == QString(PROTOCOL_REBOOT))
    {
        QProcess process;
        process.startDetached("sudo reboot");
    }
    else if(message == QString(PROTOCOL_GITUPDATE))
    {
        QProcess process;
        process.startDetached("cd /home/sineco/git/Kinect2TCPIP && git stash && git pull && cd build && cmake .. && make -j4");
    }
    else if(message.contains("GrabMult"))
    {
        /* message.remove(0,8);
        int loop = message.toInt();

        for (int i = 0; i<loop; i++)
        {
            ui->myPololuController->impulseChannel0();
            Eigen::Quaternionf quat = ui->myIMU->getQuaternion();
            QString message = QString("grabbed %1 ! quat are (w, x, y, z): %2, %3, %4, %5").arg(i).arg(quat.w()).arg(quat.x()).arg(quat.y()).arg(quat.z());
            socket->write(message.toStdString().c_str());
            socket->waitForBytesWritten(1000);
            QThread::sleep(1);
        }*/

    }
}



// KINECT

int ServerWindow::OpenKinect(int i)
{
    if (dev.at(i))
    {
        qWarning() << "Kinect is already open";
        return ERROR;
    }

   /* if(freenect2.enumerateDevices() == 0)
    {
        qWarning() << tr("Error in opening Kinect, no Kinect connected");
        return ERROR;
    }*/

    if (serials.at(i).isEmpty())
    {
        qWarning() << tr("Error in opening Kinect, please define a serial number");
        return ERROR;
    }


    dev.replace(i, freenect2.openDevice(serials.at(i).toStdString(), pipeline.at(i)));



    if(dev.at(i) == 0)
    {
        qWarning() << tr("Error in opening Kinect, failure opening the default one");
        return ERROR;
    }


    dev.at(i)->setColorFrameListener(listener.at(i));
    dev.at(i)->setIrAndDepthFrameListener(listener.at(i));
    dev.at(i)->start();


    // Color
    registration.replace(i, new libfreenect2::Registration(dev.at(i)->getIrCameraParams(), dev.at(i)->getColorCameraParams()));


    return SUCCESS;
}

int ServerWindow::GrabKinect(int i)
{
    if (dev.at(i) == 0)
    {
        qWarning() << "Kinect stream is not open";
        return ERROR;
    }


    QTime t;
    t.start();



    // Acquire Frames
    libfreenect2::FrameMap frames;
    listener.at(i)->waitForNewFrame(frames);

    QDateTime timestamp = QDateTime::currentDateTime();

    libfreenect2::Frame *rgb = frames[libfreenect2::Frame::Color];
    libfreenect2::Frame *depth = frames[libfreenect2::Frame::Depth];



    // Undistort and register frames
    libfreenect2::Frame undistorted(512, 424, 4), registered(512, 424, 4);
    registration.at(i)->apply(rgb,depth,&undistorted,&registered);

    const float *undistorted_data = (float*)undistorted.data;
    const unsigned int *registered_data = (unsigned int*)registered.data;

    listener.at(i)->release(frames);



    // Initialize my Point Cloud
    PointCloudT::Ptr PC(new PointCloudT);
    //  PC.reset(new PointCloudT());
    PC->resize(512*424); // set the memory size to allocate
    PC->height = 424;        // set the height
    PC->width = 512;          // set the width
    PC->is_dense = false;                   // Kinect V2 returns organized and not dense point clouds
    PC->header.stamp = timestamp.toMSecsSinceEpoch();                               // the stamp correspond to the acquisition time
    //  PC->header.frame_id = QString("%1/PointClouds/Kinect%2_%3.pcd").arg(QDir::homePath()).arg(_serial.c_str()).arg(timestamp.toString("yyyy-MM-dd-HH:mm:ss:zzz")).toStdString();
    PC->header.frame_id = serials.at(i).toStdString();

    libfreenect2::Freenect2Device::IrCameraParams IRparam = dev.at(i)->getIrCameraParams();

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

int ServerWindow::CloseKinect(int i)
{

    if (dev.at(i) == 0)
    {
        qWarning() << "Kinect already closed";
        return ERROR;
    }

    //  _open = false;
    // TODO: restarting ir stream doesn't work!
    // TODO: bad things will happen, if frame listeners are freed before dev->stop() :(
    dev.at(i)->stop();
    dev.at(i)->close();
    dev.replace(i, 0);



    return SUCCESS;
}



// UIv

void ServerWindow::on_pushButton_Open_kin1_clicked()
{
    OpenKinect(0);
}

void ServerWindow::on_pushButton_Close_kin1_clicked()
{
    CloseKinect(0);
}

void ServerWindow::on_pushButton_Grab_kin1_clicked()
{
    GrabKinect(0);
}

void ServerWindow::on_pushButton_Open_kin2_clicked()
{
    OpenKinect(1);
}

void ServerWindow::on_pushButton_Close_kin2_clicked()
{
    CloseKinect(1);
}

void ServerWindow::on_pushButton_Grab_kin2_clicked()
{
    GrabKinect(1);
}

void ServerWindow::on_comboBox_KinectSerials_kin1_currentIndexChanged(const QString &arg1)
{
    serials.replace(0, arg1);
}

void ServerWindow::on_comboBox_KinectSerials_kin2_currentIndexChanged(const QString &arg1)
{
    serials.replace(1, arg1);
}




void ServerWindow::showPC(PointCloudT::Ptr PC)
{
    pcl::visualization::PointCloudColorHandlerRGBField<PointT> single_color(PC);
    viewer->removePointCloud(PC->header.frame_id);
    viewer->addPointCloud<PointT>(PC, single_color, PC->header.frame_id);
    ui->qvtkwidget->update ();

    qDebug() << QString::fromStdString(PC->header.frame_id);
}



void ServerWindow::on_comboBox_pipeline_kin1_currentIndexChanged(const QString &arg1)
{
    if (arg1.compare("Cpu") == 0)           pipeline.replace(0, new libfreenect2::CpuPacketPipeline());
#ifdef LIBFREENECT2_WITH_OPENGL_SUPPORT
    else if (arg1.compare("OpenGL") == 0)   pipeline.replace(0, new libfreenect2::OpenGLPacketPipeline());
#endif
#ifdef LIBFREENECT2_WITH_OPENCL_SUPPORT
    else if (arg1.compare("OpenCL") == 0)   pipeline.replace(0, new libfreenect2::OpenCLPacketPipeline());
#endif
    if (arg1.compare("Cpu") == 0)           pipeline.replace(0, new libfreenect2::CpuPacketPipeline());
    else if (arg1.compare("OpenGL") == 0)   pipeline.replace(0, new libfreenect2::OpenGLPacketPipeline());
    else if (arg1.compare("OpenCL") == 0)   pipeline.replace(0, new libfreenect2::OpenCLPacketPipeline());
#ifdef LIBFREENECT2_WITH_CUDA_SUPPORT
    else if (arg1.compare("Cuda") == 0)     pipeline.replace(0, new libfreenect2::CudaPacketPipeline());
#endif
}

void ServerWindow::on_comboBox_pipeline_kin2_currentIndexChanged(const QString &arg1)
{
    if (arg1.compare("Cpu") == 0)           pipeline.replace(1, new libfreenect2::CpuPacketPipeline());
#ifdef LIBFREENECT2_WITH_OPENGL_SUPPORT
    else if (arg1.compare("OpenGL") == 0)   pipeline.replace(1, new libfreenect2::OpenGLPacketPipeline());
#endif
#ifdef LIBFREENECT2_WITH_OPENCL_SUPPORT
    else if (arg1.compare("OpenCL") == 0)   pipeline.replace(1, new libfreenect2::OpenCLPacketPipeline());
#endif
#ifdef LIBFREENECT2_WITH_CUDA_SUPPORT
    else if (arg1.compare("Cuda") == 0)     pipeline.replace(1, new libfreenect2::CudaPacketPipeline());
#endif
}

void ServerWindow::on_comboBox_log_currentIndexChanged(const QString &arg1)
{
    if (arg1.compare("None") == 0)          libfreenect2::setGlobalLogger(libfreenect2::createConsoleLogger(libfreenect2::Logger::None));
    else if (arg1.compare("Debug") == 0)    libfreenect2::setGlobalLogger(libfreenect2::createConsoleLogger(libfreenect2::Logger::Debug));
    else if (arg1.compare("Info") == 0)     libfreenect2::setGlobalLogger(libfreenect2::createConsoleLogger(libfreenect2::Logger::Info));
    else if (arg1.compare("Warning") == 0)  libfreenect2::setGlobalLogger(libfreenect2::createConsoleLogger(libfreenect2::Logger::Warning));
    else if (arg1.compare("Error") == 0)    libfreenect2::setGlobalLogger(libfreenect2::createConsoleLogger(libfreenect2::Logger::Error));
}
