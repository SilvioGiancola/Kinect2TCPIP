#include "ServerWindow.h"
#include "ui_ServerWindow.h"


// Constructor
ServerWindow::ServerWindow(QWidget *parent) : QMainWindow(parent), ui(new Ui::ServerWindow)
{
    // Set the user interface from Qt Designer
    ui->setupUi(this);
    ui->centralWidget->setVisible(false);


    /*    // Init intern parameters
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
*/



    // Settings
    readSettings();


    /*

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
*/



    QObject::connect(ui->myKinectWidget1, SIGNAL(PCGrabbedsignal(PointCloudT::Ptr)), this, SLOT(showPC(PointCloudT::Ptr)));
    QObject::connect(ui->myKinectWidget2, SIGNAL(PCGrabbedsignal(PointCloudT::Ptr)), this, SLOT(showPC(PointCloudT::Ptr)));
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

    /* for (int i = 0; i <dev.size();i++)
        CloseKinect(i);

    dev.clear();
    listener.clear();
    registration.clear();
    pipeline.clear();
*/
    //  viewer.
    delete server;
    delete socket;
    delete ui;
}



void ServerWindow::writeSettings()
{
    QSettings settings("SilvioGiancola", "Kinect 2 TCPIP");

    settings.beginGroup("Kinect settings");

    settings.setValue("Serial Kinect 0", ui->myKinectWidget1->getSerial());
    settings.setValue("Serial Kinect 1", ui->myKinectWidget2->getSerial());


    if (!ui->myKinectWidget1->getPipeline().compare("") == 0)
        settings.setValue("Pipeline 0", ui->myKinectWidget1->getPipeline());
    if (!ui->myKinectWidget2->getPipeline().compare("") == 0)
        settings.setValue("Pipeline 1", ui->myKinectWidget2->getPipeline());

    settings.setValue("LogLevel", ui->comboBox_log->currentText());

    settings.endGroup();
}

void ServerWindow::readSettings()
{
    QSettings settings("SilvioGiancola", "Kinect 2 TCPIP");

    settings.beginGroup("Kinect settings");

    ui->myKinectWidget1->setSerial(settings.value(QString("Serial Kinect 0"),"").toString());
    ui->myKinectWidget2->setSerial(settings.value(QString("Serial Kinect 1"),"").toString());

    ui->myKinectWidget1->setPipeline(settings.value(QString("Pipeline 0"),"").toString());
    ui->myKinectWidget2->setPipeline(settings.value(QString("Pipeline 1"),"").toString());

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
    QString Answer = "->" + message;

    if (message == QString(PROTOCOL_OPEN))
    {
        Answer.append(": ");
        if (ui->myKinectWidget1->OpenKinect()== SUCCESS) Answer.append("OK");
        else Answer.append("ERR");
        Answer.append(" / ");
        if (ui->myKinectWidget2->OpenKinect()== SUCCESS) Answer.append("OK");
        else Answer.append("ERR");

    }
    else if(message.contains(QString(PROTOCOL_CLOSE)))
    {
        Answer.append(": ");
        if (ui->myKinectWidget1->CloseKinect()== SUCCESS) Answer.append("OK");
        else Answer.append("ERR");
        Answer.append(" / ");
        if (ui->myKinectWidget2->CloseKinect()== SUCCESS) Answer.append("OK");
        else Answer.append("ERR");

    }
    else if(message.contains(QString(PROTOCOL_GRAB)))
    {
        Answer.append(": ");
        if (ui->myKinectWidget1->GrabKinect()== SUCCESS) Answer.append("OK");
        else Answer.append("ERR");
        Answer.append(" / ");
        if (ui->myKinectWidget2->GrabKinect()== SUCCESS) Answer.append("OK");
        else Answer.append("ERR");
    }
    else if(message.contains(QString(PROTOCOL_PIPELINE)))
    {
        const QString submess = message.remove(0,QString(PROTOCOL_PIPELINE).length());
        Answer.append(": " + submess);
        ui->myKinectWidget1->setPipeline(submess);
        ui->myKinectWidget2->setPipeline(submess);
        /*  int index = ui->comboBox_pipeline_kin1->findText(submess);
        ui->comboBox_pipeline_kin1->setCurrentIndex(index);
        ui->comboBox_pipeline_kin2->setCurrentIndex(index);*/
    }
    else if(message.contains(QString(PROTOCOL_SAVE)))
    {
        const QString submess = message.remove(0,QString(PROTOCOL_SAVE).length());
        Answer.append(": " + submess);
        int index = submess.toInt();
        ui->checkBox_save->setChecked((bool)index);
    }


    socket->write(Answer.toLocal8Bit());

}



void ServerWindow::showPC(PointCloudT::Ptr PC)
{

    pcl::visualization::PointCloudColorHandlerRGBField<PointT> single_color(PC);
    viewer->removePointCloud(PC->header.frame_id);
    viewer->addPointCloud<PointT>(PC, single_color, PC->header.frame_id);

    Eigen::Matrix4f trans = Eigen::Matrix4f::Identity();
    trans.block(0,0,3,3) = PC->sensor_orientation_.matrix();
    trans.block(0,3,4,1) = PC->sensor_origin_;
    viewer->removeCoordinateSystem("kin1_refsyst");
    viewer->addCoordinateSystem(0.2,Eigen::Affine3f(trans),"kin1_refsyst");

    ui->qvtkwidget->update ();


}

void ServerWindow::savePC(PointCloudT::Ptr PC)
{
    QString DIR = QDir::homePath() + "/PointClouds/" + QString::fromStdString(PC->header.frame_id) + "/";
    QString NAME = QDateTime::fromMSecsSinceEpoch(PC->header.stamp).toString(DATEFORMAT);

    QDir dir;
    if (!QDir(DIR).exists())
        dir.mkpath(DIR);

    QString path = QString("%1/%2.pcd").arg(DIR).arg(NAME);
    pcl::io::savePCDFileBinary(path.toStdString(), *PC);

    qDebug() << "saved in: " << path ;
}




void ServerWindow::on_comboBox_log_currentIndexChanged(const QString &arg1)
{
    if (arg1.compare("None") == 0)          libfreenect2::setGlobalLogger(libfreenect2::createConsoleLogger(libfreenect2::Logger::None));
    else if (arg1.compare("Debug") == 0)    libfreenect2::setGlobalLogger(libfreenect2::createConsoleLogger(libfreenect2::Logger::Debug));
    else if (arg1.compare("Info") == 0)     libfreenect2::setGlobalLogger(libfreenect2::createConsoleLogger(libfreenect2::Logger::Info));
    else if (arg1.compare("Warning") == 0)  libfreenect2::setGlobalLogger(libfreenect2::createConsoleLogger(libfreenect2::Logger::Warning));
    else if (arg1.compare("Error") == 0)    libfreenect2::setGlobalLogger(libfreenect2::createConsoleLogger(libfreenect2::Logger::Error));
}

void ServerWindow::on_checkBox_save_clicked(bool checked)
{
    if (checked)
    {
        QObject::connect(ui->myKinectWidget1, SIGNAL(PCGrabbedsignal(PointCloudT::Ptr)), this, SLOT(savePC(PointCloudT::Ptr)));
        QObject::connect(ui->myKinectWidget2, SIGNAL(PCGrabbedsignal(PointCloudT::Ptr)), this, SLOT(savePC(PointCloudT::Ptr)));
    }
    else
    {
        QObject::disconnect(ui->myKinectWidget1, SIGNAL(PCGrabbedsignal(PointCloudT::Ptr)), this, SLOT(savePC(PointCloudT::Ptr)));
        QObject::disconnect(ui->myKinectWidget2, SIGNAL(PCGrabbedsignal(PointCloudT::Ptr)), this, SLOT(savePC(PointCloudT::Ptr)));
    }
}
