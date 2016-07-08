#include "ServerWindow.h"
#include "ui_ServerWindow.h"


// Constructor
ServerWindow::ServerWindow(QWidget *parent) : QMainWindow(parent), ui(new Ui::ServerWindow)
{
    // Set the user interface from Qt Designer
    ui->setupUi(this);
    ui->centralWidget->setVisible(false);


    // Settings
    readSettings();


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

}

// Destructor
ServerWindow::~ServerWindow()
{
    writeSettings();

    delete server;
    delete socket;
    delete ui;
}


// QSettings
void ServerWindow::writeSettings()
{
    QSettings settings("SilvioGiancola", "Kinect 2 TCPIP");

    settings.beginGroup("Kinect settings");

    if (!ui->myKinectWidget1->getSerial().length() > 5) settings.setValue("Serial Kinect 0", ui->myKinectWidget1->getSerial());
    if (!ui->myKinectWidget1->getSerial().length() > 5) settings.setValue("Serial Kinect 1", ui->myKinectWidget2->getSerial());

    if (!ui->myKinectWidget1->getPipeline().compare("") == 0)   settings.setValue("Pipeline 0", ui->myKinectWidget1->getPipeline());
    if (!ui->myKinectWidget2->getPipeline().compare("") == 0)   settings.setValue("Pipeline 1", ui->myKinectWidget2->getPipeline());

    settings.setValue("Origin 0", ui->myKinectWidget1->getTransform().getOriginQVector3D());
    settings.setValue("Origin 1", ui->myKinectWidget2->getTransform().getOriginQVector3D());

    settings.setValue("Euler 0", ui->myKinectWidget1->getTransform().getEulerAnglesQVector3D());
    settings.setValue("Euler 1", ui->myKinectWidget2->getTransform().getEulerAnglesQVector3D());

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

    QVector3D Orig0= settings.value(QString("Origin 0"),"").value<QVector3D>();
    QVector3D Euler0= settings.value(QString("Euler 0"),"").value<QVector3D>();
    ui->myKinectWidget1->setTransform(Transform(Orig0, Euler0));


    QVector3D Orig1 = settings.value(QString("Origin 1"),"").value<QVector3D>();
    QVector3D Euler1 = settings.value(QString("Euler 1"),"").value<QVector3D>();
    ui->myKinectWidget2->setTransform(Transform(Orig1, Euler1));

    //  ui->myKinectWidget2->setTransform(settings.value(QString("Origin 1"),"").toString());

    ui->comboBox_log->setCurrentIndex(ui->comboBox_log->findText(settings.value("LogLevel","").toString()));

    settings.endGroup();
}


PointCloudT::Ptr ServerWindow::getPointCloud(int i)
{
    if (i == 0)         return ui->myKinectWidget1->getPointCloud();
    else if ( i == 1)   return ui->myKinectWidget2->getPointCloud();

    return PointCloudT::Ptr(new PointCloudT);
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
    QString stateName;
    if (state == QAbstractSocket::UnconnectedState) stateName = "Socket : UnconnectedState";
    if (state == QAbstractSocket::HostLookupState)  stateName = "Socket : HostLookupState";
    if (state == QAbstractSocket::ConnectingState)  stateName = "Socket : ConnectingState";
    if (state == QAbstractSocket::ConnectedState)   stateName = "Socket : ConnectedState";
    if (state == QAbstractSocket::BoundState)       stateName = "Socket : BoundState";
    if (state == QAbstractSocket::ListeningState)   stateName = "Socket : ListeningState";
    if (state == QAbstractSocket::ClosingState)     stateName = "Socket : ClosingState";

    ui->label_ClientState->setText(stateName);
    ui->logWidget_server->appendText("[StateChanged] " + stateName);
}



void ServerWindow::newMessageReceived()
{
    QString message = QString(socket->readAll());
    ui->logWidget_server->appendText("[TCP/IP] " + message);

    QString Answer = "->" + message;

    if (message == QString(PROTOCOL_OPEN))
    {
        Answer.append(": ");
        if (ui->myKinectWidget1->OpenKinect()== SUCCESS) Answer.append(ui->myKinectWidget1->getSerial());
        else Answer.append("ERR");
        Answer.append(" / ");
        if (ui->myKinectWidget2->OpenKinect()== SUCCESS) Answer.append(ui->myKinectWidget2->getSerial());
        else Answer.append("ERR");

    }
    else if(message == QString(PROTOCOL_CLOSE))
    {
        Answer.append(": ");
        if (ui->myKinectWidget1->CloseKinect()== SUCCESS) Answer.append("OK");
        else Answer.append("ERR");
        Answer.append(" / ");
        if (ui->myKinectWidget2->CloseKinect()== SUCCESS) Answer.append("OK");
        else Answer.append("ERR");

    }
    else if(message == QString(PROTOCOL_SAVE_SETTINGS))
    {
        writeSettings();
        Answer.append(": OK");
    }
    else if(message == QString(PROTOCOL_GRAB))
    {
        Answer.append(": ");
        if (ui->myKinectWidget1->GrabKinect()== SUCCESS) Answer.append("OK");
        else Answer.append("ERR");
        Answer.append(" / ");
        if (ui->myKinectWidget2->GrabKinect()== SUCCESS) Answer.append("OK");
        else Answer.append("ERR");
    }
    else if(message == QString(PROTOCOL_REGISTER))
    {
        on_pushButton_registrer_clicked();
        Answer.append(": OK");
    }
    else if(message == QString(PROTOCOL_TRANSMIT_POINTCLOUDS))
    {
        QString path0 = savePC(getPointCloud(0));
        QString path1 = savePC(getPointCloud(1));

        Answer = path0 + ":"+ path1;
    }
    else if(message == QString(PROTOCOL_TIME))
    {
        Answer.append(QDateTime::currentDateTime().toString(TIMEFORMAT));
    }

    // answers with return parameters
    else if(message.contains(QString(PROTOCOL_PIPELINE)))
    {
        const QString submess = message.remove(0,QString(PROTOCOL_PIPELINE).length());
        Answer.append(": " + submess);
        ui->myKinectWidget1->setPipeline(submess);
        ui->myKinectWidget2->setPipeline(submess);
    }
    else if(message.contains(QString(PROTOCOL_SAVE)))
    {
        const QString submess = message.remove(0,QString(PROTOCOL_SAVE).length());
        Answer.append(": " + submess);
        int index = submess.toInt();
        ui->checkBox_save->setChecked((bool)index);
    }
    else if(message.contains(QString(PROTOCOL_POSE)))
    {
        const QString submess = message.remove(0,QString(PROTOCOL_POSE).length());

        Transform trans;
        if (submess.section("_",0,0).toInt() == 0)
        {
            QString pose=submess.section("_",1,-1);
            trans.fromPrettyPrint(pose);

            ui->myKinectWidget1->setTransform(trans);
            Answer.append(": OK");
        }
        else if (submess.section("_",0,0).toInt() == 1)
        {
            QString pose = submess.section("_",1,-1);
            trans.fromPrettyPrint(pose);

            ui->myKinectWidget2->setTransform(trans);
            Answer.append("/ OK");
        }
        Answer.append(": " + submess);
    }


    socket->write(Answer.toLocal8Bit());

}


QString ServerWindow::savePC(PointCloudT::Ptr PC)
{
    QString DIR = QDir::homePath() + "/PointClouds/" + QDateTime::fromMSecsSinceEpoch(PC->header.stamp).toString(DATEFORMAT);
    QString NAME = QDateTime::fromMSecsSinceEpoch(PC->header.stamp).toString(TIMEFORMAT) + "_" + QString::fromStdString(PC->header.frame_id);
    QString path =  QString("%1/%2.pcd").arg(DIR).arg(NAME);

    QDir().mkpath(QFileInfo(path).absolutePath());

    pcl::io::savePCDFileBinary(path.toStdString(), *PC);

    qDebug() << "saved in: " << path ;
    return path;
}

/*

// TODO : Verify Registration

#include <pcl/features/integral_image_normal.h>
#include <pcl/filters/filter.h>
#include <pcl/registration/icp.h> //RegistrationICP
#include <pcl/registration/correspondence_estimation_organized_projection.h>
#include <pcl/registration/correspondence_estimation_normal_shooting.h>
#include <pcl/registration/correspondence_rejection_one_to_one.h>
#include <pcl/registration/correspondence_rejection_median_distance.h>
#include <pcl/registration/transformation_estimation_point_to_plane.h>

*/

#include <registration.h>

void ServerWindow::on_pushButton_registrer_clicked()
{
    try
    {
        ui->myKinectWidget1->GrabKinect();
        ui->myKinectWidget2->GrabKinect();
        PointCloudT::Ptr PC_target = ui->myKinectWidget1->getPointCloud();
        PointCloudT::Ptr PC_input = ui->myKinectWidget2->getPointCloud();


        Transform T =  utils::getTransformation(PC_target, PC_input);


        PointCloudT::Ptr PCnew = ui->myKinectWidget2->getPointCloud();
        Transform currentPose = Transform(PCnew->sensor_origin_, PCnew->sensor_orientation_);


        T = T.postmultiplyby(currentPose);
        T.print();

        ui->myKinectWidget2->setTransform(T);


        writeSettings();

        ui->myKinectWidget1->GrabKinect();
        ui->myKinectWidget2->GrabKinect();
    }
    catch (std::exception& ex)
    {
        qDebug() << ex.what();
    }





}



void ServerWindow::on_comboBox_log_currentIndexChanged(const QString &arg1)
{
    if (arg1.compare("None") == 0)          libfreenect2::setGlobalLogger(libfreenect2::createConsoleLogger(libfreenect2::Logger::None));
    else if (arg1.compare("Debug") == 0)    libfreenect2::setGlobalLogger(libfreenect2::createConsoleLogger(libfreenect2::Logger::Debug));
    else if (arg1.compare("Info") == 0)     libfreenect2::setGlobalLogger(libfreenect2::createConsoleLogger(libfreenect2::Logger::Info));
    else if (arg1.compare("Warning") == 0)  libfreenect2::setGlobalLogger(libfreenect2::createConsoleLogger(libfreenect2::Logger::Warning));
    else if (arg1.compare("Error") == 0)    libfreenect2::setGlobalLogger(libfreenect2::createConsoleLogger(libfreenect2::Logger::Error));
}

void ServerWindow::on_checkBox_save_toggled(bool checked)
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

