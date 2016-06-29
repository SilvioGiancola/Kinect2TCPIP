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

    settings.setValue("Serial Kinect 0", ui->myKinectWidget1->getSerial());
    settings.setValue("Serial Kinect 1", ui->myKinectWidget2->getSerial());


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
    else if(message.contains(QString(PROTOCOL_SAVE_SETTINGS)))
    {
        writeSettings();
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
    else if(message.contains(QString(PROTOCOL_REGISTER)))
    {
        on_pushButton_registrer_clicked();
        Answer.append(": OK");
    }
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

    socket->write(Answer.toLocal8Bit());

}



// Application
void ServerWindow::savePC(PointCloudT::Ptr PC)
{
    QString DIR = QDir::homePath() + "/PointClouds/" + QDateTime::fromMSecsSinceEpoch(PC->header.stamp).toString(DATEFORMAT) + "/";
    QString NAME = QDateTime::fromMSecsSinceEpoch(PC->header.stamp).toString(TIMEFORMAT) + "_" + QString::fromStdString(PC->header.frame_id);

    QDir dir;
    if (!QDir(DIR).exists())
        dir.mkpath(DIR);

    QString path = QString("%1/%2.pcd").arg(DIR).arg(NAME);
    pcl::io::savePCDFileBinary(path.toStdString(), *PC);

    qDebug() << "saved in: " << path ;
}



// TODO : Verify Registration

#include <pcl/features/integral_image_normal.h>
#include <pcl/filters/filter.h>
#include <pcl/registration/icp.h> //RegistrationICP
#include <pcl/registration/correspondence_estimation_organized_projection.h>
#include <pcl/registration/correspondence_estimation_normal_shooting.h>
#include <pcl/registration/correspondence_rejection_one_to_one.h>
#include <pcl/registration/correspondence_rejection_median_distance.h>
#include <pcl/registration/transformation_estimation_point_to_plane.h>



void ServerWindow::on_pushButton_registrer_clicked()
{
    try
    {
        ui->myKinectWidget1->GrabKinect();
        ui->myKinectWidget2->GrabKinect();
        PointCloudT::Ptr PC1 = ui->myKinectWidget1->getPointCloud();
        PointCloudT::Ptr PC2 = ui->myKinectWidget2->getPointCloud();


        // NORMAL
        qDebug() << "NORMAL: " << PC1->size();
        pcl::IntegralImageNormalEstimation<PointT, PointT> ne;
        ne.setNormalEstimationMethod (ne.AVERAGE_3D_GRADIENT);
        ne.setMaxDepthChangeFactor(0.02);
        ne.setNormalSmoothingSize(10.0);

        ne.setInputCloud(PC1);
        ne.compute(*PC1);
        ne.setInputCloud(PC2);
        ne.compute(*PC2);


        // TRANSFORM
        qDebug() << "TRANSFORM: " << PC1->size();
        pcl::transformPointCloud(*PC1, *PC1, PC1->sensor_origin_.head(3), PC1->sensor_orientation_);
        pcl::transformPointCloud(*PC2, *PC2, PC2->sensor_origin_.head(3), PC2->sensor_orientation_);


        //REMOVE NAN
        qDebug() << "REMOVE NAN: " << PC1->size();
        std::vector<int> ind;
        pcl::removeNaNFromPointCloud(*PC1, *PC1, ind);
        pcl::removeNaNNormalsFromPointCloud(*PC1, *PC1, ind);
        pcl::removeNaNFromPointCloud(*PC2, *PC2, ind);
        pcl::removeNaNNormalsFromPointCloud(*PC2, *PC2, ind);




        // Creo il mio elemento ICP
        qDebug() << "ICP: " << PC1->size();
        pcl::IterativeClosestPoint<PointT, PointT> icp;




        // Corrispondence Estimation
        // Scelgo il mia stima dei accopiamenti
        qDebug() << "Corrispondence Estimation";
        pcl::registration::CorrespondenceEstimationBase<PointT, PointT>::Ptr cens;
        cens.reset(new pcl::registration::CorrespondenceEstimationNormalShooting<PointT, PointT,PointT>);

        cens->setInputTarget (PC1);
        cens->setInputSource (PC2);

        icp.setCorrespondenceEstimation (cens);




        // Corrispondence Rejection
        qDebug() << "Corrispondence Rejection";



        pcl::registration::CorrespondenceRejectorOneToOne::Ptr cor_rej_o2o (new pcl::registration::CorrespondenceRejectorOneToOne);
        icp.addCorrespondenceRejector (cor_rej_o2o);

        /*
        pcl::registration::CorrespondenceRejectorMedianDistance::Ptr cor_rej_med (new pcl::registration::CorrespondenceRejectorMedianDistance);
        cor_rej_med->setInputTarget<PointT> (PC1);
        cor_rej_med->setInputSource<PointT> (PC2);
        icp.addCorrespondenceRejector (cor_rej_med);*/








        // Transformation Estimation
        // Scelgo un metodo per risolvere il problema
        qDebug() << "Transformation Estimation";
        pcl::registration::TransformationEstimation<PointT, PointT>::Ptr te;
        te.reset(new pcl::registration::TransformationEstimationPointToPlane<PointT, PointT>);

        icp.setTransformationEstimation (te);





        icp.setInputSource(PC2);
        icp.setInputTarget(PC1);


        // Modalità di fine ICP
        //icp.setEuclideanFitnessEpsilon(10E-9);
        //icp.setTransformationEpsilon(10E-9);
        icp.setMaximumIterations(1);


        qDebug() << "Align";
        PointCloudT::Ptr Final(new PointCloudT);
        icp.align(*Final);
        qDebug() << "has converged:" << icp.hasConverged() << " score: " << icp.getFitnessScore();


        Eigen::Matrix4f ICPtransformation = icp.getFinalTransformation();
        std::cout << ICPtransformation  << std::endl;


        Transform T;
        PointCloudT::Ptr PCnew = ui->myKinectWidget2->getPointCloud();
        T.setOrigin4(ICPtransformation.block<4,1>(0,3) + PCnew->sensor_origin_);
        T.setQuaternion(Eigen::Quaternionf(ICPtransformation.block<3,3>(0,0)) * PCnew->sensor_orientation_);

        T.print();

        ui->myKinectWidget2->setTransform(T);
        //  result->sensor_origin_ = ICPtransformation.block<4,1>(0,3) + PC->sensor_origin_;
        //  result->sensor_orientation_ = Eigen::Quaternionf(ICPtransformation.block<3,3>(0,0)) * PC->sensor_orientation_;

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
