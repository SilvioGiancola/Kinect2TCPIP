#include "mainwindow.h"
#include "ui_mainwindow.h"


// Constructor
MainWindow::MainWindow(QWidget *parent) : QMainWindow(parent), ui(new Ui::MainWindow)
{
    // Set the user interface from Qt Designer
    ui->setupUi(this);

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
          //  usleep(1000*1000);
        }
    }
}

// Destructor
MainWindow::~MainWindow()
{
    delete ui;
}



void MainWindow::newTCPIPConnection()
{
    socket = server->nextPendingConnection();

    connect (socket, SIGNAL(readyRead()), this, SLOT(newMessageReceived()));
    connect (socket, SIGNAL(stateChanged(QAbstractSocket::SocketState)), this, SLOT(clientStateChanged(QAbstractSocket::SocketState)));
    clientStateChanged(socket->state());

    socket->write("Hello client! :)");
}

void MainWindow::clientStateChanged(QAbstractSocket::SocketState state)
{
    if (state == QAbstractSocket::UnconnectedState)
        ui->label_ClientState->setText(QString("Client : UnconnectedState"));
    if (state == QAbstractSocket::HostLookupState)
        ui->label_ClientState->setText(QString("Client : HostLookupState"));
    if (state == QAbstractSocket::ConnectingState)
        ui->label_ClientState->setText(QString("Client : ConnectingState"));
    if (state == QAbstractSocket::ConnectedState)
        ui->label_ClientState->setText(QString("Client : ConnectedState"));
    if (state == QAbstractSocket::BoundState)
        ui->label_ClientState->setText(QString("Client : BoundState"));
    if (state == QAbstractSocket::ListeningState)
        ui->label_ClientState->setText(QString("Client : ListeningState"));
    if (state == QAbstractSocket::ClosingState)
        ui->label_ClientState->setText(QString("Client : ClosingState"));
}

void MainWindow::newMessageReceived()
{
    QString message = QString(socket->readAll());
    socket->write("ok!");
    qDebug() << "Server received the following message : " << message;
    if (message == QString("Connect"))
    {
       /* if (ui->myIMU->openConnection() == SUCCESS)
            socket->write("IMU connected!");
        else
            socket->write("ERROR in IMU connection!");

        if (ui->myPololuController->openConnection() == SUCCESS)
            socket->write("Pololu connected!");
        else
            socket->write("ERROR in Pololu connection!");*/

    }
    else if(message == QString("Disconnect"))
    {
     /*   ui->myIMU->closeConnection();
        ui->myPololuController->closeConnection();
        socket->write("disconnected!");*/
    }
    else if(message == QString("Grab"))
    {
       /* ui->myPololuController->impulseChannel0();
        Eigen::Quaternionf quat = ui->myIMU->getQuaternion();
        QString message = QString("grabbed! quat are (w, x, y, z): %1, %2, %3, %4").arg(quat.w()).arg(quat.x()).arg(quat.y()).arg(quat.z());
        socket->write(message.toStdString().c_str());*/
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







