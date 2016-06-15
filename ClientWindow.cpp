#include "ClientWindow.h"
#include "ui_ClientWindow.h"

ClientWindow::ClientWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::ClientWindow)
{
    ui->setupUi(this);

    readSettings();

    mySocket = new QTcpSocket();
    mySocket->setReadBufferSize(0);
    connect(mySocket, SIGNAL(stateChanged(QAbstractSocket::SocketState)), this, SLOT(plotState(QAbstractSocket::SocketState)));
    connect(mySocket, SIGNAL(readyRead()), this, SLOT (newMessageReceived()));
}



ClientWindow::~ClientWindow()
{
    mySocket->disconnectFromHost();
    writeSettings();
    delete ui;
}


void ClientWindow::writeSettings()
{
    QSettings settings("SilvioGiancola", "Kinect 2 TCPIP");

    settings.beginGroup("ClientWindow");
    settings.setValue("SINECO2 IP",ui->lineEdit_IP->text());
    settings.setValue("SINECO2 Port",ui->lineEdit_Port->text());
    settings.endGroup();
}

void ClientWindow::readSettings()
{
    QSettings settings("SilvioGiancola", "Kinect 2 TCPIP");

    settings.beginGroup("ClientWindow");
    ui->lineEdit_IP->setText(settings.value("SINECO2 IP","0.0.0.0").toString());
    ui->lineEdit_Port->setText(settings.value("SINECO2 Port","1234").toString());
    settings.endGroup();
}


void ClientWindow::on_pushButton_Connect_clicked()
{
    mySocket->connectToHost(ui->lineEdit_IP->text(), ui->lineEdit_Port->text().toInt());
}

void ClientWindow::on_pushButton_Disconnect_clicked()
{
    mySocket->disconnectFromHost();
}

void ClientWindow::plotState(QAbstractSocket::SocketState state)
{
    if (state == QAbstractSocket::UnconnectedState)
        ui->label_ConnectionState->setText(QString("UnconnectedState"));
    else if (state == QAbstractSocket::HostLookupState)
        ui->label_ConnectionState->setText(QString("HostLookupState"));
    else if (state == QAbstractSocket::ConnectingState)
        ui->label_ConnectionState->setText(QString("ConnectingState"));
    else if (state == QAbstractSocket::ConnectedState)
        ui->label_ConnectionState->setText(QString("ConnectedState"));
    else if (state == QAbstractSocket::BoundState)
        ui->label_ConnectionState->setText(QString("BoundState"));
    else if (state == QAbstractSocket::ListeningState)
        ui->label_ConnectionState->setText(QString("ListeningState"));
    else if (state == QAbstractSocket::ClosingState)
        ui->label_ConnectionState->setText(QString("ClosingState"));

    qDebug() << state;
}



void ClientWindow::on_pushButton_Send_clicked()
{
    WriteMessage(ui->lineEdit_Message->text());
}




void ClientWindow::on_pushButton_Connect_Devices_clicked()
{
    WriteMessage(QString("Connect"));
}

void ClientWindow::on_pushButton_Disconnect_Devices_clicked()
{
    WriteMessage(QString("Disconnect"));
}

void ClientWindow::on_pushButton_Grab_Devices_clicked()
{
    WriteMessage(QString("Grab"));
}


void ClientWindow::newMessageReceived()
{
    QString message = QString(mySocket->readAll());
    qDebug() << "Client Revceived the following message : " << message;
}


void ClientWindow::WriteMessage(QString message)
{
    mySocket->write(message.toStdString().c_str());
    return;
}

void ClientWindow::on_pushButton_Grab_Multiple_clicked()
{
    WriteMessage(QString("GrabMult%1").arg(ui->spinBox_nb_Grab->value()));
}
