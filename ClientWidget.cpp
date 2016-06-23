#include "ClientWidget.h"
#include "ui_ClientWidget.h"

ClientWidget::ClientWidget(QWidget *parent) :
    QWidget(parent),
    ui(new Ui::ClientWidget)
{
    ui->setupUi(this);

    mySocket = new QTcpSocket();
    mySocket->setReadBufferSize(0);
    connect(mySocket, SIGNAL(stateChanged(QAbstractSocket::SocketState)), this, SLOT(plotState(QAbstractSocket::SocketState)));
    connect(mySocket, SIGNAL(readyRead()), this, SLOT (newMessageReceived()));
}



ClientWidget::~ClientWidget()
{
    mySocket->disconnectFromHost();
    delete ui;
}




// GET/SET
void ClientWidget::setIP(QString str){ui->lineEdit_IP->setText(str);}
void ClientWidget::setPort(QString str){ui->lineEdit_Port->setText(str);}
QString ClientWidget::getIP(){return ui->lineEdit_IP->text();}
QString ClientWidget::getPort(){return ui->lineEdit_Port->text();}




// CLICK
// Socket Connection
void ClientWidget::on_pushButton_Connect_clicked()
{
    mySocket->connectToHost(ui->lineEdit_IP->text(), ui->lineEdit_Port->text().toInt());
}

void ClientWidget::on_pushButton_Disconnect_clicked()
{
    mySocket->disconnectFromHost();
}

// Message Writing
void ClientWidget::on_pushButton_Send_clicked()                 {   WriteMessage(ui->lineEdit_Message->text());}
void ClientWidget::on_pushButton_Connect_Devices_clicked()      {   WriteMessage(QString(PROTOCOL_OPEN));}
void ClientWidget::on_pushButton_Disconnect_Devices_clicked()   {   WriteMessage(QString(PROTOCOL_CLOSE));}
void ClientWidget::on_pushButton_Grab_Devices_clicked()         {   WriteMessage(QString(PROTOCOL_GRAB));}
void ClientWidget::on_pushButton_Reboot_clicked()               {   WriteMessage(QString(PROTOCOL_REBOOT));}



/// TO DO:
/// ADD REBOOT server
/// ADD Setup OpenCL/OpenGL/CPU control (pipeline)
/// ADD verification number of Kinect
/// ADD answer on server if correctly grabbed / open / closed
/// ADD multiple computer handler
/// ADD position matrix handle
/// CONNECT 2 kinect on 2nd PC

// SIGNAL

void ClientWidget::plotState(QAbstractSocket::SocketState state)
{
    if (state == QAbstractSocket::UnconnectedState)     ui->label_ConnectionState->setText(QString("UnconnectedState"));
    else if (state == QAbstractSocket::HostLookupState) ui->label_ConnectionState->setText(QString("HostLookupState"));
    else if (state == QAbstractSocket::ConnectingState) ui->label_ConnectionState->setText(QString("ConnectingState"));
    else if (state == QAbstractSocket::ConnectedState)  ui->label_ConnectionState->setText(QString("ConnectedState"));
    else if (state == QAbstractSocket::BoundState)      ui->label_ConnectionState->setText(QString("BoundState"));
    else if (state == QAbstractSocket::ListeningState)  ui->label_ConnectionState->setText(QString("ListeningState"));
    else if (state == QAbstractSocket::ClosingState)    ui->label_ConnectionState->setText(QString("ClosingState"));

    qDebug() << state;
}

void ClientWidget::newMessageReceived()
{
    QString message = QString(mySocket->readAll());
     ui->plainTextEdit_received->appendPlainText(QString("[%1]: %2").arg(QDateTime::currentDateTime().toString()).arg(message));
}


void ClientWidget::WriteMessage(QString message)
{
    mySocket->write(message.toStdString().c_str());
    ui->plainTextEdit_sent->appendPlainText(QString("[%1]: %2").arg(QDateTime::currentDateTime().toString()).arg(message));
    return;
}

