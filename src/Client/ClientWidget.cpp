#include "ClientWidget.h"
#include "ui_ClientWidget.h"

ClientWidget::ClientWidget(QWidget *parent) :
    QWidget(parent),
    ui(new Ui::ClientWidget)
{
    ui->setupUi(this);

    ui->groupBox_ExpertMode->setChecked(false);

    mySocket = new QTcpSocket();
    mySocket->setReadBufferSize(0);
    connect(mySocket, SIGNAL(stateChanged(QAbstractSocket::SocketState)), this, SLOT(plotState(QAbstractSocket::SocketState)));
    connect(mySocket, SIGNAL(readyRead()), this, SLOT (newMessageReceived()));

    //SSH stuff
    connect(&proc, SIGNAL(readyReadStandardOutput()), this, SLOT(SSHlog()));
    connect(&proc, SIGNAL(stateChanged(QProcess::ProcessState)), this, SLOT(showProcState(QProcess::ProcessState)));



    PointCloudDecoder = new pcl::io::OctreePointCloudCompression<PointT> ();

}



ClientWidget::~ClientWidget()
{
    delete (PointCloudDecoder);
    mySocket->disconnectFromHost();
    delete ui;
}




// GET/SET
void ClientWidget::setIPCompletion(QStringList *strList)
{
    IPhistory = strList;
    QCompleter *comp = new QCompleter(*IPhistory, this);
    comp->setCaseSensitivity(Qt::CaseInsensitive);
    ui->lineEdit_IP->setCompleter(comp);
}


void ClientWidget::setIP(QString str){ui->lineEdit_IP->setText(str);}
void ClientWidget::setPort(QString str){ui->lineEdit_Port->setText(str);}
void ClientWidget::setMessage(QString str) {ui->lineEdit_Message->setText(str);}
QString ClientWidget::getIP(){return ui->lineEdit_IP->text();}
QString ClientWidget::getPort(){return ui->lineEdit_Port->text();}
QString ClientWidget::getLastMessage(){return ui->lineEdit_Message->text();}





// CLICK
// Socket Connection
void ClientWidget::on_pushButton_Connect_clicked()
{
    mySocket->connectToHost(ui->lineEdit_IP->text(), ui->lineEdit_Port->text().toInt());

    // Auto Completion
    IPhistory->append(ui->lineEdit_IP->text());
    IPhistory->removeDuplicates();
    IPhistory->sort();
    QCompleter *comp = new QCompleter(*IPhistory, this);
    comp->setCaseSensitivity(Qt::CaseInsensitive);
    ui->lineEdit_IP->setCompleter(comp);
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
void ClientWidget::on_pushButton_Register_clicked()             {   WriteMessage(QString(PROTOCOL_REGISTER));}
void ClientWidget::on_pushButton_Save_Settings_clicked()        {   WriteMessage(QString(PROTOCOL_SAVE_SETTINGS));}
void ClientWidget::on_comboBox_activated(const QString &arg1)   {   WriteMessage(QString(PROTOCOL_PIPELINE+arg1));}
void ClientWidget::on_checkBox_savePC_clicked(bool checked)     {   WriteMessage(QString("%1%2").arg(PROTOCOL_SAVE).arg((int)checked));}
void ClientWidget::on_pushButton_GrabAndTransmit_clicked()      {   WriteMessage(QString(PROTOCOL_GRAB_TRANSMIT));}




// SSH communication




void ClientWidget::on_pushButton_SSHReboot_clicked()
{
    if (proc.state() != QProcess::NotRunning)
    {
        qDebug() << "Proc already opened";
        return;
    }

    proc.start(QString("ssh sineco@%1 sudo reboot").arg(ui->lineEdit_IP->text()));
    if (proc.waitForFinished() == false)
        ui->plainTextEdit_SSHLog->appendPlainText("Reboot Timeout reached");
    else   ui->plainTextEdit_SSHLog->appendPlainText("Reboot Sent");
}

void ClientWidget::on_pushButton_SSHUpdate_clicked()
{
    // check if running
    if (proc.state() != QProcess::NotRunning)
    {
        qDebug() << "Proc already opened";
        return;
    }


    proc.start(QString("ssh sineco@%1 rm -r /home/sineco/Kinect2TCPIP").arg(ui->lineEdit_IP->text()));
    if (proc.waitForFinished() == false)
        ui->plainTextEdit_SSHLog->appendPlainText("Update Timeout reached");
    else   ui->plainTextEdit_SSHLog->appendPlainText("Update Done");


    proc.start(QString("ssh sineco@%1 mkdir -p /home/sineco/Kinect2TCPIP").arg(ui->lineEdit_IP->text()));
    if (proc.waitForFinished() == false)
        ui->plainTextEdit_SSHLog->appendPlainText("Update Timeout reached");
    else   ui->plainTextEdit_SSHLog->appendPlainText("Update Done");

    proc.start(QString("scp /home/silvio/git/Kinect2TCPIP/CompileScript.sh sineco@%1:/home/sineco/Kinect2TCPIP").arg(ui->lineEdit_IP->text()));
    if (proc.waitForFinished() == false)
        ui->plainTextEdit_SSHLog->appendPlainText("Update Timeout reached");
    else   ui->plainTextEdit_SSHLog->appendPlainText("Update Done");


    proc.start(QString("scp -r /home/silvio/git/Kinect2TCPIP/src sineco@%1:/home/sineco/Kinect2TCPIP").arg(ui->lineEdit_IP->text()));
    if (proc.waitForFinished() == false)
        ui->plainTextEdit_SSHLog->appendPlainText("Update Timeout reached");
    else   ui->plainTextEdit_SSHLog->appendPlainText("Update Done");

}

void ClientWidget::on_pushButton_SSHClientCompile_clicked()
{
    if (proc.state() != QProcess::NotRunning)
    {
        qDebug() << "Proc already opened";
        return;
    }

    proc.start(QString("ssh sineco@%1 sh /home/sineco/Kinect2TCPIP/CompileScript.sh").arg(ui->lineEdit_IP->text()));
}

void ClientWidget::showProcState(QProcess::ProcessState newState)
{
    if (newState == QProcess::NotRunning)
    {
        ui->Layout_ExpertMode->setEnabled(true);
        ui->label_State->setText("State: NotRunning");
    }
    else if (newState == QProcess::Starting)
    {
        ui->Layout_ExpertMode->setEnabled(false);
        ui->label_State->setText("State: Starting");
    }
    else if (newState == QProcess::Running)
    {
        ui->label_State->setText("State: Running");
    }
}

void ClientWidget::SSHlog()
{
    QString output = proc.readAllStandardOutput().replace("\n","");
    ui->plainTextEdit_SSHLog->appendPlainText(output);
}


// TCPIP handling

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
    QString message = QString::fromLocal8Bit(mySocket->readAll());
    ui->plainTextEdit_received->appendPlainText(QString("[%1]: %2").arg(QDateTime::currentDateTime().toString(TIMEFORMAT)).arg(message));

    if (message.contains("MY_"))
    {
        PCmode = false;
    }

    if (message.contains("<PCL-OCT-COMPRESSED>"))
    {
        PCmode = true;
        compressedDataPart = QString();
    }

    if (PCmode == true)
    {
        compressedDataPart = compressedDataPart.append(message);
    }

}


void ClientWidget::on_pushButton_ShowArrived_clicked()
{
    qDebug() << "PC detected";
    qDebug() << compressedDataPart;
    // decompress point cloud
    std::stringstream compressedData;
    compressedData.str(compressedDataPart.toStdString());
    PointCloudT::Ptr cloudOut (new PointCloudT ());
    qDebug() << "before decoding";
    PointCloudDecoder->decodePointCloud (compressedData, cloudOut);
    qDebug() << "decoded";


  //  emit PCtransmitted(cloudOut);
}


void ClientWidget::WriteMessage(QString message)
{
    mySocket->write(message.toStdString().c_str());
    ui->plainTextEdit_sent->appendPlainText(QString("[%1]: %2").arg(QDateTime::currentDateTime().toString()).arg(message));
    mySocket->waitForBytesWritten(1000);
    return;
}


