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



    cloud0.reset(new PointCloudT);
    cloud1.reset(new PointCloudT);

}



ClientWidget::~ClientWidget()
{
    on_pushButton_Disconnect_clicked();
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
QString ClientWidget::getIP(){return ui->lineEdit_IP->text().replace(" ","");}
QString ClientWidget::getPort(){return ui->lineEdit_Port->text();}
QString ClientWidget::getLastMessage(){return ui->lineEdit_Message->text();}


PointCloudT::Ptr ClientWidget::getPointCloud(int index)
{
    if (index == 0) return cloud0;
    else if (index == 1) return cloud1;
}

void ClientWidget::setPointCloud(int index, PointCloudT::Ptr PC)
{
    if (index == 0)
    {
        cloud0 = PC;
        ui->transformationWidget_Kin1->setTransform(getPointCloudPose(0));
        ui->transformationWidget_Kin1->emitTransform();
    }
    else if (index == 1)
    {
        cloud1 = PC;
        ui->transformationWidget_Kin2->setTransform(getPointCloudPose(1));
        ui->transformationWidget_Kin2->emitTransform();
    }
}

Transform ClientWidget::getPointCloudPose(int index)
{
    if (index == 0) return Transform(cloud0->sensor_origin_, cloud0->sensor_orientation_);
    else if (index == 1) return Transform(cloud1->sensor_origin_, cloud1->sensor_orientation_);
}


void ClientWidget::setPointCloudPose(int index, Transform T)
{
    if (index == 0)
    {
        ui->transformationWidget_Kin1->setTransform(T);
        ui->transformationWidget_Kin1->emitTransform();
    }
    else if (index == 1)
    {
        ui->transformationWidget_Kin2->setTransform(T);
        ui->transformationWidget_Kin2->emitTransform();
    }
}


// CLICK
// Socket Connection
void ClientWidget::on_pushButton_Connect_clicked()
{
    if (mySocket->state() != QAbstractSocket::UnconnectedState)
        return;

    mySocket->connectToHost(ui->lineEdit_IP->text(), ui->lineEdit_Port->text().toInt());

    // Auto Completion
    IPhistory->append(ui->lineEdit_IP->text());
    IPhistory->removeDuplicates();
    IPhistory->sort();
    QCompleter *comp = new QCompleter(*IPhistory, this);
    comp->setCaseSensitivity(Qt::CaseInsensitive);
    ui->lineEdit_IP->setCompleter(comp);


    return;
}

void ClientWidget::on_pushButton_Disconnect_clicked()
{
    if (mySocket->state() != QAbstractSocket::ConnectedState)
        return;

    mySocket->disconnectFromHost();
}


// Message Writing
void ClientWidget::on_pushButton_Send_clicked()                 {   WriteMessage(ui->lineEdit_Message->text());}
void ClientWidget::on_pushButton_Connect_Devices_clicked()      {   WriteMessage(QString(PROTOCOL_OPEN));}
void ClientWidget::on_pushButton_Disconnect_Devices_clicked()   {   WriteMessage(QString(PROTOCOL_CLOSE));}
void ClientWidget::on_pushButton_Register_clicked()             {   WriteMessage(QString(PROTOCOL_REGISTER));}
void ClientWidget::on_pushButton_Save_Settings_clicked()        {   WriteMessage(QString(PROTOCOL_SAVE_SETTINGS));}
void ClientWidget::on_comboBox_pipeline_activated(const QString &arg1){   WriteMessage(QString(PROTOCOL_PIPELINE+arg1));}
void ClientWidget::on_checkBox_savePC_clicked(bool checked)     {   WriteMessage(QString("%1%2").arg(PROTOCOL_SAVE).arg((int)checked));}


void ClientWidget::on_pushButton_Grab_Devices_clicked()
{
    if (ui->checkBox_autoGetPointClouds->isChecked())
    {
         WriteMessageAndWaitForAnswer(QString(PROTOCOL_GRAB));
         on_pushButton_GetPointCloud_clicked();
    }
    else
    {
        WriteMessage(QString(PROTOCOL_GRAB));
    }
}
void ClientWidget::on_pushButton_GetPointCloud_clicked()
{


    QString answer =  WriteMessageAndWaitForAnswer(QString(PROTOCOL_TRANSMIT_POINTCLOUDS));





    if (answer.split(":").length() != 2)
        return;

    QString remotepath0 = answer.split(":").at(0);
    QString remotepath1 = answer.split(":").at(1);
    qDebug() << remotepath0;
    qDebug() << remotepath1;

    QString localpath0 = remotepath0;
    localpath0.replace("sineco","silvio");
    QString localpath1 = remotepath1;
    localpath1.replace("sineco","silvio");
    qDebug() << localpath0;
    qDebug() << localpath1;

    // create folders
    QDir().mkpath(QFileInfo(localpath0).absolutePath());
    QDir().mkpath(QFileInfo(localpath1).absolutePath());



    QString log = "->" + QString(PROTOCOL_TRANSMIT_POINTCLOUDS);

    QString cmdline0 = QString("scp sineco@%1:%2 %3").arg(ui->lineEdit_IP->text()).arg(remotepath0).arg(localpath0);
    qDebug() << cmdline0;
    proc.start(cmdline0);
    if (proc.waitForFinished() == false)
        log.append(": ERR");
    else  log.append(": OK");


    QString cmdline1 = QString("scp sineco@%1:%2 %3").arg(ui->lineEdit_IP->text()).arg(remotepath1).arg(localpath1);
    qDebug() << cmdline1;
    proc.start(cmdline1);
    if (proc.waitForFinished() == false)
        log.append(": ERR");
    else  log.append(": OK");

    ui->logWidget_received->appendText(log);

    // qDebug() << "load PC0";

    cloud0.reset(new PointCloudT);
    pcl::io::loadPCDFile(localpath0.toStdString(), *cloud0);
    cloud0->header.frame_id = localpath0.section("/",-1, -1).section("_",-1,-1).section(".",-2,-2).toStdString();
    Transform(cloud0->sensor_origin_, cloud0->sensor_orientation_).print();
    emit PCtransmitted(cloud0);
    ui->transformationWidget_Kin1->setTransform(getPointCloudPose(0));

    // qDebug() << "load PC1";
    cloud1.reset(new PointCloudT);
    pcl::io::loadPCDFile(localpath1.toStdString(), *cloud1);
    cloud1->header.frame_id = localpath1.section("/",-1, -1).section("_",-1,-1).section(".",-2,-2).toStdString();
    Transform(cloud1->sensor_origin_, cloud1->sensor_orientation_).print();
    emit PCtransmitted(cloud1);
    ui->transformationWidget_Kin2->setTransform(getPointCloudPose(1));

}



// SSH communication

void ClientWidget::on_pushButton_SSHReboot_clicked()
{
    if (proc.state() != QProcess::NotRunning)
    {
        qDebug() << "Proc already opened";
        return;
    }

    runProc(QString("ssh sineco@%1 sudo reboot").arg(ui->lineEdit_IP->text()));

}

void ClientWidget::on_pushButton_SSHUpdate_clicked()
{
    // check if running
    if (proc.state() != QProcess::NotRunning)
    {
        qDebug() << "Proc already opened";
        return;
    }


    // Remove old
    runProc(QString("ssh sineco@%1 rm -r /home/sineco/Kinect2TCPIP").arg(ui->lineEdit_IP->text()));


    // Create Folder
    runProc(QString("ssh sineco@%1 mkdir -p /home/sineco/Kinect2TCPIP").arg(ui->lineEdit_IP->text()));

    // Copy Compilation script
    runProc(QString("scp /home/silvio/git/Kinect2TCPIP/CompileScript.sh sineco@%1:/home/sineco/Kinect2TCPIP").arg(ui->lineEdit_IP->text()));

    // Copy new src files
    runProc(QString("scp -r /home/silvio/git/Kinect2TCPIP/src sineco@%1:/home/sineco/Kinect2TCPIP").arg(ui->lineEdit_IP->text()));

    // Compile
    proc.start(QString("ssh sineco@%1 sh /home/sineco/Kinect2TCPIP/CompileScript.sh").arg(ui->lineEdit_IP->text()));


}

void ClientWidget::runProc(QString cmdline)
{
    qDebug() << cmdline;
    proc.start(cmdline);
    if (proc.waitForFinished() == false)
        ui->logWidget_ssh->appendText("Timeout reached");
    else   ui->logWidget_ssh->appendText("Done");
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
    ui->logWidget_ssh->appendText(output);
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
    ui->logWidget_received->appendText(message);
}



// Read / Write
void ClientWidget::WriteMessage(QString message)
{
    mySocket->write(message.toStdString().c_str());
    ui->logWidget_sent->appendText(message);
    mySocket->waitForBytesWritten(1000);
    return;
}

QString ClientWidget::readAnswer()
{
    mySocket->waitForReadyRead(1000);
    return QString::fromLocal8Bit(mySocket->readAll());
}

QString ClientWidget::WriteMessageAndWaitForAnswer(QString message)
{
    if (mySocket->state() != QAbstractSocket::ConnectedState)
        return QString("");


    // disconnect auto read
    disconnect(mySocket, SIGNAL(readyRead()), this, SLOT (newMessageReceived()));


    ui->logWidget_sent->appendText(message);
    mySocket->write(message.toStdString().c_str());
    mySocket->waitForBytesWritten(1000);

    mySocket->waitForReadyRead(1000);
    QString answer =  QString::fromLocal8Bit(mySocket->readAll());


    // reconnect auto read
    connect(mySocket, SIGNAL(readyRead()), this, SLOT (newMessageReceived()));

    return answer;
}





void ClientWidget::on_transformationWidget_Kin1_matrixchanged( Transform T)
{
    // qDebug() << "here";
    QString pose = T.prettyprint();
    WriteMessage(QString("%1%2_%3").arg(PROTOCOL_POSE).arg(0).arg(pose));
}

void ClientWidget::on_transformationWidget_Kin2_matrixchanged(Transform T)
{
    const QString pose = T.prettyprint();
    WriteMessage(QString("%1%2_%3").arg(PROTOCOL_POSE).arg(1).arg(pose));
}


