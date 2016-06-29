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
}



ClientWidget::~ClientWidget()
{
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




// SSH communication



void ClientWidget::on_groupBox_ExpertMode_toggled(bool checked)
{
    if (checked)
    {
        ui->pushButton_SSHReboot->setVisible(true);
        ui->pushButton_SSHUpdate->setVisible(true);
        ui->pushButton_SSHClientCompile->setVisible(true);
    }
    else
    {
        ui->pushButton_SSHReboot->setVisible(false);
        ui->pushButton_SSHUpdate->setVisible(false);
        ui->pushButton_SSHClientCompile->setVisible(false);
    }
}


void ClientWidget::on_pushButton_SSHReboot_clicked()
{
    this->setEnabled(false);
    QProcess proc;
    proc.start(QString("ssh sineco@%1 sudo reboot").arg(ui->lineEdit_IP->text()));
    if (proc.waitForFinished() == false)
        qWarning() << "Reboot Timeout reached";
    else qDebug() << "Reboot Done";
    this->setEnabled(true);
}

void ClientWidget::on_pushButton_SSHUpdate_clicked()
{
    this->setEnabled(false);
    QProcess proc;

  /*  proc.start(QString("ssh sineco@%1 mkdir -p /home/sineco/Kinect2TCPIP/build-Server").arg(ui->lineEdit_IP->text()));
    if (proc.waitForFinished() == false)
        qWarning() << "Update Timeout reached";
    else qDebug() << "Update Done";*/


    proc.start(QString("ssh sineco@%1 mkdir -p /home/sineco/Kinect2TCPIP/build-Server").arg(ui->lineEdit_IP->text()));
    if (proc.waitForFinished() == false)
        qWarning() << "Update Timeout reached";
    else qDebug() << "Update Done";

    proc.start(QString("scp /home/silvio/git/Kinect2TCPIP/CompileScript.sh sineco@%1:/home/sineco/Kinect2TCPIP").arg(ui->lineEdit_IP->text()));
    if (proc.waitForFinished() == false)
        qWarning() << "Update Timeout reached";
    else qDebug() << "Update Done";

    proc.start(QString("scp -r /home/silvio/git/Kinect2TCPIP/src sineco@%1:/home/sineco/Kinect2TCPIP").arg(ui->lineEdit_IP->text()));
    if (proc.waitForFinished() == false)
        qWarning() << "Update Timeout reached";
    else qDebug() << "Update Done";

    this->setEnabled(true);
}

void ClientWidget::on_pushButton_SSHClientCompile_clicked()
{
    this->setEnabled(false);
    QProcess proc;
    proc.startDetached(QString("ssh sineco@%1 sh /home/sineco/Kinect2TCPIP/CompileScript.sh").arg(ui->lineEdit_IP->text()));
    this->setEnabled(true);
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
    QString message = QString(mySocket->readAll());
    ui->plainTextEdit_received->appendPlainText(QString("[%1]: %2").arg(QDateTime::currentDateTime().toString(TIMEFORMAT)).arg(message));
}

void ClientWidget::WriteMessage(QString message)
{
    mySocket->write(message.toStdString().c_str());
    ui->plainTextEdit_sent->appendPlainText(QString("[%1]: %2").arg(QDateTime::currentDateTime().toString()).arg(message));
    mySocket->waitForBytesWritten(1000);
    return;
}


