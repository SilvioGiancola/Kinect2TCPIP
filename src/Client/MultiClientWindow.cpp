#include "MultiClientWindow.h"
#include "ui_MultiClientWindow.h"

MultiClientWindow::MultiClientWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MultiClientWindow)
{
    ui->setupUi(this);

    IPhistory = new QStringList();

    ui->widget_clientA->setIPCompletion(IPhistory);
    ui->widget_clientB->setIPCompletion(IPhistory);
    readSettings();
}



MultiClientWindow::~MultiClientWindow()
{
    ui->widget_clientA->on_pushButton_Disconnect_Devices_clicked();
    ui->widget_clientB->on_pushButton_Disconnect_Devices_clicked();
    writeSettings();
    ui->widget_clientA->on_pushButton_Disconnect_clicked();
    ui->widget_clientB->on_pushButton_Disconnect_clicked();
    delete ui;
}


void MultiClientWindow::writeSettings()
{
    QSettings settings("SilvioGiancola", "Kinect 2 TCPIP");

    settings.beginGroup("MultiClientWindow");
    settings.setValue("SINECO2 IP A",ui->widget_clientA->getIP());
    settings.setValue("SINECO2 IP B",ui->widget_clientB->getIP());
    settings.setValue("SINECO2 Port A",ui->widget_clientA->getPort());
    settings.setValue("SINECO2 Port B",ui->widget_clientB->getPort());
    settings.setValue("SINECO2 PreviousMessage A",ui->widget_clientA->getLastMessage());
    settings.setValue("SINECO2 PreviousMessage B",ui->widget_clientB->getLastMessage());

    IPhistory->removeDuplicates();
    IPhistory->sort();
    settings.setValue("SINECO2 Completion", *IPhistory);

    settings.endGroup();
}

void MultiClientWindow::readSettings()
{
    QSettings settings("SilvioGiancola", "Kinect 2 TCPIP");

    settings.beginGroup("MultiClientWindow");
    ui->widget_clientA->setIP(settings.value("SINECO2 IP A","0.0.0.0").toString());
    ui->widget_clientB->setIP(settings.value("SINECO2 IP B","0.0.0.0").toString());

    ui->widget_clientA->setPort(settings.value("SINECO2 Port A","8080").toString());
    ui->widget_clientB->setPort(settings.value("SINECO2 Port B","8080").toString());

    ui->widget_clientA->setMessage(settings.value("SINECO2 PreviousMessage A","").toString());
    ui->widget_clientB->setMessage(settings.value("SINECO2 PreviousMessage B","").toString());

    *IPhistory = settings.value("SINECO2 Completion","").toStringList();
    IPhistory->removeDuplicates();
    IPhistory->sort();
    ui->widget_clientA->setIPCompletion(IPhistory);
    ui->widget_clientB->setIPCompletion(IPhistory);

    settings.endGroup();
}




void MultiClientWindow::on_pushButton_ConnectALL_clicked()
{
    ui->widget_clientA->on_pushButton_Connect_clicked();
    ui->widget_clientB->on_pushButton_Connect_clicked();
    ui->widget_clientA->on_pushButton_Connect_Devices_clicked();
    ui->widget_clientB->on_pushButton_Connect_Devices_clicked();
}

void MultiClientWindow::on_pushButton_DisconnectALL_clicked()
{
    ui->widget_clientA->on_pushButton_Disconnect_Devices_clicked();
    ui->widget_clientB->on_pushButton_Disconnect_Devices_clicked();
    ui->widget_clientA->on_pushButton_Disconnect_clicked();
    ui->widget_clientB->on_pushButton_Disconnect_clicked();
}

void MultiClientWindow::on_pushButton_GrabALL_clicked()
{
    ui->widget_clientA->on_pushButton_Grab_Devices_clicked();
    ui->widget_clientB->on_pushButton_Grab_Devices_clicked();
}

void MultiClientWindow::on_pushButton_SendALL_clicked()
{
    ui->widget_clientA->on_pushButton_Send_clicked();
    ui->widget_clientB->on_pushButton_Send_clicked();
}


void MultiClientWindow::showPointCloud(PointCloudT::Ptr PC)
{
    ui->widget->showPC(PC);
}
