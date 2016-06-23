#include "MultiClientWindow.h"
#include "ui_MultiClientWindow.h"

MultiClientWindow::MultiClientWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MultiClientWindow)
{
    ui->setupUi(this);

    readSettings();
}



MultiClientWindow::~MultiClientWindow()
{
  //  mySocket->disconnectFromHost();
    writeSettings();
    delete ui;
}


void MultiClientWindow::writeSettings()
{
    QSettings settings("SilvioGiancola", "Kinect 2 TCPIP");

    settings.beginGroup("MultiClientWindow");
    settings.setValue("SINECO2 IP A",ui->widget_clientA->getIP());
    settings.setValue("SINECO2 Port A",ui->widget_clientA->getPort());
    settings.setValue("SINECO2 IP B",ui->widget_clientB->getIP());
    settings.setValue("SINECO2 Port B",ui->widget_clientB->getPort());
    settings.endGroup();
}

void MultiClientWindow::readSettings()
{
    QSettings settings("SilvioGiancola", "Kinect 2 TCPIP");

    settings.beginGroup("MultiClientWindow");
    ui->widget_clientA->setIP(settings.value("SINECO2 IP A","0.0.0.0").toString());
    ui->widget_clientA->setPort(settings.value("SINECO2 Port A","8080").toString());
    ui->widget_clientB->setIP(settings.value("SINECO2 IP B","0.0.0.0").toString());
    ui->widget_clientB->setPort(settings.value("SINECO2 Port B","8080").toString());
    settings.endGroup();
}



