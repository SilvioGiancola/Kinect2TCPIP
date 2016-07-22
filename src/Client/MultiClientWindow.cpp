#include "MultiClientWindow.h"
#include "ui_MultiClientWindow.h"

MultiClientWindow::MultiClientWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MultiClientWindow)
{
    ui->setupUi(this);

    ui->centralWidget->setVisible(false);

    IPhistory = new QStringList();

    ui->widget_clientA->setIPCompletion(IPhistory);
    ui->widget_clientB->setIPCompletion(IPhistory);
    readSettings();

    // Repeated message
    timer1 = new QTimer();
    connect(timer1, SIGNAL(timeout()), ui->widget_clientA, SLOT(on_pushButton_Grab_Devices_clicked()));
    connect(timer1, SIGNAL(timeout()), ui->widget_clientB, SLOT(on_pushButton_Grab_Devices_clicked()));


}



MultiClientWindow::~MultiClientWindow()
{
    writeSettings();
    this->on_pushButton_DisconnectALL_clicked();
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

void MultiClientWindow::on_pushButton_RepeatGrabALL_clicked()
{

    if (timer1->isActive())
    {
        timer1->stop();
        ui->doubleSpinBox_time_Resend->setEnabled(true);
    }
    else
    {
        timer1->start(ui->doubleSpinBox_time_Resend->value()*1000);
        ui->doubleSpinBox_time_Resend->setEnabled(false);
    }
}

void MultiClientWindow::on_pushButton_TransmitPointCloud_clicked()
{
    ui->widget_clientA->on_pushButton_GetPointCloud_clicked();
    ui->widget_clientB->on_pushButton_GetPointCloud_clicked();
}

void MultiClientWindow::showPointCloud(PointCloudT::Ptr PC)
{
    ui->widget->showPC(PC);
}

PointCloudT::Ptr MultiClientWindow::getPointCloud(int index)
{
    if (index == 1) return ui->widget_clientA->getPointCloud(0);
    if (index == 2) return ui->widget_clientA->getPointCloud(1);
    if (index == 3) return ui->widget_clientB->getPointCloud(0);
    if (index == 4) return ui->widget_clientB->getPointCloud(1);
}

Transform MultiClientWindow::getPointCloudPose(int index)
{
    if (index == 1) return ui->widget_clientA->getPointCloudPose(0);
    if (index == 2) return ui->widget_clientA->getPointCloudPose(1);
    if (index == 3) return ui->widget_clientB->getPointCloudPose(0);
    if (index == 4) return ui->widget_clientB->getPointCloudPose(1);
}

void MultiClientWindow::setPointCloudPose(int index, Transform T)
{
    if (index == 1) return ui->widget_clientA->setPointCloudPose(0, T);
    if (index == 2) return ui->widget_clientA->setPointCloudPose(1, T);
    if (index == 3) return ui->widget_clientB->setPointCloudPose(0, T);
    if (index == 4) return ui->widget_clientB->setPointCloudPose(1, T);
}

void MultiClientWindow::setPointCloud(int index, PointCloudT::Ptr PC)
{
    if (index == 1) return ui->widget_clientA->setPointCloud(0, PC);
    if (index == 2) return ui->widget_clientA->setPointCloud(1, PC);
    if (index == 3) return ui->widget_clientB->setPointCloud(0, PC);
    if (index == 4) return ui->widget_clientB->setPointCloud(1, PC);
}





void MultiClientWindow::on_pushButton_RegisterLocally_clicked()
{
    int i_target = ui->spinBox_PC_Target->value();
    int i_input = ui->spinBox_PC_Input->value();

    PointCloudT::Ptr PC_target(new PointCloudT);
    PointCloudT::Ptr PC_input(new PointCloudT);

    PC_target = getPointCloud(i_target);
    PC_input = getPointCloud(i_input);

    PointCloudT::Ptr PCnew(new PointCloudT);
    PCnew = getPointCloud(i_input);

    Transform currentPose = Transform(PCnew->sensor_origin_, PCnew->sensor_orientation_);


    Transform T = utils::getTransformation(PC_target, PC_input);

    T.print();


    T = T.postmultiplyby(currentPose);
    T.print();


    setPointCloudPose(i_input,T);

}

void MultiClientWindow::on_pushButton_reg12_clicked()
{
    on_pushButton_TransmitPointCloud_clicked();
    ui->spinBox_PC_Target->setValue(1);
    ui->spinBox_PC_Input->setValue(2);
    on_pushButton_RegisterLocally_clicked();
}

void MultiClientWindow::on_pushButton_reg23_clicked()
{
    on_pushButton_TransmitPointCloud_clicked();
    ui->spinBox_PC_Target->setValue(2);
    ui->spinBox_PC_Input->setValue(3);
    on_pushButton_RegisterLocally_clicked();
}

void MultiClientWindow::on_pushButton_reg34_clicked()
{
    on_pushButton_TransmitPointCloud_clicked();
    ui->spinBox_PC_Target->setValue(3);
    ui->spinBox_PC_Input->setValue(4);
    on_pushButton_RegisterLocally_clicked();
}


void MultiClientWindow::on_pushButton_Clean_clicked()
{

    pcl::RadiusOutlierRemoval<PointT> outrem;
    // build the filter
    outrem.setRadiusSearch(0.1);
    outrem.setMinNeighborsInRadius(5);
    outrem.setNegative(false);



    for (int i = 1; i < 5; i++)
    {

        PointCloudT::Ptr PC_input(new PointCloudT);
        PC_input = getPointCloud(i);


        PointCloudT::Ptr PC_output(new PointCloudT);
        pcl::copyPointCloud(*PC_input, *PC_output);


        outrem.setInputCloud(PC_input);
        outrem.setKeepOrganized(PC_input->isOrganized());

        // apply filter
        outrem.filter (*PC_output);

        // update PC
        setPointCloud(i, PC_output);

        ui->widget->replacePC(PC_output);


    }
}

void MultiClientWindow::on_doubleSpinBox_OffsetCampata_valueChanged(double arg1)
{
    Transform Trans(0,arg1,0,0,0,0);
    ui->widget_clientA->setOffsetCampata(Trans);
    ui->widget_clientB->setOffsetCampata(Trans);
}

void MultiClientWindow::on_pushButton_SavePointCloud_clicked()
{
    for (int i = 1; i < 5; i++)
    {
        PointCloudT::Ptr PC(new PointCloudT);
        PC = getPointCloud(i);

        QString Path = QString("%1/PointClouds/%2/%4m_%3.pcd")
                .arg(QDir::homePath())
                .arg(ui->lineEdit_CampataPath->text())
                .arg(QString::fromStdString(PC->header.frame_id))
                .arg(QString::number(ui->doubleSpinBox_OffsetCampata->value(),'f',1));

        QDir().mkpath(QFileInfo(Path).absolutePath());

        qDebug() << Path;

        pcl::io::savePCDFileBinary(Path.toStdString(), *PC);


    }
}
