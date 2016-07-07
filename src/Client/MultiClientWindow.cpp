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
    PC_target = getPointCloud(i_target);

    PointCloudT::Ptr PC_input(new PointCloudT);
    PC_input = getPointCloud(i_input);

    PointCloudT::Ptr PC_result(new PointCloudT);
    pcl::copyPointCloud(*PC_input,*PC_result);


    Transform trans = utils::getTransformation(PC_target, PC_input);

    trans.print();



    Transform currentPose = getPointCloudPose(i_input);

    //       Transform(PC_result->sensor_origin_, PC_result->sensor_orientation_);
  //  currentPose.print();



   // Transform refPose = getPointCloudPose(i_target);

    trans = trans.postmultiplyby(currentPose);
    trans.print();



    setPointCloudPose(i_input,trans);

   /* ui->myKinectWidget2->setTransform(trans);

    currentPose = currentPose.premultiplyby(trans);//.postmultiplyby(refPose);
   // currentPose.print();

    PC_result->sensor_orientation_ = currentPose.getQuaternion();
    PC_result->sensor_origin_ = currentPose.getOrigin4();
*/

    //setPointCloud(i_input, PC_result);*/




}
