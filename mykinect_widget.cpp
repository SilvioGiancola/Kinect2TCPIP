#include "mykinect_widget.h"
#include "ui_mykinect_widget.h"

mykinect_widget::mykinect_widget(QWidget *parent) :
    QWidget(parent),
    ui(new Ui::mykinect_widget)
{
    ui->setupUi(this);

    libfreenect2::Freenect2 freenect2;
    int n = freenect2.enumerateDevices();
    for (int i = 0; i < n; i++)
        ui->comboBox_KinectSerials->addItem(QString::fromStdString(freenect2.getDeviceSerialNumber(i)));

}

mykinect_widget::~mykinect_widget()
{
    delete ui;
}


void mykinect_widget::defineKinect (MyKinect *kin)
{
    _kin = kin;
    int index = ui->comboBox_KinectSerials->findText(kin->getSerial()) ;
    ui->comboBox_KinectSerials->setCurrentIndex(index);
    return;
}



void mykinect_widget::on_comboBox_KinectSerials_activated(const QString &arg1)
{
    _kin->setSerial(arg1);
}

void mykinect_widget::on_pushButton_Open_clicked()
{
    if (_kin->Open() == SUCCESS)
        ui->label_status->setText("Open");

}

void mykinect_widget::on_pushButton_Close_clicked()
{
    if(_kin->Close() == SUCCESS)
        ui->label_status->setText("Closed");

}

void mykinect_widget::on_pushButton_Grab_clicked()
{
    _kin->Grab();
}
