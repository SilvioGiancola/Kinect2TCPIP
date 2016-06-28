#include "KinectWidget.h"
#include "ui_KinectWidget.h"

KinectWidget::KinectWidget(QWidget *parent) :
    QWidget(parent),
    ui(new Ui::KinectWidget)
{
    ui->setupUi(this);
}

KinectWidget::~KinectWidget()
{
    delete ui;
}

