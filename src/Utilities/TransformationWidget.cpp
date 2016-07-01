#include "TransformationWidget.h"
#include "ui_TransformationWidget.h"

TransformationWidget::TransformationWidget(QWidget *parent) :
    QWidget(parent),
    ui(new Ui::TransformationWidget)
{
    ui->setupUi(this);
}

TransformationWidget::~TransformationWidget()
{
    delete ui;
}



Transform TransformationWidget::getTransform()
{
    return Transform(ui->TranslationX->value(),
                     ui->TranslationY->value(),
                     ui->TranslationZ->value(),
                     ui->RotationX->value()*M_PI/180.0,
                     ui->RotationY->value()*M_PI/180.0,
                     ui->RotationZ->value()*M_PI/180.0);
}

void TransformationWidget::setTransform(Transform transform)
{
    Eigen::Vector4f orig = transform.getOrigin4();
    ui->TranslationX->setValue(orig(0));
    ui->TranslationY->setValue(orig(1));
    ui->TranslationZ->setValue(orig(2));

    Eigen::Vector3f ea = transform.getEulerAngles();
    ui->RotationX->setValue(ea(0)*180.0/M_PI);
    ui->RotationY->setValue(ea(1)*180.0/M_PI);
    ui->RotationZ->setValue(ea(2)*180.0/M_PI);

    return;
}



void TransformationWidget::on_pushButton_reset_clicked()
{
    setTransform(Transform());
}
