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



TransformT TransformationWidget::getTransform()
{
    Eigen::Translation<float,3> translation (ui->TranslationX->value(), ui->TranslationY->value(), ui->TranslationZ->value());
    Eigen::AngleAxis<float> alfa (ui->RotationX->value()/180.0*M_PI, Eigen::Vector3f(1,0,0));
    Eigen::AngleAxis<float> beta (ui->RotationY->value()/180.0*M_PI, Eigen::Vector3f(0,1,0));
    Eigen::AngleAxis<float> gamma (ui->RotationZ->value()/180.0*M_PI, Eigen::Vector3f(0,0,1));

    TransformT transformation = translation * alfa * beta * gamma;

    return transformation;
}

void TransformationWidget::setTransform(TransformT transform)
{
    Eigen::Vector3f t = transform.translation();
    ui->TranslationX->setValue(t.x());
    ui->TranslationY->setValue(t.y());
    ui->TranslationZ->setValue(t.z());


    Eigen::Matrix3f mat = transform.rotation();
    Eigen::Vector3f ea = mat.eulerAngles(0, 1, 2);
    ui->RotationX->setValue(ea(0)*180.0/M_PI);
    ui->RotationY->setValue(ea(1)*180.0/M_PI);
    ui->RotationZ->setValue(ea(2)*180.0/M_PI);

    return;
}


