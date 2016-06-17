#ifndef mykinect_widget_H
#define mykinect_widget_H

#include <QWidget>
#include <Eigen/Dense>
#include <mykinect.h>

namespace Ui {
class mykinect_widget;
}

class mykinect_widget : public QWidget
{
    Q_OBJECT

public:
    explicit mykinect_widget(QWidget *parent = 0);
    ~mykinect_widget();

    void defineKinect (MyKinect *kin);



private slots:
    void on_comboBox_KinectSerials_activated(const QString &arg1);
    void on_pushButton_Open_clicked();
    void on_pushButton_Close_clicked();
    void on_pushButton_Grab_clicked();

private:
    Ui::mykinect_widget *ui;

    MyKinect *_kin;
};

#endif // mykinect_widget_H
