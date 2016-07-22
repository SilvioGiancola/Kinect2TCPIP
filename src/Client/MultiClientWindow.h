#ifndef MultiClientWindow_H
#define MultiClientWindow_H

#include <QMainWindow>
#include <QTcpSocket>
#include <QSettings>
#include <QCompleter>

#include <ClientWidget.h>

#include <define.h>
#include <registration.h>

// Post
#include <pcl/filters/radius_outlier_removal.h>   // remove radius outliers
#include <pcl/kdtree/kdtree_flann.h>

#include <QTimer>


namespace Ui {
class MultiClientWindow;
}

class MultiClientWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MultiClientWindow(QWidget *parent = 0);
    ~MultiClientWindow();

    void showPointCloud(PointCloudT::Ptr PC);
    PointCloudT::Ptr getPointCloud(int index);
    void setPointCloud(int index, PointCloudT::Ptr PC);
    Transform getPointCloudPose(int index);
    void setPointCloudPose(int index, Transform T);


private slots:
    void on_pushButton_ConnectALL_clicked();
    void on_pushButton_DisconnectALL_clicked();
    void on_pushButton_GrabALL_clicked();
    void on_pushButton_SendALL_clicked();

    void on_pushButton_RegisterLocally_clicked();

    void on_pushButton_reg12_clicked();

    void on_pushButton_reg23_clicked();

    void on_pushButton_reg34_clicked();

    void on_pushButton_TransmitPointCloud_clicked();

    void on_pushButton_RepeatGrabALL_clicked();

    void on_pushButton_Clean_clicked();

    void on_doubleSpinBox_OffsetCampata_valueChanged(double arg1);

    void on_pushButton_SavePointCloud_clicked();

private:
    Ui::MultiClientWindow *ui;

    void writeSettings();
    void readSettings();
    QStringList *IPhistory;


    // repeat stuff
    QTimer* timer1;


};

#endif // MultiClientWindow_H
