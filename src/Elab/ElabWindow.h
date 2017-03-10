#ifndef ElabWindow_H
#define ElabWindow_H

#include <QMainWindow>
#include <QDebug>
#include <QSettings>
#include <QFileDialog>
#include <QStringList>
#include <define.h>


#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <MatrixOfCloud.h>

#include <stdio.h>
#include <sstream>
#include <stdlib.h>




namespace Ui {
class ElabWindow;
}

class ElabWindow : public QMainWindow
{
    Q_OBJECT
    
public:
    explicit ElabWindow(QWidget *parent = 0);
    ~ElabWindow();

private slots:
    void on_actionOpenAcq_triggered();
    void on_actionSavePointCloudAs_triggered();
    void on_actionExportReconstruction_triggered();

    void on_actionShowPC_triggered();

    void on_EstimateNormals_clicked();
    void on_RemoveOutliers_clicked();
    void on_LineAlign_clicked();
    void on_LineRANSAC_clicked();
    void on_BackBoneAlignICP_clicked();
    void on_BackBoneAlignRANSAC_clicked();


  /*  void on_comboBox_activated(const QString &arg1);

    void on_comboBox_2_activated(const QString &arg1);*/


    void on_doubleSpinBox_valueChanged(double arg1);

  //  void on_ICP_Decimation_valueChanged(double arg1);

    void on_ICP_iteration_valueChanged(int arg1);

    void on_doubleSpinBox_2_valueChanged(double arg1);

    void on_spinBox_valueChanged(int arg1);

    void on_doubleSpinBox_3_valueChanged(double arg1);

    void on_doubleSpinBox_4_valueChanged(double arg1);

    void on_doubleSpinBox_8_valueChanged(double arg1);

    void on_doubleSpinBox_6_valueChanged(double arg1);

    void on_spinBox_3_valueChanged(int arg1);

    void on_spinBox_4_valueChanged(int arg1);

    void on_doubleSpinBox_7_valueChanged(double arg1);

    void on_spinBox_2_valueChanged(int arg1);

    void on_radioButton_objectNameChanged(const QString &objectName);

    void on_tabWidget_currentChanged(int index);

    void on_Downsampling_currentChanged(int index);

    void on_checkBox_clicked(bool checked);

    void on_pushButton_ApplyT_clicked();

    void on_PreOrient_clicked();

private:
    Ui::ElabWindow *ui;



    void writeSettings();
    void readSettings();


    MatrixOfCloud myMatrixOfPointCloud;


};
#endif // ElabWindow_H
