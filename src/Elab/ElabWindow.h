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

    void on_actionShowPC_triggered();

    void on_LineAlign_clicked();

    void on_EstimateNormals_clicked();

    void on_RemoveOutliers_clicked();

private:
    Ui::ElabWindow *ui;



    void writeSettings();
    void readSettings();


    MatrixOfCloud myMatrixOfPointCloud;

  //  QStringList IDorder = {0,0,0,0};

};
#endif // ElabWindow_H
