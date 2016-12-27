#ifndef ViewerWindow_H
#define ViewerWindow_H

#include <QMainWindow>
#include <QDebug>
#include <QSettings>
#include <QFileDialog>
#include <QList>
#include <CloudListModel.h>



#include <define.h>


#include <pcl/io/pcd_io.h> // for opening & saving .pcd
#include <pcl/io/ply_io.h> // for opening & saving .pcd


#include <stdio.h>
#include <sstream>
#include <stdlib.h>



namespace Ui {
class ViewerWindow;
}

class ViewerWindow : public QMainWindow
{
    Q_OBJECT
    
public:
    explicit ViewerWindow(QWidget *parent = 0);
    ~ViewerWindow();

private slots:
    void on_actionOpen_Acquisition_triggered();

private:
    Ui::ViewerWindow *ui;


    QString DefaultDir;

    QList<PointCloudT> PCList;

    void defaultSettings();
    void writeSettings();
    void readSettings();

    CloudListModel *myCloudList;



};
#endif // ViewerWindow_H
