#ifndef CLOUDVIEWER_H_
#define CLOUDVIEWER_H_

#include <QVTKWidget.h>
#include <QDebug>
#include <QColorDialog>
#include <QSettings>

#include "define.h"

#include <pcl/visualization/pcl_visualizer.h>
#include <vtkCamera.h>
#include <vtkRenderWindow.h>

#include <QMenu>
#include <QtGui/QContextMenuEvent>

//#include <CloudListModel.h>


namespace Ui {
class CloudViewer;
}

class CloudViewer : public QVTKWidget
{
    Q_OBJECT

public:
    CloudViewer(QWidget * parent = 0);
    virtual ~CloudViewer();


    // checkable Action
    bool isMainReferenceSystemShown() const;
    void setMainReferenceSystemShown(bool shown);
    bool isPCReferenceSystemShown() const;
    void setPCReferenceSystemShown(bool shown);

    // single shot Action
    void doClearPointClouds();
    void changeBackGroundColor(QColor Color = QColor());


protected:
    virtual void contextMenuEvent(QContextMenuEvent * event);

public slots:

    //void setModel(CloudListModel *MyModel);

    //PC
    void showPC(PointCloudT::Ptr PC, std::string str = "");
    void showPC(PointCloudNormalT::Ptr PC, std::string str = "");
    void removePC(std::string str);
    void removePC(QString str);
    void removePC(PointCloudT::Ptr PC);
    void replacePC(PointCloudT::Ptr PC);

    void setPointCloudPose(std::string, Transform);
    void setPointCloudPose(QString, Transform);

    void setBackGroundColor(QColor);
    QColor getBackGroundColor();



private slots:
    void on_actionClearViewer_triggered();
    void on_actionSet_Background_Color_triggered();
    void on_actionShowMainRefSyst_triggered(bool value);
    void on_actionShowPCRefSyst_triggered(bool value);


private:
    Ui::CloudViewer *ui;
    QMenu * _menu;
    pcl::visualization::PCLVisualizer::Ptr _visualizer;

    QColor _backgroundColor;
    //CloudListModel *myModel;

    QSettings settings;


};

#endif /* CLOUDVIEWER_H_ */
