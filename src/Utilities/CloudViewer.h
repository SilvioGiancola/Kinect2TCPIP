#ifndef CLOUDVIEWER_H_
#define CLOUDVIEWER_H_

#include <QVTKWidget.h>
#include <QDebug>

#include "define.h"

#include <pcl/visualization/pcl_visualizer.h>
#include <vtkCamera.h>
#include <vtkRenderWindow.h>

#include <QMenu>
#include <QtGui/QContextMenuEvent>


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


protected:
    virtual void contextMenuEvent(QContextMenuEvent * event);

public slots:

    //PC
    void showPC(PointCloudT::Ptr PC);
    void removePC(std::string str);
    void removePC(QString str);

    void setPointCloudPose(std::string, Transform);
    void setPointCloudPose(QString, Transform);



private slots:
    void on_actionClearViewer_triggered();
    void on_actionShowMainRefSyst_triggered(bool value);
    void on_actionShowPCRefSyst_triggered(bool value);

private:
    Ui::CloudViewer *ui;
    QMenu * _menu;
    pcl::visualization::PCLVisualizer::Ptr _visualizer;


};

#endif /* CLOUDVIEWER_H_ */
