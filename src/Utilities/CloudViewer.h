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


protected:
    virtual void contextMenuEvent(QContextMenuEvent * event);

public slots:

    //PC
    void showPC(PointCloudT::Ptr PC);
    void removePC(std::string str);
    void removePC(QString str);

    //ON/OFF
    void showReferenceSystemPointCloud(bool value);
    void showReferenceSystemGlobal(bool value);


private slots:
    void on_actionClearViewer_triggered();

private:
    Ui::CloudViewer *ui;
QMenu * _menu;
    pcl::visualization::PCLVisualizer::Ptr _visualizer;
    bool _showReferenceSystemPointCloud;

};

#endif /* CLOUDVIEWER_H_ */
