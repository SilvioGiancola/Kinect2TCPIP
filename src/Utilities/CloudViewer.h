#ifndef CLOUDVIEWER_H_
#define CLOUDVIEWER_H_

#include <QVTKWidget.h>
#include <QDebug>

#include "define.h"

#include <pcl/visualization/pcl_visualizer.h>
#include <vtkCamera.h>
#include <vtkRenderWindow.h>


namespace Ui {
class CloudViewer;
}

class CloudViewer : public QVTKWidget
{
	Q_OBJECT

public:
	CloudViewer(QWidget * parent = 0);
	virtual ~CloudViewer();



public slots:
    void clear();

    //PC
    void showPC(PointCloudT::Ptr PC);
    void removePC(std::string str);
    void removePC(QString str);

    //ON/OFF
    void showReferenceSystemPointCloud(bool value);
    void showReferenceSystemGlobal(bool value);


private:
    Ui::CloudViewer *ui;

    pcl::visualization::PCLVisualizer::Ptr _visualizer;
    bool _showReferenceSystemPointCloud;

};

#endif /* CLOUDVIEWER_H_ */
