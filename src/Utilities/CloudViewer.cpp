#include "CloudViewer.h"
#include "ui_CloudViewer.h"

CloudViewer::CloudViewer(QWidget *parent) :
    QVTKWidget(parent),
    ui(new Ui::CloudViewer)
{
    // UI
    ui->setupUi(this);

    this->showReferenceSystemPointCloud(true);
    this->setMouseTracking(false);

    _visualizer.reset(new pcl::visualization::PCLVisualizer("Viewer",false));

    this->SetRenderWindow(_visualizer->getRenderWindow());
    this->GetInteractor()->SetInteractorStyle (_visualizer->getInteractorStyle());

    _visualizer->setCameraPosition(-3.5,1,1, // mi posiziono dietro ad un Kinect
                                   0.5,0.5,0.5, // guardo un punto centrale
                                   0,0,1);   // orientato con la z verso l'alto
    _visualizer->setCameraClipDistances(-10,10);
    _visualizer->setBackgroundColor (0.5, 0.5, 0.5);
    _visualizer->setShowFPS(false);
    this->showReferenceSystemGlobal(true);
    this->update ();


    // Menu
    _menu = new QMenu(this);
    _menu->addAction(ui->actionClearViewer);
}

CloudViewer::~CloudViewer()
{
  //  this->clear();
}


// Visualize Menu
void CloudViewer::contextMenuEvent(QContextMenuEvent * event)
{
    QAction * a = _menu->exec(event->globalPos());
    a->trigger();
}




// ON/OFF
void CloudViewer::showReferenceSystemGlobal(bool value)
{
    if (value) _visualizer->addCoordinateSystem(1.0,"Global Reference System");
    else       _visualizer->removeCoordinateSystem("Global Reference System");
    return;
}

void CloudViewer::showReferenceSystemPointCloud(bool value)
{
    _showReferenceSystemPointCloud = value;
    return;
}



// Visualization PC
void CloudViewer::showPC(PointCloudT::Ptr PC)
{
    pcl::visualization::PointCloudColorHandlerRGBField<PointT> single_color(PC);
    _visualizer->removePointCloud(PC->header.frame_id);
    _visualizer->addPointCloud<PointT>(PC, single_color, PC->header.frame_id);

    if (_showReferenceSystemPointCloud)
    {
        Eigen::Matrix4f trans = Eigen::Matrix4f::Identity();
        trans.block(0,0,3,3) = PC->sensor_orientation_.matrix();
        trans.block(0,3,4,1) = PC->sensor_origin_;
        _visualizer->removeCoordinateSystem(PC->header.frame_id + "Reference system");
        _visualizer->addCoordinateSystem(0.2,Eigen::Affine3f(trans), PC->header.frame_id + "Reference system");
    }
    this->update ();
}

void CloudViewer::removePC(std::string str)
{
    _visualizer->removePointCloud(str);
}

void CloudViewer::removePC(QString str)
{
    removePC(str.toStdString());
}

void CloudViewer::on_actionClearViewer_triggered()
{
    _visualizer->removeAllCoordinateSystems();
    _visualizer->removeAllPointClouds();
    _visualizer->removeAllShapes();
   this->showReferenceSystemGlobal(true);


    this->update ();

}
