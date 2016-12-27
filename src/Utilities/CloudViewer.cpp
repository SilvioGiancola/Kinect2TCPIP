#include "CloudViewer.h"
#include "ui_CloudViewer.h"

CloudViewer::CloudViewer(QWidget *parent) :
    QVTKWidget(parent),
    ui(new Ui::CloudViewer),
     settings("SilvioGiancola", "CloudViewer")
{
    // UI
    ui->setupUi(this);

    this->setMouseTracking(false);

    _visualizer.reset(new pcl::visualization::PCLVisualizer("Viewer",false));

    this->SetRenderWindow(_visualizer->getRenderWindow());
    this->GetInteractor()->SetInteractorStyle (_visualizer->getInteractorStyle());

    _visualizer->setCameraPosition(-3.5,1,1, // mi posiziono dietro ad un Kinect
                                   0.5,0.5,0.5, // guardo un punto centrale
                                   0,0,1);   // orientato con la z verso l'alto
    _visualizer->setCameraClipDistances(-10,10);

   // setBackGroundColor(QColor(127,127,127));
    setBackGroundColor(settings.value("BackGroundColor").value<QColor>());


    _visualizer->setShowFPS(false);


    // this->setPCReferenceSystemShown(true);
    this->on_actionShowMainRefSyst_triggered(true);

    this->update ();


    // Menu
    _menu = new QMenu(this);
    _menu->addAction(ui->actionClearViewer);
    _menu->addAction(ui->actionSet_Background_Color);
    _menu->addAction(ui->actionShowMainRefSyst);
    _menu->addAction(ui->actionShowPCRefSyst);
}

CloudViewer::~CloudViewer()
{

    settings.setValue("BackGroundColor",_backgroundColor);


    delete _menu;
    delete ui;
}


void CloudViewer::setModel(CloudListModel *MyModel)
{
    myModel = MyModel;
}

// Visualize Menu
void CloudViewer::contextMenuEvent(QContextMenuEvent * event)
{
    _menu->exec(event->globalPos());
}





// Visualization PC
void CloudViewer::showPC(PointCloudT::Ptr PC)
{
    pcl::visualization::PointCloudColorHandlerRGBField<PointT> single_color(PC);
    _visualizer->removePointCloud(PC->header.frame_id);
    _visualizer->addPointCloud<PointT>(PC, single_color, PC->header.frame_id);

    qDebug() << QString::fromStdString(PC->header.frame_id) << " has been added to the viewer";

    if (isPCReferenceSystemShown())
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

void CloudViewer::removePC(PointCloudT::Ptr PC)
{
    _visualizer->removePointCloud(PC->header.frame_id);
}

void CloudViewer::removePC(QString str)
{
    removePC(str.toStdString());
}


void CloudViewer::replacePC(PointCloudT::Ptr PC)
{
    removePC(PC);
    showPC(PC);
}

void CloudViewer::setPointCloudPose(std::string str, Transform T)
{
    //qDebug() << str.c_str();
    _visualizer->updatePointCloudPose(str, T.getAffine3f());
    this->update ();
}
void CloudViewer::setPointCloudPose(QString str, Transform T)
{
    setPointCloudPose(str.toStdString(), T);
}


void CloudViewer::setBackGroundColor(QColor color)
{
    if (color.isValid())
        _backgroundColor = color;

    _visualizer->setBackgroundColor (_backgroundColor.redF(), _backgroundColor.greenF(), _backgroundColor.blueF());

    return;
}

QColor CloudViewer::getBackGroundColor()
{
    return _backgroundColor;
}




// Checkable Actions

bool CloudViewer::isMainReferenceSystemShown() const
{
    return ui->actionShowMainRefSyst->isChecked();
}

void CloudViewer::setMainReferenceSystemShown(bool shown)
{
    ui->actionShowMainRefSyst->setChecked(shown);
}

void CloudViewer::on_actionShowMainRefSyst_triggered(bool value)
{
    if (value)
        _visualizer->addCoordinateSystem(1.0,"Global Reference System");
    else
        _visualizer->removeCoordinateSystem("Global Reference System");
    return;
}





bool CloudViewer::isPCReferenceSystemShown() const
{
    return ui->actionShowPCRefSyst->isChecked();
}

void CloudViewer::setPCReferenceSystemShown(bool shown)
{
    ui->actionShowPCRefSyst->setChecked(shown);
}

void CloudViewer::on_actionShowPCRefSyst_triggered(bool checked)
{
    // nothing to do here, eventually get point cloud ref syst?
}





// Single shot action

void CloudViewer::doClearPointClouds()
{
    _visualizer->removeAllCoordinateSystems();
    _visualizer->removeAllPointClouds();
    _visualizer->removeAllShapes();
    if(isMainReferenceSystemShown())
        on_actionShowMainRefSyst_triggered(true);
    this->update();
}

void CloudViewer::changeBackGroundColor(QColor color)
{
    if (!color.isValid())
        color = QColorDialog::getColor(_backgroundColor, this);

    if (color.isValid())
        setBackGroundColor(color);

    /* if (color == QColor())
    {
        color = QColorDialog::getColor(color, this);
    }*/
}

// Action

void CloudViewer::on_actionClearViewer_triggered()
{
    doClearPointClouds();
}

void CloudViewer::on_actionSet_Background_Color_triggered()
{
    changeBackGroundColor();
}
