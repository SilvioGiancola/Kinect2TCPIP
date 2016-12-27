#include "ViewerWindow.h"
#include "ui_ViewerWindow.h"


// Constructor
ViewerWindow::ViewerWindow(QWidget *parent) : QMainWindow(parent), ui(new Ui::ViewerWindow)
{
    // Set the user interface from Qt Designer
    ui->setupUi(this);
    ui->centralWidget->setVisible(false);



    // Default Value for Settings
    defaultSettings();

    // Settings
    readSettings();



    myCloudList = new CloudListModel();
    ui->myCloudList->setModel(myCloudList);
    ui->myCloudViewer->setModel(myCloudList);

}

// Destructor
ViewerWindow::~ViewerWindow()
{
    //Settings
    writeSettings();
    delete ui;
}





// QSettings
void ViewerWindow::defaultSettings()
{
    DefaultDir = QString("%1/PointClouds").arg(QDir::homePath());
}

void ViewerWindow::writeSettings()
{
    QSettings settings("SilvioGiancola", "Viewer");

    settings.beginGroup("Viewer settings");


    qDebug() << "Writting the Settings :";

    settings.setValue("DefaultDir", DefaultDir);
    qDebug() << " - DefaultDir :" << DefaultDir;


    settings.endGroup();

    qDebug() << "Setting written";
}

void ViewerWindow::readSettings()
{
    // Settings Name
    QSettings settings("SilvioGiancola", "Viewer");

    // Settings Group
    settings.beginGroup("Viewer settings");


    qDebug() << "Reading the Settings :";

    DefaultDir = settings.value(QString("DefaultDir"),DefaultDir).toString();
    qDebug() << " - DefaultDir :" << DefaultDir;


    // End of Settins Group
    settings.endGroup();

    qDebug() << "Setting read";

}






void ViewerWindow::on_actionOpen_Acquisition_triggered()
{

    QStringList Files = QFileDialog::getOpenFileNames(this, tr("Open file..."), DefaultDir);//, tr("All Files (*.pcd;*.ply)"));


    foreach (QString file, Files)
    {
        PointCloudT::Ptr cloud (new PointCloudT);

        if (file.endsWith(".pcd"))
            pcl::io::loadPCDFile(file.toStdString(), *cloud);

        else if (file.endsWith(".ply"))
            pcl::io::loadPLYFile(file.toStdString(), *cloud);

        else
            return;

        qDebug() << file << " has been opened";

        cloud->header.frame_id = file.toStdString();


        ui->myCloudViewer->showPC(cloud);
     //   ui->myCloudList->addCloud(cloud);
        myCloudList->addCloud(cloud);
    }
    ui->statusBar->showMessage(QString("%1 Point Clouds has been opened successfully").arg(Files.size()));

}
