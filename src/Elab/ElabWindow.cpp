#include "ElabWindow.h"
#include "ui_ElabWindow.h"


// Constructor
ElabWindow::ElabWindow(QWidget *parent) : QMainWindow(parent), ui(new Ui::ElabWindow)
{
    // Set the user interface from Qt Designer
    ui->setupUi(this);
    ui->centralWidget->setVisible(false);


    // Settings
    readSettings();

}

// Destructor
ElabWindow::~ElabWindow()
{
    writeSettings();

    delete ui;
}


// QSettings
void ElabWindow::writeSettings()
{
    QSettings settings("SilvioGiancola", "Kinect 2 TCPIP");

    settings.beginGroup("Elab settings");

    settings.endGroup();

    qDebug() << "Setting written";
}

void ElabWindow::readSettings()
{
    QSettings settings("SilvioGiancola", "Kinect 2 TCPIP");

    settings.beginGroup("Elab settings");

    settings.endGroup();

    qDebug() << "Setting opened";
}


void ElabWindow::on_actionOpenAcq_triggered()
{

    // get DIR
    QStringList allFiles= QFileDialog::getOpenFileNames(this, tr("Open Directory"),
                                                        QDir::homePath() + "/PointClouds/",
                                                        "Point Cloud (*.pcd *.ply)");


    if (allFiles.count() <= 0) return;



    QString dir = QFileInfo(allFiles.at(0)).dir().absolutePath();
    ui->label->setText(dir);
    qDebug() << dir;




    // Get List ID Kinect
    QStringList myIDList;
    for (int i = 0; i < ui->listWidget->count(); i++)
        myIDList.append(ui->listWidget->item(i)->text());
    qDebug() << myIDList;


    myMatrixOfPointCloud.parsePointClouds(allFiles);


}

void ElabWindow::on_actionShowPC_triggered()
{
    ui->myCloudViewer->doClearPointClouds();
    for (int line = 0; line < myMatrixOfPointCloud.getNumberOfPointCloudLines(); line++)
    {
        for (int index = 0; index < 4; index++)
        {
            ui->myCloudViewer->showPC(myMatrixOfPointCloud.getPointCloud(line, index));
        }
    }
}



void ElabWindow::on_LineAlign_clicked()
{
    myMatrixOfPointCloud.AlignLines(
                ui->lineindex_ref->value(),
                ui->lineindex_new->value());
}

void ElabWindow::on_EstimateNormals_clicked()
{
    myMatrixOfPointCloud.EstimateNormals();
}

void ElabWindow::on_RemoveOutliers_clicked()
{
    myMatrixOfPointCloud.RemoveOutliers();
}
