#include "ElabWindow.h"
#include "ui_ElabWindow.h"


// Constructor
ElabWindow::ElabWindow(QWidget *parent) : QMainWindow(parent), ui(new Ui::ElabWindow)
{
    // Set the user interface from Qt Designer
    ui->setupUi(this);
    ui->centralWidget->setVisible(false);

    QStringList IDList;
    IDList.append("006670253647");
    IDList.append("006662153647");
    IDList.append("500875340242");
    IDList.append("507040542542");

    ui->IDlistcomboBox->clear();
    ui->IDlistWidget->clear();
    ui->IDlistcomboBox->addItems(IDList);
    ui->IDlistWidget->addItems(IDList);

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

    // get list of files
    QStringList allFiles = QFileDialog::getOpenFileNames(this,
                                                         tr("Open Directory"),
                                                         QDir::homePath() + "/PointClouds/",
                                                         "Point Cloud (*.pcd *.ply)");

    if (allFiles.count() <= 0) return;


    QString dir = QFileInfo(allFiles.at(0)).dir().absolutePath();
    ui->label->setText(dir);
    qDebug() << dir;




    // Get List ID Kinect
    QStringList myIDList;
    for (int i = 0; i < ui->IDlistWidget->count(); i++)
        myIDList.append(ui->IDlistWidget->item(i)->text());
    qDebug() << myIDList;



    myMatrixOfPointCloud.openPointCloud(allFiles, myIDList);


}


void ElabWindow::on_actionSavePointCloudAs_triggered()
{
    // get new directory where to save my list of point clouds
    QString newDir = QFileDialog::getExistingDirectory(this,
                                                       tr("Save File"),
                                                       QDir::homePath() + "/PointClouds/");
    myMatrixOfPointCloud.savePointClouds(newDir);
}

void ElabWindow::on_actionExportReconstruction_triggered()
{
    // get file name where to export the reconstruction
    QString newPath = QFileDialog::getSaveFileName(this,
                                                   tr("Saving Directory"),
                                                   QDir::homePath() + "/PointClouds/");
    myMatrixOfPointCloud.exportReconstruction(newPath);
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

void ElabWindow::on_LignAlignAll_clicked()
{
    for (int i = 0; i < myMatrixOfPointCloud.getNumberOfPointCloudLines() - 1; i++)
    {
        myMatrixOfPointCloud.AlignLines(i, i+1);
        on_actionShowPC_triggered();
    }
}

void ElabWindow::on_BackBoneAlign_clicked()
{
    QString BackBoneID = ui->IDlistcomboBox->currentText();

    myMatrixOfPointCloud.BackBoneAlign(BackBoneID);

}
