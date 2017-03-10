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
    ui->IDlistcomboBox->addItem("smallest covariance");
    ui->IDlistcomboBox->addItem("most inliers");
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


void ElabWindow::on_EstimateNormals_clicked()
{
    myMatrixOfPointCloud.EstimateNormals();
}

void ElabWindow::on_RemoveOutliers_clicked()
{
    myMatrixOfPointCloud.RemoveOutliers();
}

void ElabWindow::on_PreOrient_clicked()
{
    myMatrixOfPointCloud.SinglePreOrient(-13.0/180.0*3.14,0,0);
}



void ElabWindow::on_LineAlign_clicked()
{
    if (ui->checkBox_AllLines->isChecked())
    {
        for (int i = 0; i < myMatrixOfPointCloud.getNumberOfPointCloudLines() - 1; i++)
            myMatrixOfPointCloud.AlignLinesICP(i, i+1);
    }
    else
    {
        myMatrixOfPointCloud.AlignLinesICP(
                    ui->lineindex_ref->value(),
                    ui->lineindex_new->value());
    }
    on_actionShowPC_triggered();
}

void ElabWindow::on_LineRANSAC_clicked()
{
    if (ui->checkBox_AllLines->isChecked())
    {
        for (int i = 0; i < myMatrixOfPointCloud.getNumberOfPointCloudLines() - 1; i++)
            myMatrixOfPointCloud.AlignLinesRANSAC(i, i+1);
    }
    else
    {
        myMatrixOfPointCloud.AlignLinesRANSAC(
                    ui->lineindex_ref->value(),
                    ui->lineindex_new->value());
    }
    on_actionShowPC_triggered();
}

void ElabWindow::on_BackBoneAlignICP_clicked()
{

    if (ui->checkBox_AllLines->isChecked())
    {
        for (int i = 0; i < myMatrixOfPointCloud.getNumberOfPointCloudLines() - 1; i++)
            myMatrixOfPointCloud.BackBoneAlignICP(i, i+1,
                                                  ui->IDlistcomboBox->currentText());
    }
    else
    {
        myMatrixOfPointCloud.BackBoneAlignICP(ui->lineindex_ref->value(),
                                              ui->lineindex_new->value(),
                                              ui->IDlistcomboBox->currentText());
    }
    on_actionShowPC_triggered();
}

void ElabWindow::on_BackBoneAlignRANSAC_clicked()
{
    if (ui->checkBox_AllLines->isChecked())
    {
        for (int i = 0; i < myMatrixOfPointCloud.getNumberOfPointCloudLines() - 1; i++)
            myMatrixOfPointCloud.BackBoneAlignRANSAC(i, i+1,
                                                     ui->IDlistcomboBox->currentText());
    }
    else
    {
        myMatrixOfPointCloud.BackBoneAlignRANSAC(ui->lineindex_ref->value(),
                                                 ui->lineindex_new->value(),
                                                 ui->IDlistcomboBox->currentText());
    }
    on_actionShowPC_triggered();
}




void ElabWindow::on_doubleSpinBox_valueChanged(double arg1)
{
    myMatrixOfPointCloud.setMaxDistance(arg1);
}

void ElabWindow::on_ICP_iteration_valueChanged(int arg1)
{
    myMatrixOfPointCloud.setICPIteration(arg1);
}

void ElabWindow::on_doubleSpinBox_2_valueChanged(double arg1)
{
    // Downsampler myDS;
    Downsampler::BRISK_Threshold = arg1;
}

void ElabWindow::on_spinBox_valueChanged(int arg1)
{
    //Downsampler myDS;
    Downsampler::BRISK_Octave = arg1;
}

void ElabWindow::on_doubleSpinBox_3_valueChanged(double arg1)
{
    Downsampler::AGAST_Threshold = arg1;
}

void ElabWindow::on_doubleSpinBox_4_valueChanged(double arg1)
{
    Downsampler::VoxelGrid_LeafSize = arg1;
}

void ElabWindow::on_doubleSpinBox_8_valueChanged(double arg1)
{
    Downsampler::min_scale = arg1;
}

void ElabWindow::on_doubleSpinBox_6_valueChanged(double arg1)
{
    Downsampler::min_contrast = arg1;
}

void ElabWindow::on_spinBox_3_valueChanged(int arg1)
{
    Downsampler::nr_octaves = arg1;
}

void ElabWindow::on_spinBox_4_valueChanged(int arg1)
{
    Downsampler::nr_scales_per_octave = arg1;
}

void ElabWindow::on_doubleSpinBox_7_valueChanged(double arg1)
{
    Matcher::radiusSearchValue = arg1;
}

void ElabWindow::on_spinBox_2_valueChanged(int arg1)
{
    Matcher::kSearchValue = arg1;
}

void ElabWindow::on_radioButton_objectNameChanged(const QString &objectName)
{
    qDebug() << "objectName = " << objectName;

    if (objectName.contains(QString("Radius")))
        Matcher::myDescMethod = RadiusSearch;

    else if (objectName.contains(QString("K")))
        Matcher::myDescMethod = KSearch;
}

void ElabWindow::on_tabWidget_currentChanged(int index)
{
    QString objectName = ui->tabWidget->tabText(index);

    if (objectName.contains(QString("Dist")))
        Matcher::myMethod = Dist;

    else if (objectName.contains(QString("PFHRGB")))
        Matcher::myMethod = PFHRGB;

    else if (objectName.contains(QString("FPFH")))
        Matcher::myMethod = FPFH;

    else if (objectName.contains(QString("PFH")))
        Matcher::myMethod = PFH;

    else if (objectName.contains(QString("CSHOT")))
        Matcher::myMethod = CSHOT;

    else if (objectName.contains(QString("SHOT")))
        Matcher::myMethod = SHOT;

}

void ElabWindow::on_Downsampling_currentChanged(int index)
{
    QString objectName = ui->Downsampling->tabText(index);
    qDebug() << "objectName = " << objectName;

    if (objectName.contains(QString("BRISK")))
        Downsampler::myMethod = BRISK;

    else if (objectName.contains(QString("AGAST")))
        Downsampler::myMethod = AGAST;

    else if (objectName.contains(QString("SIFT")))
        Downsampler::myMethod = SIFT3D;

    else if (objectName.contains(QString("Voxel")))
        Downsampler::myMethod = VOXEL_GRID;

    else if (objectName.contains(QString("Random")))
        Downsampler::myMethod = RANDOM_SAMPLE;
}

void ElabWindow::on_checkBox_clicked(bool checked)
{
    myMatrixOfPointCloud.pt2pl = checked;
}

void ElabWindow::on_pushButton_ApplyT_clicked()
{
    Transform T(ui->doubleSpinBox_Tx->value(),
                ui->doubleSpinBox_Ty->value(),
                ui->doubleSpinBox_Tz->value(),
                ui->doubleSpinBox_Rx->value()/180*3.14,
                ui->doubleSpinBox_Ry->value()/180*3.14,
                ui->doubleSpinBox_Rz->value()/180*3.14);





    Transform refPose = Transform( myMatrixOfPointCloud.getPointCloud(ui->lineindex_new->value(), 0)->sensor_origin_,
                                   myMatrixOfPointCloud.getPointCloud(ui->lineindex_new->value(), 0)->sensor_orientation_);


    for (int l = ui->lineindex_new->value(); l < myMatrixOfPointCloud.getNumberOfPointCloudLines(); l++)
    {
        pcl::ScopeTime t(QString("Transforming line %1 ").arg(l).toStdString().c_str());

        myMatrixOfPointCloud.TransformLine(l, T, refPose);

    }
    on_actionShowPC_triggered();
}
