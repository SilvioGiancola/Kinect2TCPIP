#include "MatrixOfCloud.h"

MatrixOfCloud::MatrixOfCloud()
{

}


MatrixOfCloud::~MatrixOfCloud()
{

}


PointCloudNormalT::Ptr MatrixOfCloud::getLineOfPointCloud(int line)
{
    SetOfCloud mySetOfCloud;
    if (line < _MatrixOfCloud.count() && line > 0)
        mySetOfCloud = _MatrixOfCloud.at(line);
    return mySetOfCloud.getMergedPointCloud();
}

PointCloudNormalT::Ptr MatrixOfCloud::getPointCloud(int line, int index)
{
    SetOfCloud mySetOfCloud;
    if (line < _MatrixOfCloud.count() && line > 0)
        mySetOfCloud = _MatrixOfCloud.at(line);
    return mySetOfCloud.getPointCloud(index);
}

void MatrixOfCloud::setPointCloud(int line, int index, PointCloudNormalT::Ptr PC)
{
    SetOfCloud mySetOfCloud;
    if (line < _MatrixOfCloud.count() && line > 0)
        mySetOfCloud = _MatrixOfCloud.at(line);
    mySetOfCloud.setPointCloud(index, PC);
}

PointCloudNormalT::Ptr MatrixOfCloud::getCompletePointCloud()
{
    PointCloudNormalT::Ptr CompletePC ( new PointCloudNormalT);
    for (int i = 0; i < _MatrixOfCloud.count(); i++)
    {
        PointCloudNormalT::Ptr temp(new PointCloudNormalT());

        Eigen::Matrix4f trans = Eigen::Matrix4f::Identity();
        trans.block(0,0,3,3) = getLineOfPointCloud(i)->sensor_orientation_.matrix();
        trans.block(0,3,4,1) = getLineOfPointCloud(i)->sensor_origin_;

        pcl::transformPointCloudWithNormals(*getLineOfPointCloud(i), *temp, trans);

        *CompletePC += *temp;

    }

    return CompletePC;
}



int MatrixOfCloud::getNumberOfPointCloudLines()
{
    return _MatrixOfCloud.count();
}




void MatrixOfCloud::parsePointClouds(QStringList allFiles)
{

    // fill matrix of PC
    _MatrixOfCloud.clear();

    for (int line = 0; line < allFiles.count()/4; line++)
    {
        SetOfCloud newlineofcloud;
        for (int i = 0; i < 4; i++)
        {

            qDebug() << allFiles.at(line*4 + i);


            PointCloudNormalT::Ptr PC(new PointCloudNormalT);
            pcl::io::loadPCDFile(allFiles.at(line*4 + i).toStdString(), *PC);

            PC->header.frame_id = allFiles.at(line*4 + i).toStdString();
            newlineofcloud.setPointCloud(i,PC);

            //Find set of line of cloud
        }
        _MatrixOfCloud.append(newlineofcloud);
        qDebug() << "matrix of cloud is now " << _MatrixOfCloud.length() << " long";

    }
}

void MatrixOfCloud::EstimateNormals()
{
    for (int line = 0; line < _MatrixOfCloud.count(); line++)
    {
        SetOfCloud lineofcloud = _MatrixOfCloud.at(line);
        for (int i = 0; i < 4; i++)
        {
            // ui->myCloudViewer->showPC(lineofcloud.getPointCloud(i));
            PointCloudNormalT::Ptr input(new PointCloudNormalT);
            input = lineofcloud.getPointCloud(i);


            pcl::IntegralImageNormalEstimation<PointNormalT, PointNormalT> ne;
            ne.setNormalEstimationMethod (ne.AVERAGE_3D_GRADIENT);
            ne.setMaxDepthChangeFactor(0.02);
            ne.setNormalSmoothingSize(10.0);

            ne.setInputCloud(input);

            ne.compute(*input);
        }
    }
}


void MatrixOfCloud::AlignLines(int refindex, int newindex)
{
    SetOfCloud refline = _MatrixOfCloud.at(refindex);
    SetOfCloud newline = _MatrixOfCloud.at(newindex);


    PointCloudNormalT::Ptr ref4cloud(new PointCloudNormalT);
    ref4cloud = refline.getMergedPointCloud();
    qDebug() << "ref4cloud: " << ref4cloud->size();


    PointCloudNormalT::Ptr new4cloud(new PointCloudNormalT);
    new4cloud = newline.getMergedPointCloud();
    qDebug() << "new4cloud: " << new4cloud->size();

    /*
    pcl::VoxelGrid<PointT> sor;
    sor.setLeafSize (0.01f, 0.01f, 0.01f);
    sor.setInputCloud (ref4cloud);
    sor.filter (*ref4cloud);
    sor.setInputCloud (new4cloud);
    sor.filter (*new4cloud);
*/


    float decimate_percent =  0.5;
    pcl::RandomSample<PointNormalT> random_sampler;
    random_sampler.setKeepOrganized(false);

    random_sampler.setInputCloud(ref4cloud);
    random_sampler.setSample((int) (decimate_percent*ref4cloud->points.size()));
    random_sampler.filter(*ref4cloud);

    random_sampler.setInputCloud(new4cloud);
    random_sampler.setSample((int) (decimate_percent*new4cloud->points.size()));
    random_sampler.filter(*new4cloud);


    qDebug() << "ref4cloud: " << ref4cloud->size();
    qDebug() << "new4cloud: " << new4cloud->size();





    /* // PointCloudT::Ptr ref4cloud(new PointCloudT);
   // PointCloudT::Ptr new4cloud(new PointCloudT);
    pcl::transformPointCloudWithNormals(*ref4cloud, *ref4cloud, ref4cloud->sensor_origin_.head(3), ref4cloud->sensor_orientation_);
    pcl::transformPointCloudWithNormals(*new4cloud, *new4cloud, new4cloud->sensor_origin_.head(3), new4cloud->sensor_orientation_);
*/

    qDebug() << "PC transformed";



    // Find Correspondences
    pcl::registration::CorrespondenceEstimation<PointNormalT, PointNormalT> myCorrespondenceEstimation;
    pcl::CorrespondencesPtr correspondences(new pcl::Correspondences);
    //myCorrespondenceEstimation.reset(new pcl::registration::CorrespondenceEstimation<PointNormalT, PointNormalT>);
    myCorrespondenceEstimation.setInputSource (new4cloud);
    myCorrespondenceEstimation.setInputTarget (ref4cloud);
    qDebug() << "corresp started";
    myCorrespondenceEstimation.determineReciprocalCorrespondences (*correspondences);


    qDebug() << "corresp found";

    // Filter Correspondences
    pcl::registration::CorrespondenceRejectorDistance::Ptr cor_rej_dist (new pcl::registration::CorrespondenceRejectorDistance);
    cor_rej_dist->setMaximumDistance(0.2);

    if (cor_rej_dist->requiresSourcePoints())
    {
        pcl::PCLPointCloud2::Ptr Source2 (new pcl::PCLPointCloud2);
        pcl::toPCLPointCloud2 (*new4cloud, *Source2);
        cor_rej_dist->setSourcePoints(Source2);
    }

    if (cor_rej_dist->requiresTargetPoints())
    {
        pcl::PCLPointCloud2::Ptr Target2 (new pcl::PCLPointCloud2);
        pcl::toPCLPointCloud2 (*ref4cloud, *Target2);
        cor_rej_dist->setTargetPoints(Target2);
    }

    cor_rej_dist->setInputCorrespondences (correspondences);
    cor_rej_dist->getCorrespondences (*correspondences);

    qDebug() << "corresp filtered";


    // Find Transformation
    pcl::registration::TransformationEstimationPointToPlaneLLS<PointNormalT, PointNormalT> transf_est;
    Eigen::Matrix4f transformation_ = Eigen::Matrix4f::Identity();
    transf_est.estimateRigidTransformation (*new4cloud, *ref4cloud, *correspondences, transformation_);

    qDebug() << "transf est.";

    /*
   PointCloudNormalT::Ptr ref4cloud(new PointCloudNormalT);
   ref4cloud = refline.getMergedPointCloud();
   qDebug() << "ref4cloud: " << ref4cloud->size();

*/
    /* PointCloudNormalT::Ptr new4cloud(new PointCloudNormalT);
   */
    new4cloud = newline.getMergedPointCloud();

    /* qDebug() << "new4cloud: " << new4cloud->size();
   */


    pcl::transformPointCloudWithNormals (*new4cloud, *new4cloud, transformation_);




}

void MatrixOfCloud::RemoveOutliers()
{
    for (int line = 0; line < _MatrixOfCloud.count(); line++)
    {
        SetOfCloud lineofcloud = _MatrixOfCloud.at(line);
        for (int i = 0; i < 4; i++)
        {
            // ui->myCloudViewer->showPC(lineofcloud.getPointCloud(i));
            PointCloudNormalT::Ptr input(new PointCloudNormalT);
            input = lineofcloud.getPointCloud(i);

            pcl::RadiusOutlierRemoval<PointNormalT> outrem;
            // build the filter
            outrem.setInputCloud(input);
            outrem.setRadiusSearch(0.05);
            outrem.setMinNeighborsInRadius (10);
            outrem.setNegative(false);
            outrem.setKeepOrganized(input->isOrganized());
            // apply filter
            outrem.filter (*input);
        }
    }
}
