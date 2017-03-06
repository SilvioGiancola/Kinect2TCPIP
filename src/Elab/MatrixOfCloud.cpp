#include "MatrixOfCloud.h"

MatrixOfCloud::MatrixOfCloud()
{

}


MatrixOfCloud::~MatrixOfCloud()
{

}



// POINT CLOUDS handling functions
PointCloudNormalT::Ptr MatrixOfCloud::getLineOfPointCloud(int line)
{
    SetOfCloud mySetOfCloud;
    if (line < _MatrixOfCloud.count() && line >= 0)
        mySetOfCloud = _MatrixOfCloud.at(line);
    return mySetOfCloud.getMergedPointCloud();
}

PointCloudNormalT::Ptr MatrixOfCloud::getPointCloud(int line, int index)
{
    SetOfCloud mySetOfCloud;
    if (line < _MatrixOfCloud.count() && line >= 0)
        mySetOfCloud = _MatrixOfCloud.at(line);
    return mySetOfCloud.getPointCloud(index);
}

PointCloudNormalT::Ptr MatrixOfCloud::getPointCloud(int line, QString ID)
{
    SetOfCloud mySetOfCloud;
    if (line < _MatrixOfCloud.count() && line >= 0)
        mySetOfCloud = _MatrixOfCloud.at(line);
    return mySetOfCloud.getPointCloud(ID);
}

void MatrixOfCloud::setPointCloud(int line, int index, PointCloudNormalT::Ptr PC)
{
    SetOfCloud mySetOfCloud;
    if (line < _MatrixOfCloud.count() && line >= 0)
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



// OPEN, SAVE and EXPORT functions
void MatrixOfCloud::openPointCloud(QStringList allFiles, QStringList IDlist)
{

    // fill matrix of PC
    _MatrixOfCloud.clear();

    for (int line = 0; line < allFiles.count()/4; line++)
    {
        qDebug() << "line: " << line;
        SetOfCloud newlineofcloud;
        newlineofcloud.setIDlist(IDlist);

        QStringList lineFiles;
        for (int i = 0; i < 4; i++)
            lineFiles.append(allFiles.at(line*4 + i));

        //  int i = 0;
        QString ID;
        foreach (ID, IDlist)
        {
            QString file;
            foreach (file, lineFiles)
            {
                if (file.contains(ID))
                {
                    qDebug() << file << " -> ID: " << ID;

                    PointCloudNormalT::Ptr PC(new PointCloudNormalT);
                    pcl::io::loadPCDFile(file.toStdString(), *PC);

                    PC->header.frame_id = file.toStdString();
                    newlineofcloud.setPointCloud(ID,PC);
                    // i++;
                    continue;
                }
            }
        }

        _MatrixOfCloud.append(newlineofcloud);
        qDebug() << "matrix of cloud is now " << _MatrixOfCloud.length() << " long";

    }
}

void MatrixOfCloud::savePointClouds(QString newDir)
{
    for (int line = 0; line < _MatrixOfCloud.count(); line++)
    {
        qDebug() << "\n-> line: " << line;
        for (int i = 0; i < 4; i++)
        {
            qDebug() << "  -> index: " << i;
            PointCloudNormalT::Ptr PC(new PointCloudNormalT);
            PC = this->getPointCloud(line, i);

            QString Path = QString::fromStdString(PC->header.frame_id);
            qDebug() << Path;

            Path = newDir + "/" + Path.section("/",-1,-1);
            qDebug() << "PC should be saved in : " << Path;
            pcl::io::savePCDFileBinary(Path.toStdString(),*PC);
        }
    }
}

void MatrixOfCloud::exportReconstruction(QString Path)
{
    PointCloudNormalT::Ptr PC(new PointCloudNormalT);
    PC = this->getCompletePointCloud();

    qDebug() << "Reconstruction should be saved in : " << Path;
    pcl::io::savePCDFileBinary(Path.toStdString(),*PC);

}




// ELABORATION functions
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

void MatrixOfCloud::BackBoneAlign(QString ID)
{
    for (int i = 0; i < _MatrixOfCloud.length() - 1; i++)
    {
        for (int iter = 0; iter < 1; iter++)
        {
            PointCloudNormalT::Ptr PC_Target(new PointCloudNormalT);
            PC_Target = this->getPointCloud(i,ID);
            qDebug() << "PC_Target: " << PC_Target->size();
            qDebug() << "PC_Target: " << QString::fromStdString(PC_Target->header.frame_id);

            PointCloudNormalT::Ptr PC_Input(new PointCloudNormalT);
            PC_Input = this->getPointCloud(i+1,ID);
            qDebug() << "PC_Input: " << PC_Input->size();
            qDebug() << "PC_Input: " << QString::fromStdString(PC_Input->header.frame_id);


            float decimate_percent = 1.0;
            if (iter < 10)  decimate_percent = 0.0625;
            else if (iter < 15)  decimate_percent = 0.25;
            else if (iter < 18)  decimate_percent = 0.5;
            else if (iter < 20)  decimate_percent = 1.0;

            float corr_max_distance = 0.5;
            if (iter < 10)  corr_max_distance = (0.5);
            else if (iter < 15)  corr_max_distance = 0.2;
            else if (iter < 18)  corr_max_distance = (0.1);
            else if (iter < 20)  corr_max_distance = (0.05);



            PointCloudNormalT::Ptr PC_Target_copy(new PointCloudNormalT);
            PointCloudNormalT::Ptr PC_Input_copy(new PointCloudNormalT);
            pcl::copyPointCloud(*PC_Target, *PC_Target_copy);
            pcl::copyPointCloud(*PC_Input, *PC_Input_copy);
            /*

            //DO BRISK STUFF
            {
                pcl::ScopeTime t ("Detect BRISK");
                // constructor
                pcl::BriskKeypoint2D<PointNormalT> brisk_keypoint_estimation;

                //output
                pcl::PointCloud<pcl::PointWithScale> brisk_keypoints_2D;

                brisk_keypoint_estimation.setThreshold(10);
                brisk_keypoint_estimation.setOctaves(2);
                brisk_keypoint_estimation.setInputCloud (PC_Input);

                //compute
                brisk_keypoint_estimation.compute (brisk_keypoints_2D);

                //convert pointwithscale to 3D
                PC_Input_copy->resize(brisk_keypoints_2D.size());

                int k = brisk_keypoints_2D.size();
                for(int i = 0, j = 0; i < k; ++i)
                {
                    /// TO DO: improve accuracy
                    int u = floor(brisk_keypoints_2D.points[i].x + 0.5);
                    int v = floor(brisk_keypoints_2D.points[i].y + 0.5);
                    j = u + v * PC_Input->width;
                    if(isnan(PC_Input->points[j].x)) --k;
                    else PC_Input_copy->points[i]=PC_Input->points[j];
                }

                std::vector<PointNormalT,Eigen::aligned_allocator<PointNormalT> >::iterator    keypointIt=PC_Input_copy->begin();

                for(size_t i=k; k<brisk_keypoints_2D.size(); ++k)
                    PC_Input_copy->erase(keypointIt+i);
            }

            {
                pcl::ScopeTime t ("Detect BRISK");
                // constructor
                pcl::BriskKeypoint2D<PointNormalT> brisk_keypoint_estimation;

                //output
                pcl::PointCloud<pcl::PointWithScale> brisk_keypoints_2D;

                brisk_keypoint_estimation.setThreshold(10);
                brisk_keypoint_estimation.setOctaves(2);
                brisk_keypoint_estimation.setInputCloud (PC_Target);

                //compute
                brisk_keypoint_estimation.compute (brisk_keypoints_2D);

                //convert pointwithscale to 3D
                PC_Target_copy->resize(brisk_keypoints_2D.size());

                int k = brisk_keypoints_2D.size();
                for(int i = 0, j = 0; i < k; ++i)
                {
                    /// TO DO: improve accuracy
                    int u = floor(brisk_keypoints_2D.points[i].x + 0.5);
                    int v = floor(brisk_keypoints_2D.points[i].y + 0.5);
                    j = u + v * PC_Target->width;
                    if(isnan(PC_Target->points[j].x)) --k;
                    else PC_Target_copy->points[i]=PC_Target->points[j];
                }

                std::vector<PointNormalT,Eigen::aligned_allocator<PointNormalT> >::iterator    keypointIt=PC_Target_copy->begin();

                for(size_t i=k; k<brisk_keypoints_2D.size(); ++k)
                    PC_Target_copy->erase(keypointIt+i);
            }*/

            // perform alignment
            //Transform T = AlignICP(PC_Target_copy, PC_Input_copy, decimate_percent, corr_max_distance);
            Transform T = AlignRANSAC(PC_Target_copy, PC_Input_copy);

            for (int l = i+1; l < this->getNumberOfPointCloudLines(); l++)
            {
                for (int j = 0; j < 4; j++)
                {
                    Transform currentPose = Transform( this->getPointCloud(l, j)->sensor_origin_,
                                                       this->getPointCloud(l, j)->sensor_orientation_);
                    currentPose = T.postmultiplyby(currentPose);
                    // currentPose.print();

                    this->getPointCloud(l, j)->sensor_origin_ = currentPose.getOrigin4();
                    this->getPointCloud(l, j)->sensor_orientation_ = currentPose.getQuaternion();

                    // this->getPointCloud(l, j)->sensor_origin_ = T.block<4,1>(0,3)+ this->getPointCloud(l, j)->sensor_origin_;
                    // this->getPointCloud(l, j)->sensor_orientation_ = Eigen::Quaternionf(T.block<3,3>(0,0)) * this->getPointCloud(l, j)->sensor_orientation_;
                }
            }

        }
    }
}

void MatrixOfCloud::AlignLines(int refindex, int newindex)
{
    SetOfCloud refline = _MatrixOfCloud.at(refindex);
    SetOfCloud newline = _MatrixOfCloud.at(newindex);



    for (int iter = 0; iter < 20; iter++)
    {
        PointCloudNormalT::Ptr PC_Target(new PointCloudNormalT);
        PC_Target = refline.getMergedPointCloud();
        qDebug() << "PC_Target: " << PC_Target->size();


        PointCloudNormalT::Ptr PC_Input(new PointCloudNormalT);
        PC_Input = newline.getMergedPointCloud();
        qDebug() << "PC_Input: " << PC_Input->size();


        float decimate_percent = 1.0;
        if (iter < 10)  decimate_percent = 0.0625;
        else if (iter < 15)  decimate_percent = 0.25;
        else if (iter < 18)  decimate_percent = 0.5;
        else if (iter < 20)  decimate_percent = 0.9;

        float corr_max_distance = 0.2;
        if (iter < 10)  corr_max_distance = (0.5);
        else if (iter < 15)  corr_max_distance = 0.2;
        else if (iter < 18)  corr_max_distance = (0.1);
        else if (iter < 20)  corr_max_distance = (0.05);

        // perform alignment
        Transform T = AlignICP(PC_Target, PC_Input, decimate_percent, corr_max_distance);



        for (int l = newindex; l < this->getNumberOfPointCloudLines(); l++)
        {
            for (int i = 0; i < 4; i++)
            {
                Transform currentPose = Transform( this->getPointCloud(l, i)->sensor_origin_,
                                                   this->getPointCloud(l, i)->sensor_orientation_);
                currentPose = T.postmultiplyby(currentPose);
                // currentPose.print();

                this->getPointCloud(l, i)->sensor_origin_ = currentPose.getOrigin4();
                this->getPointCloud(l, i)->sensor_orientation_ = currentPose.getQuaternion();
            }
        }
    }

    //  PC_Input = newline.getMergedPointCloud();


    //  pcl::transformPointCloudWithNormals (*PC_Input, *PC_Input, transformation_);


    /*
    Transform T =  utils::getTransformationNormal(PC_target, PC_input);


    PointCloudT::Ptr PCnew = ui->myKinectWidget2->getPointCloud();
    Transform currentPose = Transform(PCnew->sensor_origin_, PCnew->sensor_orientation_);


    T = T.postmultiplyby(currentPose);
    T.print();
*/

}



Transform MatrixOfCloud::AlignICP(PointCloudNormalT::Ptr PC_Target, PointCloudNormalT::Ptr PC_Input, float decimate_percent, float corr_max_distance)
{
    // PRE - TRANSFORM
    pcl::transformPointCloudWithNormals(*PC_Target, *PC_Target, PC_Target->sensor_origin_.head(3), PC_Target->sensor_orientation_);
    pcl::transformPointCloudWithNormals(*PC_Input, *PC_Input, PC_Input->sensor_origin_.head(3), PC_Input->sensor_orientation_);

    qDebug() << "PC_Target: " << PC_Target->size();
    qDebug() << "PC_Input: " << PC_Input->size();


    pcl::RandomSample<PointNormalT> random_sampler;
    random_sampler.setKeepOrganized(false);

    random_sampler.setInputCloud(PC_Target);
    random_sampler.setSample((int) (decimate_percent*PC_Target->points.size()));
    random_sampler.filter(*PC_Target);

    random_sampler.setInputCloud(PC_Input);
    random_sampler.setSample((int) (decimate_percent*PC_Input->points.size()));
    random_sampler.filter(*PC_Input);


    qDebug() << "PC_Target_decim: " << PC_Target->size();
    qDebug() << "PC_Input_decim: " << PC_Input->size();


    // remove NAN
    std::vector<int> ind;
    pcl::removeNaNFromPointCloud(*PC_Target, *PC_Target, ind);
    pcl::removeNaNNormalsFromPointCloud(*PC_Target, *PC_Target, ind);
    pcl::removeNaNFromPointCloud(*PC_Input, *PC_Input, ind);
    pcl::removeNaNNormalsFromPointCloud(*PC_Input, *PC_Input, ind);









    /*
pcl::IterativeClosestPoint<PointNormalT, PointNormalT> icp;


// Corrispondence Estimation
// Scelgo il mia stima dei accopiamenti
qDebug() << "Corrispondence Estimation";
pcl::registration::CorrespondenceEstimationBase<PointNormalT, PointNormalT>::Ptr cens;
cens.reset(new pcl::registration::CorrespondenceEstimation<PointNormalT, PointNormalT>);
cens->setInputTarget (PC_Target);
cens->setInputSource (PC_Input);
icp.setCorrespondenceEstimation (cens);


// Corrispondence Rejection
qDebug() << "Corrispondence Rejection";
pcl::registration::CorrespondenceRejectorDistance::Ptr cor_rej_dist (new pcl::registration::CorrespondenceRejectorDistance);
cor_rej_dist->setMaximumDistance(0.2);
cor_rej_dist->setInputTarget<PointNormalT> (PC_Target);
cor_rej_dist->setInputSource<PointNormalT> (PC_Input);
icp.addCorrespondenceRejector (cor_rej_dist);


// Transformation Estimation
// Scelgo un metodo per risolvere il problema
qDebug() << "Transformation Estimation";
pcl::registration::TransformationEstimation<PointNormalT, PointNormalT>::Ptr te;
te.reset(new pcl::registration::TransformationEstimationPointToPlaneLLS<PointNormalT, PointNormalT>);
icp.setTransformationEstimation (te);


icp.setInputSource(PC_Input);
icp.setInputTarget(PC_Target);

// Modalità di fine ICP
// icp.setEuclideanFitnessEpsilon(10E-9);
//  icp.setTransformationEpsilon(10E-9);
icp.setMaximumIterations(1);


qDebug() << "Align";
PointCloudNormalT::Ptr Final(new PointCloudNormalT);
icp.align(*Final);
qDebug() << "has converged:" << icp.hasConverged() << " score: " << icp.getFitnessScore();


Eigen::Matrix4f transformation_ = icp.getFinalTransformation();
*/









    pcl::CorrespondencesPtr correspondences(new pcl::Correspondences);

    // Find Correspondences
    {
        pcl::ScopeTime t ("Find Correspondences");
        pcl::registration::CorrespondenceEstimationBase<PointNormalT, PointNormalT>::Ptr myCorrespondenceEstimation;
        myCorrespondenceEstimation.reset(new pcl::registration::CorrespondenceEstimation<PointNormalT, PointNormalT>);
        myCorrespondenceEstimation->setInputSource (PC_Input);
        myCorrespondenceEstimation->setInputTarget (PC_Target);
        // myCorrespondenceEstimation->determineReciprocalCorrespondences (*correspondences);
        myCorrespondenceEstimation->determineCorrespondences(*correspondences);
        qDebug() << "corresp found : " << correspondences->size() ;
    }

    // Filter Correspondences
    {
        pcl::ScopeTime t ("Reject Correspondences");
        pcl::registration::CorrespondenceRejectorDistance::Ptr cor_rej_dist (new pcl::registration::CorrespondenceRejectorDistance);
        cor_rej_dist->setMaximumDistance(corr_max_distance);
        if (cor_rej_dist->requiresSourcePoints())
        {
            pcl::PCLPointCloud2::Ptr Source2 (new pcl::PCLPointCloud2);
            pcl::toPCLPointCloud2 (*PC_Input, *Source2);
            cor_rej_dist->setSourcePoints(Source2);
        }
        if (cor_rej_dist->requiresTargetPoints())
        {
            pcl::PCLPointCloud2::Ptr Target2 (new pcl::PCLPointCloud2);
            pcl::toPCLPointCloud2 (*PC_Target, *Target2);
            cor_rej_dist->setTargetPoints(Target2);
        }
        cor_rej_dist->setInputCorrespondences (correspondences);
        cor_rej_dist->getCorrespondences (*correspondences);
        qDebug() << "corresp filtered : " << correspondences->size();
    }


    //   if (correspondences->size() < 10) return Transform();
    // Find Transformation
    pcl::ScopeTime t ("Find Transformation");
    Eigen::Matrix4f transformation_ = Eigen::Matrix4f::Identity();
    pcl::registration::TransformationEstimationPointToPlaneLLS<PointNormalT, PointNormalT> transf_est;
    transf_est.estimateRigidTransformation (*PC_Input, *PC_Target, *correspondences, transformation_);
    Transform T(transformation_);
    qDebug() << "transf est : " << T.prettyprint();

    return T;
}


#include <pcl/filters/conditional_removal.h>
Transform MatrixOfCloud::AlignRANSAC(PointCloudNormalT::Ptr PC_Target, PointCloudNormalT::Ptr PC_Input)
{


    pcl::transformPointCloudWithNormals(*PC_Target, *PC_Target, PC_Target->sensor_origin_.head(3), PC_Target->sensor_orientation_);
    // pcl::transformPointCloudWithNormals(*PC_Target_BRISK, *PC_Target_BRISK, PC_Target_BRISK->sensor_origin_.head(3), PC_Target_BRISK->sensor_orientation_);

    pcl::transformPointCloudWithNormals(*PC_Input, *PC_Input, PC_Input->sensor_origin_.head(3), PC_Input->sensor_orientation_);
    // pcl::transformPointCloudWithNormals(*PC_Input_BRISK, *PC_Input_BRISK, PC_Input_BRISK->sensor_origin_.head(3), PC_Input_BRISK->sensor_orientation_);



    PointCloudNormalT::Ptr PC_Target_BRISK(new PointCloudNormalT);
    DetectBRISK(PC_Target,PC_Target_BRISK,15,4);

    PointCloudNormalT::Ptr PC_Input_BRISK(new PointCloudNormalT);
    DetectBRISK(PC_Input,PC_Input_BRISK,15,4);




    // remove 0 point
    pcl::ConditionAnd<PointNormalT>::Ptr range_cond (new pcl::ConditionAnd<PointNormalT> ());
    range_cond->addComparison (pcl::FieldComparison<PointNormalT>::ConstPtr (new pcl::FieldComparison<PointNormalT> ("z", pcl::ComparisonOps::GT, 0.1)));

    // build the filter
    pcl::ConditionalRemoval<PointNormalT> condrem(true);
    condrem.setCondition(range_cond);

    condrem.setInputCloud (PC_Target_BRISK);
    condrem.setKeepOrganized(PC_Target_BRISK->isOrganized());
    condrem.filter (*PC_Target_BRISK);

    condrem.setInputCloud (PC_Input_BRISK);
    condrem.setKeepOrganized(PC_Input_BRISK->isOrganized());
    condrem.filter (*PC_Input_BRISK);




    std::vector<int> ind;
    //  PointCloudT::Ptr Model_nonan (new PointCloudT);
    // PointCloudT::Ptr PC_nonan (new PointCloudT);
    pcl::removeNaNFromPointCloud(*PC_Target, *PC_Target, ind);
    pcl::removeNaNNormalsFromPointCloud(*PC_Target, *PC_Target, ind);
    pcl::removeNaNFromPointCloud(*PC_Input, *PC_Input, ind);
    pcl::removeNaNNormalsFromPointCloud(*PC_Input, *PC_Input, ind);

    pcl::PointCloud<pcl::SHOT1344>::Ptr PC_Target_desc (new pcl::PointCloud<pcl::SHOT1344>);
    DescribeCSHOT(PC_Target, PC_Target_BRISK, PC_Target_desc);

    pcl::PointCloud<pcl::SHOT1344>::Ptr PC_Input_desc (new pcl::PointCloud<pcl::SHOT1344>);
    DescribeCSHOT(PC_Input, PC_Input_BRISK, PC_Input_desc);



    pcl::CorrespondencesPtr correspondences(new pcl::Correspondences);
    // Filter Correspondences
    {
        pcl::ScopeTime t ("Estimate Correspondences");
        pcl::registration::CorrespondenceEstimationBase<pcl::SHOT1344, pcl::SHOT1344>::Ptr myCorrespondenceEstimation;
        myCorrespondenceEstimation.reset(new pcl::registration::CorrespondenceEstimation<pcl::SHOT1344, pcl::SHOT1344>);

        myCorrespondenceEstimation->setInputSource (PC_Input_desc);
        myCorrespondenceEstimation->setInputTarget (PC_Target_desc);
        //if (ui->myCorrespondenceEstimation->_reciprok)
        myCorrespondenceEstimation->determineReciprocalCorrespondences (*correspondences);
        // else
        //   myCorrespondenceEstimation->determineCorrespondences (*correspondences);

        qDebug() << "correspondence size : " << correspondences->size();
    }


/*
    // Filter Correspondences
    {
        pcl::ScopeTime t ("Reject Correspondences");
        pcl::registration::CorrespondenceRejectorDistance::Ptr cor_rej_dist (new pcl::registration::CorrespondenceRejectorDistance);
        cor_rej_dist->setMaximumDistance(0.5);
        if (cor_rej_dist->requiresSourcePoints())
        {
            pcl::PCLPointCloud2::Ptr Source2 (new pcl::PCLPointCloud2);
            pcl::toPCLPointCloud2 (*PC_Input_BRISK, *Source2);
            cor_rej_dist->setSourcePoints(Source2);
        }
        if (cor_rej_dist->requiresTargetPoints())
        {
            pcl::PCLPointCloud2::Ptr Target2 (new pcl::PCLPointCloud2);
            pcl::toPCLPointCloud2 (*PC_Target_BRISK, *Target2);
            cor_rej_dist->setTargetPoints(Target2);
        }
        cor_rej_dist->setInputCorrespondences (correspondences);
        cor_rej_dist->getCorrespondences (*correspondences);
        qDebug() << "corresp filtered : " << correspondences->size();
    }
*/
    if (correspondences->size() < 4)
        return Transform();



    std::vector<int> source_indices(correspondences->size());
    std::vector<int> target_indices(correspondences->size());
    for (int i = 0; i < (int)correspondences->size(); ++i)
    {
        source_indices[i] = correspondences->at(i).index_query;
        target_indices[i] = correspondences->at(i).index_match;
    }
    double inlierThreshold = 0.1;

    pcl::SampleConsensusModelRegistration<PointNormalT>::Ptr model;
    std::vector<int> inliers;
    Eigen::VectorXf model_coefficients;
    Eigen::Matrix4f Transformation;

    {
        pcl::ScopeTime t("ransac-main");

        model.reset(new pcl::SampleConsensusModelRegistration<PointNormalT>(PC_Input_BRISK, source_indices ));
        model->setInputTarget(PC_Target_BRISK, target_indices);


        pcl::RandomSampleConsensus<PointNormalT> sac(model, inlierThreshold);


        sac.setMaxIterations(10000000);
        sac.setProbability(0.99);
        sac.computeModel();


        sac.getInliers(inliers);
        sac.getModelCoefficients (model_coefficients);
    }


    Transformation.row (0) = model_coefficients.segment<4>(0);
    Transformation.row (1) = model_coefficients.segment<4>(4);
    Transformation.row (2) = model_coefficients.segment<4>(8);
    Transformation.row (3) = model_coefficients.segment<4>(12);



    std::cout << Transformation << std::endl;



    return Transform(Transformation);
}


#include <pcl/keypoints/brisk_2d.h>

void MatrixOfCloud::DetectBRISK(PointCloudNormalT::Ptr input, PointCloudNormalT::Ptr output, int paramThreshold, int octave)
{
    std::cout << "BRISK Detector...";

    QTime t;
    t.start();

    // constructor
    pcl::BriskKeypoint2D<PointNormalT> brisk_keypoint_estimation;

    //output
    pcl::PointCloud<pcl::PointWithScale> brisk_keypoints_2D;

    //parameters
    brisk_keypoint_estimation.setThreshold(paramThreshold);
    brisk_keypoint_estimation.setOctaves(octave);
    brisk_keypoint_estimation.setInputCloud (input);

    //compute
    brisk_keypoint_estimation.compute (brisk_keypoints_2D);

    //convert pointwithscale to 3D
    output->resize(brisk_keypoints_2D.size());

    int k = brisk_keypoints_2D.size();
    for(int i = 0, j = 0; i < k; ++i)
    {
        /// TO DO: improve accuracy
        int u = floor(brisk_keypoints_2D.points[i].x + 0.5);
        int v = floor(brisk_keypoints_2D.points[i].y + 0.5);

        j = u + v * input->width;

        if(isnan(input->points[j].x))
        {
            --k;
        }
        else
        {
            output->points[i]=input->points[j];
            /* output->points[i].b=input->points[j].b;
            output->points[i].g=input->points[j].g;
            output->points[i].r=input->points[j].r;
            output->points[i].x=input->points[j].x;
            output->points[i].y=input->points[j].y;
            output->points[i].z=input->points[j].z;*/
        }
    }

    std::vector<PointNormalT,Eigen::aligned_allocator<PointNormalT> >::iterator    keypointIt=output->begin();

    for(size_t i=k; k<brisk_keypoints_2D.size(); ++k)
        output->erase(keypointIt+i);



    pcl::PointIndices::Ptr indices (new pcl::PointIndices);

    // Remove NAN from keypoints cloud
    pcl::removeNaNFromPointCloud(*output, *output,indices->indices);
    pcl::removeNaNNormalsFromPointCloud(*output, *output,indices->indices);



    /*  // remove 0 points
    pcl::ConditionAnd<PointT>::Ptr range_cond (new pcl::ConditionAnd<PointT> ());
    range_cond->addComparison (pcl::FieldComparison<PointT>::ConstPtr (new pcl::FieldComparison<PointT> ("z", pcl::ComparisonOps::GT, 0.1)));

    // build the filter
    pcl::ConditionalRemoval<PointT> condrem(true);
    condrem.setCondition(range_cond);

    condrem.setInputCloud (output);
    condrem.setKeepOrganized(output->isOrganized());
    condrem.filter (*output);

*/

    std::cout << "DONE " << output->size() << " in " << t.elapsed() << std::endl;

}


#include <pcl/features/shot.h>
#include <pcl/filters/extract_indices.h>

void MatrixOfCloud::DescribeCSHOT(PointCloudNormalT::Ptr input,PointCloudNormalT::Ptr keypoints, pcl::PointCloud<pcl::SHOT1344>::Ptr descriptor )
{

    pcl::ScopeTime t ("DescribeCSHOT");
    //cout << "Description with SHOTRGB..." <<endl;


    // Calculate features FPFH
    pcl::SHOTColorEstimation<PointNormalT, PointNormalT, pcl::SHOT1344> shotrgb;
    shotrgb.setInputCloud (keypoints);
    shotrgb.setSearchSurface(input);
    shotrgb.setInputNormals (input);

    // Create an empty kdtree representation, and pass it to the FPFH estimation object.
    // Its content will be filled inside the object, based on the given input dataset (as no other search surface is given).
    pcl::search::KdTree<PointNormalT>::Ptr tree (new pcl::search::KdTree<PointNormalT>);

    shotrgb.setSearchMethod (tree);

    // Use all neighbors in a sphere of radius 5cm
    // IMPORTANT: the radius used here has to be larger than the radius used to estimate the surface normals!!!
    shotrgb.setRadiusSearch (0.05);

    // Compute the features
    shotrgb.compute(*descriptor);

    //cout << "OK. keypoints described in " << Timer->elapsed() << " milliseconds" << endl;

    /*
    // Check if there are NAN points in input cloud
    for ( size_t i=0; i<input_noNAN->size(); i++ )
    {
        cout <<  "x:   " << input_noNAN->points[i].x << "   y:   " << input_noNAN->points[i].y << "   z:   " << input_noNAN->points[i].z <<"  normal_x:   " << input_noNAN->points[i].normal_x  << "  normal_y:   "<< input_noNAN->points[i].normal_y <<"  normal_z:   "<< input_noNAN->points[i].normal_z <<endl;
    }

    // Visualize histogram
    for ( size_t i=0; i<descriptor->size(); i++ )
        for ( int j=0; j<33; j++ )
            cout << descriptor->points[i].histogram[j] <<endl;
*/

    pcl::PointIndices::Ptr indices_NAN_feature (new pcl::PointIndices);
    bool found_NAN_at_i =false;

    //Take Nan indices of point of features that have an histogram value NAN
    for ( size_t i=0; i<descriptor->size(); i++ )
    {
        found_NAN_at_i =false;
        for ( int j=0; j<1344; j++ )
            if( !pcl_isfinite(descriptor->points[i].descriptor[j]))
                found_NAN_at_i =true;


        if(found_NAN_at_i)
            indices_NAN_feature->indices.push_back(i);
    }

    // Filter descriptor with the indices founded
    pcl::PointCloud<pcl::SHOT1344>::Ptr descriptor_noNAN ( new pcl::PointCloud<pcl::SHOT1344>);

    pcl::ExtractIndices<pcl::SHOT1344> extract;
    extract.setInputCloud (descriptor);
    extract.setIndices (indices_NAN_feature);
    extract.setNegative (true);
    extract.filter (*descriptor_noNAN);
    *descriptor = *descriptor_noNAN;

    /*
      // Check same length -> obviously not!
      cout << "Try: Is:" << endl;
      cout << descriptor->size() << " = " << keypoints->size() << endl;
     */

    // We have to erase the keypoints that have no more features.
    pcl::ExtractIndices<pcl::PointXYZRGBNormal> extractKP;
    extractKP.setInputCloud (keypoints);
    extractKP.setIndices (indices_NAN_feature);
    extractKP.setNegative (true);
    extractKP.filter (*keypoints);

    // Check same length ->obviously yes!
    //cout << "Try2: Is:" << endl;
    //cout <<  descriptor->size() << " = " << keypoints->size() << endl;
}

