#include "MatrixOfCloud.h"

MatrixOfCloud::MatrixOfCloud()
{
    //myDownSampler.setMethod(BRISK);
    //   myDownSampler.BRISK_Octave = 4;
    //  myDownSampler.BRISK_Threshold = 2;

    // myMatcher.setMethod(PFH);
    //  myMatcher::reciprok = true;


    max_dist = 0.5;
    iteration = 1;
    pt2pl = true;
    //   decimation = 0.5;

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


void MatrixOfCloud::TransformLine(int l, Transform T, Transform refPose)
{
    for (int i = 0; i < 4; i++)
    {
        Transform currentPose = Transform( this->getPointCloud(l, i)->sensor_origin_,
                                           this->getPointCloud(l, i)->sensor_orientation_);
        currentPose = currentPose.premultiplyby(refPose.inverse()).premultiplyby(T).premultiplyby(refPose);

        this->getPointCloud(l, i)->sensor_origin_ = currentPose.getOrigin4();
        this->getPointCloud(l, i)->sensor_orientation_ = currentPose.getQuaternion();
    }
}

// ELABORATION functions
void MatrixOfCloud::EstimateNormals()
{
    for (int line = 0; line < _MatrixOfCloud.count(); line++)
    {
        pcl::ScopeTime t(QString("Normals for line %1 ").arg(line).toStdString().c_str());
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
        pcl::ScopeTime t(QString("Remove outliers for line %1 ").arg(line).toStdString().c_str());
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

void MatrixOfCloud::SinglePreOrient(double Rx, double Ry, double Rz)
{
    Transform T(0,0,0,(float)Rx,Ry,Rz);
    qDebug() << T.prettyprint();

    for (int l = 0; l < this->getNumberOfPointCloudLines(); l++)
    {
        for (int i = 0; i < 4; i++)
        {
            Transform currentPose = Transform( this->getPointCloud(l, i)->sensor_origin_,
                                               this->getPointCloud(l, i)->sensor_orientation_);
            currentPose = currentPose.postmultiplyby(T);

            this->getPointCloud(l, i)->sensor_origin_ = currentPose.getOrigin4();
            this->getPointCloud(l, i)->sensor_orientation_ = currentPose.getQuaternion();
        }
    }
}

void MatrixOfCloud::BackBoneAlignICP(int refindex, int newindex, QString ID)
{
    // for (int i = 0; i < _MatrixOfCloud.length() - 1; i++)
    // {
    for (int iter = 0; iter < iteration; iter++)
    {
        PointCloudNormalT::Ptr PC_Target(new PointCloudNormalT);
        PC_Target = this->getPointCloud(refindex,ID);
        qDebug() << "PC_Target: " << PC_Target->size();
        qDebug() << "PC_Target: " << QString::fromStdString(PC_Target->header.frame_id);

        PointCloudNormalT::Ptr PC_Input(new PointCloudNormalT);
        PC_Input = this->getPointCloud(newindex,ID);
        qDebug() << "PC_Input: " << PC_Input->size();
        qDebug() << "PC_Input: " << QString::fromStdString(PC_Input->header.frame_id);


        /*  float decimate_percent = 1.0;
            if (iter < 10)  decimate_percent = 0.0625;
            else if (iter < 15)  decimate_percent = 0.25;
            else if (iter < 18)  decimate_percent = 0.5;
            else if (iter < 20)  decimate_percent = 1.0;

            float corr_max_distance = 0.1;
            if (iter < 10)  corr_max_distance = (0.05);
            else if (iter < 15)  corr_max_distance = 0.2;
            else if (iter < 18)  corr_max_distance = (0.1);
            else if (iter < 20)  corr_max_distance = (0.05);
*/


        PointCloudNormalT::Ptr PC_Target_copy(new PointCloudNormalT);
        PointCloudNormalT::Ptr PC_Input_copy(new PointCloudNormalT);
        pcl::copyPointCloud(*PC_Target, *PC_Target_copy);
        pcl::copyPointCloud(*PC_Input, *PC_Input_copy);


        // perform alignment
        Transform T = AlignICP(PC_Target_copy, PC_Input_copy);

        Transform refPose = Transform( this->getPointCloud(refindex, 0)->sensor_origin_,
                                       this->getPointCloud(refindex, 0)->sensor_orientation_);

        for (int l = newindex; l < this->getNumberOfPointCloudLines(); l++)
        {
            this->TransformLine(l, T, refPose);

            /*    for (int j = 0; j < 4; j++)
            {
                Transform currentPose = Transform( this->getPointCloud(l, j)->sensor_origin_,
                                                   this->getPointCloud(l, j)->sensor_orientation_);
                currentPose = T.postmultiplyby(currentPose);
                // currentPose.print();

                this->getPointCloud(l, j)->sensor_origin_ = currentPose.getOrigin4();
                this->getPointCloud(l, j)->sensor_orientation_ = currentPose.getQuaternion();

                // this->getPointCloud(l, j)->sensor_origin_ = T.block<4,1>(0,3)+ this->getPointCloud(l, j)->sensor_origin_;
                // this->getPointCloud(l, j)->sensor_orientation_ = Eigen::Quaternionf(T.block<3,3>(0,0)) * this->getPointCloud(l, j)->sensor_orientation_;
            }*/
        }

    }
    // }
}

void MatrixOfCloud::BackBoneAlignRANSAC(int refindex, int newindex, QString ID)
{
    // for (int i = 0; i < _MatrixOfCloud.length() - 1; i++)
    // {
    Transform BestTransform = Transform();

    int best_i = 0;
    if (ID.compare("smallest covariance") == 0)
    {
        double smallestVariance = 99999;
        for (int iter = 0; iter < 4; iter++)
        {
            PointCloudNormalT::Ptr PC_Target(new PointCloudNormalT);
            PC_Target = this->getPointCloud(refindex,iter);
            qDebug() << "PC_Target: " << PC_Target->size();
            qDebug() << "PC_Target: " << QString::fromStdString(PC_Target->header.frame_id);

            PointCloudNormalT::Ptr PC_Input(new PointCloudNormalT);
            PC_Input = this->getPointCloud(newindex,iter);
            qDebug() << "PC_Input: " << PC_Input->size();
            qDebug() << "PC_Input: " << QString::fromStdString(PC_Input->header.frame_id);



            PointCloudNormalT::Ptr PC_Target_copy(new PointCloudNormalT);
            PointCloudNormalT::Ptr PC_Input_copy(new PointCloudNormalT);
            pcl::copyPointCloud(*PC_Target, *PC_Target_copy);
            pcl::copyPointCloud(*PC_Input, *PC_Input_copy);


            // perform alignment
            double variance = 0;
            int inliers = 0;
            Transform T = AlignRANSAC(PC_Target_copy, PC_Input_copy, &variance, &inliers);
            qDebug() << "     >>>>    Variance = " << variance;
            if (variance < smallestVariance)
            {
                best_i = iter;
                smallestVariance = variance;
                BestTransform = T ;
                qDebug() << "     >>>>    New BEST!! = " << smallestVariance;
            }
        }
    }

    else if (ID.compare("most inliers") == 0)
    {
        int mostinliers = 0;
        for (int iter = 0; iter < 4; iter++)
        {
            PointCloudNormalT::Ptr PC_Target(new PointCloudNormalT);
            PC_Target = this->getPointCloud(refindex,iter);
            qDebug() << "PC_Target: " << PC_Target->size();
            qDebug() << "PC_Target: " << QString::fromStdString(PC_Target->header.frame_id);

            PointCloudNormalT::Ptr PC_Input(new PointCloudNormalT);
            PC_Input = this->getPointCloud(newindex,iter);
            qDebug() << "PC_Input: " << PC_Input->size();
            qDebug() << "PC_Input: " << QString::fromStdString(PC_Input->header.frame_id);



            PointCloudNormalT::Ptr PC_Target_copy(new PointCloudNormalT);
            PointCloudNormalT::Ptr PC_Input_copy(new PointCloudNormalT);
            pcl::copyPointCloud(*PC_Target, *PC_Target_copy);
            pcl::copyPointCloud(*PC_Input, *PC_Input_copy);


            // perform alignment
            double variance = 0;
            int inliers = 0;
            Transform T = AlignRANSAC(PC_Target_copy, PC_Input_copy, &variance, &inliers);
            qDebug() << "     >>>>    inliers = " << inliers;
            if (inliers > mostinliers)
            {
                best_i = iter;
                mostinliers = inliers;
                BestTransform = T ;
                qDebug() << "     >>>>    New BEST!! = " << mostinliers;
            }
        }
    }
    else
    {
        PointCloudNormalT::Ptr PC_Target(new PointCloudNormalT);
        PC_Target = this->getPointCloud(refindex,ID);
        qDebug() << "PC_Target: " << PC_Target->size();
        qDebug() << "PC_Target: " << QString::fromStdString(PC_Target->header.frame_id);

        PointCloudNormalT::Ptr PC_Input(new PointCloudNormalT);
        PC_Input = this->getPointCloud(newindex,ID);
        qDebug() << "PC_Input: " << PC_Input->size();
        qDebug() << "PC_Input: " << QString::fromStdString(PC_Input->header.frame_id);



        PointCloudNormalT::Ptr PC_Target_copy(new PointCloudNormalT);
        PointCloudNormalT::Ptr PC_Input_copy(new PointCloudNormalT);
        pcl::copyPointCloud(*PC_Target, *PC_Target_copy);
        pcl::copyPointCloud(*PC_Input, *PC_Input_copy);


        // perform alignment
        double variance = 0;
        int inliers = 0;
        Transform T = AlignRANSAC(PC_Target_copy, PC_Input_copy, &variance, &inliers);
        qDebug() << "     >>>>    Variance = " << variance;
        qDebug() << "     >>>>    inliers = " << inliers;
        BestTransform = T;

    }

    Transform refPose = Transform( this->getPointCloud(refindex, 0)->sensor_origin_,
                                   this->getPointCloud(refindex, 0)->sensor_orientation_);

    for (int l = newindex; l < this->getNumberOfPointCloudLines(); l++)
    {
        this->TransformLine(l, BestTransform, refPose);
        /*
        for (int j = 0; j < 4; j++)
        {
            Transform currentPose = Transform( this->getPointCloud(l, j)->sensor_origin_,
                                               this->getPointCloud(l, j)->sensor_orientation_);
            currentPose = BestTransform.postmultiplyby(currentPose);
            // currentPose.print();

            this->getPointCloud(l, j)->sensor_origin_ = currentPose.getOrigin4();
            this->getPointCloud(l, j)->sensor_orientation_ = currentPose.getQuaternion();

            // this->getPointCloud(l, j)->sensor_origin_ = T.block<4,1>(0,3)+ this->getPointCloud(l, j)->sensor_origin_;
            // this->getPointCloud(l, j)->sensor_orientation_ = Eigen::Quaternionf(T.block<3,3>(0,0)) * this->getPointCloud(l, j)->sensor_orientation_;
        }*/
    }


    // }
}

void MatrixOfCloud::AlignLinesICP(int refindex, int newindex)
{
    SetOfCloud refline = _MatrixOfCloud.at(refindex);
    SetOfCloud newline = _MatrixOfCloud.at(newindex);



    for (int iter = 0; iter < iteration; iter++)
    {
        PointCloudNormalT::Ptr PC_Target(new PointCloudNormalT);
        PC_Target = refline.getMergedPointCloud();
        qDebug() << "PC_Target: " << PC_Target->size();


        PointCloudNormalT::Ptr PC_Input(new PointCloudNormalT);
        PC_Input = newline.getMergedPointCloud();
        qDebug() << "PC_Input: " << PC_Input->size();


        // perform alignment
        Transform T = AlignICP(PC_Target, PC_Input);

        Transform refPose = Transform( this->getPointCloud(refindex, 0)->sensor_origin_,
                                       this->getPointCloud(refindex, 0)->sensor_orientation_);

        for (int l = newindex; l < this->getNumberOfPointCloudLines(); l++)
        {
            this->TransformLine(l, T, refPose);
        }
    }

}

void MatrixOfCloud::AlignLinesRANSAC(int refindex, int newindex)
{

    PointCloudNormalT::Ptr PC_Target_BRISK_Concat(new PointCloudNormalT);
    PointCloudNormalT::Ptr PC_Input_BRISK_Concat(new PointCloudNormalT);
    for (int iter = 0; iter < 4; iter++)
    {
        PointCloudNormalT::Ptr PC_Target(new PointCloudNormalT);
        //PC_Target = this->getPointCloud(refindex, iter);
        pcl::copyPointCloud(*this->getPointCloud(refindex, iter), *PC_Target);
        qDebug() << "PC_Target: " << PC_Target->size();
        qDebug() << "PC_Target: " << QString::fromStdString(PC_Target->header.frame_id);

        PointCloudNormalT::Ptr PC_Input(new PointCloudNormalT);
        //PC_Input = this->getPointCloud(newindex, iter);
        pcl::copyPointCloud(*this->getPointCloud(newindex, iter), *PC_Input);
        qDebug() << "PC_Input: " << PC_Input->size();
        qDebug() << "PC_Input: " << QString::fromStdString(PC_Input->header.frame_id);




        PointCloudNormalT::Ptr PC_Target_BRISK(new PointCloudNormalT);
        // DetectBRISK(PC_Target,PC_Target_BRISK,5,4);

        PointCloudNormalT::Ptr PC_Input_BRISK(new PointCloudNormalT);
        //DetectBRISK(PC_Input,PC_Input_BRISK,5,4);

        PC_Target_BRISK = myDownSampler.downsample(PC_Target);
        PC_Input_BRISK = myDownSampler.downsample(PC_Input);

        pcl::transformPointCloudWithNormals(*PC_Target_BRISK, *PC_Target_BRISK, PC_Target->sensor_origin_.head(3), PC_Target->sensor_orientation_);
        pcl::transformPointCloudWithNormals(*PC_Input_BRISK, *PC_Input_BRISK, PC_Input->sensor_origin_.head(3), PC_Input->sensor_orientation_);



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


        *PC_Target_BRISK_Concat = *PC_Target_BRISK_Concat + *PC_Target_BRISK;
        *PC_Input_BRISK_Concat = *PC_Input_BRISK_Concat + *PC_Input_BRISK;

    }
    SetOfCloud refline = _MatrixOfCloud.at(refindex);
    SetOfCloud newline = _MatrixOfCloud.at(newindex);

    PointCloudNormalT::Ptr PC_Target_Concat(new PointCloudNormalT);
    PC_Target_Concat = refline.getMergedPointCloud();
    qDebug() << "PC_Target_Concat: " << PC_Target_Concat->size();


    PointCloudNormalT::Ptr PC_Input_Concat(new PointCloudNormalT);
    PC_Input_Concat = newline.getMergedPointCloud();
    qDebug() << "PC_Input_Concat: " << PC_Input_Concat->size();



    std::vector<int> ind;
    //  PointCloudT::Ptr Model_nonan (new PointCloudT);
    // PointCloudT::Ptr PC_nonan (new PointCloudT);
    pcl::removeNaNFromPointCloud(*PC_Target_Concat, *PC_Target_Concat, ind);
    pcl::removeNaNNormalsFromPointCloud(*PC_Target_Concat, *PC_Target_Concat, ind);
    pcl::removeNaNFromPointCloud(*PC_Input_Concat, *PC_Input_Concat, ind);
    pcl::removeNaNNormalsFromPointCloud(*PC_Input_Concat, *PC_Input_Concat, ind);

    /*
    pcl::PointCloud<pcl::SHOT1344>::Ptr PC_Target_desc (new pcl::PointCloud<pcl::SHOT1344>);
    DescribeCSHOT(PC_Target_Concat, PC_Target_BRISK_Concat, PC_Target_desc);

    pcl::PointCloud<pcl::SHOT1344>::Ptr PC_Input_desc (new pcl::PointCloud<pcl::SHOT1344>);
    DescribeCSHOT(PC_Input_Concat, PC_Input_BRISK_Concat, PC_Input_desc);
*/

    pcl::CorrespondencesPtr correspondences(new pcl::Correspondences);
    correspondences = myMatcher.match(PC_Input_Concat, PC_Input_BRISK_Concat, PC_Target_Concat, PC_Target_BRISK_Concat);

    /*  pcl::CorrespondencesPtr correspondences(new pcl::Correspondences);
    // Estimate Correspondences
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
    }*/


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
        return;





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

        model.reset(new pcl::SampleConsensusModelRegistration<PointNormalT>(PC_Input_BRISK_Concat, source_indices ));
        model->setInputTarget(PC_Target_BRISK_Concat, target_indices);


        pcl::RandomSampleConsensus<PointNormalT> sac(model, inlierThreshold);


        // int maxit = std::max(1000,(int)correspondences->size()*(int)correspondences->size()*(int)correspondences->size()*2);
        sac.setMaxIterations(100000);
        sac.setProbability(0.99);
        sac.computeModel();


        sac.getInliers(inliers);
        sac.getModelCoefficients (model_coefficients);
    }

    {
        pcl::SampleConsensusModelRegistration<PointNormalT>::Ptr newmodel = boost::static_pointer_cast<pcl::SampleConsensusModelRegistration<PointNormalT> >(model);
        pcl::ScopeTime t("ransac-refine");
        // Refine Model
        double refineModelSigma = 3.0;
        double refineModelIterations = 20;

        double inlier_distance_threshold_sqr = inlierThreshold * inlierThreshold;
        double error_threshold = inlierThreshold;
        double sigma_sqr = refineModelSigma * refineModelSigma;
        int refine_iterations = 0;
        bool inlier_changed = false, oscillating = false;
        std::vector<int> new_inliers, prev_inliers = inliers;
        std::vector<size_t> inliers_sizes;
        do
        {
            // Optimize the model coefficients
            //  model.reset(new pcl::SampleConsensusModelRegistration<PointNormalT>(KP_PC_rot, source_indices ));

            newmodel->optimizeModelCoefficients (prev_inliers, model_coefficients, model_coefficients);
            inliers_sizes.push_back (prev_inliers.size ());

            // Select the new inliers based on the optimized coefficients and new threshold
            newmodel->selectWithinDistance (model_coefficients, error_threshold, new_inliers);

            std::cout << QString("RANSAC refineModel: Number of inliers found (before/after): %1/%2, with an error threshold of %3.")
                         .arg((int)prev_inliers.size ()).arg((int)new_inliers.size ()).arg(error_threshold).toStdString() << std::endl;

            if (new_inliers.empty ())
            {
                ++refine_iterations;
                if (refine_iterations >= refineModelIterations)
                {
                    break;
                }
                continue;
            }

            // Estimate the variance and the new threshold
            double variance = newmodel->computeVariance ();
            error_threshold =  std::sqrt (std::min (inlier_distance_threshold_sqr, sigma_sqr * (variance)));

            std::cout << QString("RANSAC refineModel: New estimated error threshold: %1 (variance=%2) on iteration %3 out of %4.")
                         .arg(error_threshold).arg(variance).arg(refine_iterations).arg(refineModelIterations).toStdString() << std::endl;


            inlier_changed = false;
            std::swap (prev_inliers, new_inliers);

            // If the number of inliers changed, then we are still optimizing
            if (new_inliers.size () != prev_inliers.size ())
            {
                // Check if the number of inliers is oscillating in between two values
                if (inliers_sizes.size () >= 4)
                {
                    if (inliers_sizes[inliers_sizes.size () - 1] == inliers_sizes[inliers_sizes.size () - 3] &&
                            inliers_sizes[inliers_sizes.size () - 2] == inliers_sizes[inliers_sizes.size () - 4])
                    {
                        oscillating = true;
                        break;
                    }
                }
                inlier_changed = true;
                continue;
            }

            // Check the values of the inlier set
            for (size_t i = 0; i < prev_inliers.size (); ++i)
            {
                // If the value of the inliers changed, then we are still optimizing
                if (prev_inliers[i] != new_inliers[i])
                {
                    inlier_changed = true;
                    break;
                }
            }
        }
        while (inlier_changed && ++refine_iterations < refineModelIterations);

        // If the new set of inliers is empty, we didn't do a good job refining
        if (new_inliers.empty ())
            std::cout << "RANSAC refineModel: Refinement failed: got an empty set of inliers!" << std::endl;


        if (oscillating)
            std::cout << "RANSAC refineModel: Detected oscillations in the model refinement." << std::endl;



        std::swap (inliers, new_inliers);
    }



    Transformation.row (0) = model_coefficients.segment<4>(0);
    Transformation.row (1) = model_coefficients.segment<4>(4);
    Transformation.row (2) = model_coefficients.segment<4>(8);
    Transformation.row (3) = model_coefficients.segment<4>(12);



    std::cout << Transformation << std::endl;



    Transform T(Transformation);




    Transform refPose = Transform( this->getPointCloud(refindex, 0)->sensor_origin_,
                                   this->getPointCloud(refindex, 0)->sensor_orientation_);

    for (int l = newindex; l < this->getNumberOfPointCloudLines(); l++)
    {
        this->TransformLine(l, T, refPose);

        /*for (int l = newindex; l < this->getNumberOfPointCloudLines(); l++)
    {
        for (int i = 0; i < 4; i++)
        {
            Transform currentPose = Transform( this->getPointCloud(l, i)->sensor_origin_,
                                               this->getPointCloud(l, i)->sensor_orientation_);
            currentPose = T.postmultiplyby(currentPose);
            // currentPose.print();

            this->getPointCloud(l, i)->sensor_origin_ = currentPose.getOrigin4();
            this->getPointCloud(l, i)->sensor_orientation_ = currentPose.getQuaternion();
        }*/
    }

}



Transform MatrixOfCloud::AlignICP(PointCloudNormalT::Ptr PC_Target, PointCloudNormalT::Ptr PC_Input)
{
    // PRE - TRANSFORM
    pcl::transformPointCloudWithNormals(*PC_Target, *PC_Target, PC_Target->sensor_origin_.head(3), PC_Target->sensor_orientation_);
    pcl::transformPointCloudWithNormals(*PC_Input, *PC_Input, PC_Input->sensor_origin_.head(3), PC_Input->sensor_orientation_);

    qDebug() << "PC_Target: " << PC_Target->size();
    qDebug() << "PC_Input: " << PC_Input->size();


    /*  pcl::RandomSample<PointNormalT> random_sampler;
    random_sampler.setKeepOrganized(false);

    random_sampler.setInputCloud(PC_Target);
    random_sampler.setSample((int) (decimate_percent*PC_Target->points.size()));
    random_sampler.filter(*PC_Target);

    random_sampler.setInputCloud(PC_Input);
    random_sampler.setSample((int) (decimate_percent*PC_Input->points.size()));
    random_sampler.filter(*PC_Input);
*/
    PC_Target = myDownSampler.downsample(PC_Target);
    PC_Input = myDownSampler.downsample(PC_Input);

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
        pcl::ScopeTime t ("Reject Correspondences % overlap");
        pcl::registration::CorrespondenceRejectorTrimmed::Ptr cor_rej_trimmed (new pcl::registration::CorrespondenceRejectorTrimmed);
        //  cor_rej_dist->setMaximumDistance(max_dist);
        cor_rej_trimmed->setOverlapRatio(0.7);
        // correspondence_rejectors_.push_back(cor_rej_trimmed);
        //  ui->Registration_Trimmed->setChecked(true);
        if (cor_rej_trimmed->requiresSourcePoints())
        {
            pcl::PCLPointCloud2::Ptr Source2 (new pcl::PCLPointCloud2);
            pcl::toPCLPointCloud2 (*PC_Input, *Source2);
            cor_rej_trimmed->setSourcePoints(Source2);
        }
        if (cor_rej_trimmed->requiresTargetPoints())
        {
            pcl::PCLPointCloud2::Ptr Target2 (new pcl::PCLPointCloud2);
            pcl::toPCLPointCloud2 (*PC_Target, *Target2);
            cor_rej_trimmed->setTargetPoints(Target2);
        }
        cor_rej_trimmed->setInputCorrespondences (correspondences);
        cor_rej_trimmed->getCorrespondences (*correspondences);
        qDebug() << "corresp filtered : " << correspondences->size();
    }
    // Filter Correspondences
    {
        pcl::ScopeTime t ("Reject Correspondences");
        pcl::registration::CorrespondenceRejectorDistance::Ptr cor_rej_dist (new pcl::registration::CorrespondenceRejectorDistance);
        cor_rej_dist->setMaximumDistance(max_dist);
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

    if (pt2pl)
    {
        pcl::registration::TransformationEstimationPointToPlaneLLS<PointNormalT, PointNormalT> transf_est;
        transf_est.estimateRigidTransformation (*PC_Input, *PC_Target, *correspondences, transformation_);
    }
    else
    {
        pcl::registration::TransformationEstimationSVD<PointNormalT, PointNormalT> transf_est;
        transf_est.estimateRigidTransformation (*PC_Input, *PC_Target, *correspondences, transformation_);
    }

    Transform T(transformation_);
    qDebug() << "transf est : " << T.prettyprint();

    return T;
}

Transform MatrixOfCloud::AlignRANSAC(PointCloudNormalT::Ptr PC_Target, PointCloudNormalT::Ptr PC_Input, double *variance , int *n_inliers )
{
    *variance = 99999;
    *n_inliers = 0;

    pcl::transformPointCloudWithNormals(*PC_Target, *PC_Target, PC_Target->sensor_origin_.head(3), PC_Target->sensor_orientation_);
    // pcl::transformPointCloudWithNormals(*PC_Target_BRISK, *PC_Target_BRISK, PC_Target_BRISK->sensor_origin_.head(3), PC_Target_BRISK->sensor_orientation_);

    pcl::transformPointCloudWithNormals(*PC_Input, *PC_Input, PC_Input->sensor_origin_.head(3), PC_Input->sensor_orientation_);
    // pcl::transformPointCloudWithNormals(*PC_Input_BRISK, *PC_Input_BRISK, PC_Input_BRISK->sensor_origin_.head(3), PC_Input_BRISK->sensor_orientation_);



    PointCloudNormalT::Ptr PC_Target_BRISK(new PointCloudNormalT);
    PointCloudNormalT::Ptr PC_Input_BRISK(new PointCloudNormalT);
    //Downsampler myDownSampler;

    PC_Target_BRISK = myDownSampler.downsample(PC_Target);
    PC_Input_BRISK = myDownSampler.downsample(PC_Input);

    //  DetectBRISK(PC_Input,PC_Input_BRISK,15,4);




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
    /*
    pcl::PointCloud<pcl::SHOT1344>::Ptr PC_Target_desc (new pcl::PointCloud<pcl::SHOT1344>);
   // DescribeCSHOT(PC_Target, PC_Target_BRISK, PC_Target_desc);

    pcl::PointCloud<pcl::SHOT1344>::Ptr PC_Input_desc (new pcl::PointCloud<pcl::SHOT1344>);
   // DescribeCSHOT(PC_Input, PC_Input_BRISK, PC_Input_desc);

*/



    pcl::CorrespondencesPtr correspondences(new pcl::Correspondences);
    correspondences = myMatcher.match(PC_Input, PC_Input_BRISK, PC_Target, PC_Target_BRISK);

    qDebug() << "correspondence size : " << correspondences->size();

    /*   // Estimate Correspondences
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

    }
*/


    // Filter Correspondences
    {
        pcl::ScopeTime t ("Reject Correspondences");
        pcl::registration::CorrespondenceRejectorDistance::Ptr cor_rej_dist (new pcl::registration::CorrespondenceRejectorDistance);
        cor_rej_dist->setMaximumDistance(max_dist);
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


        //   int maxit = std::max(1000,(int)correspondences->size()*(int)correspondences->size()*(int)correspondences->size()*2);
        sac.setMaxIterations(10000);
        sac.setProbability(0.99);
        sac.computeModel();


        sac.getInliers(inliers);
        sac.getModelCoefficients (model_coefficients);
    }

    {
        pcl::SampleConsensusModelRegistration<PointNormalT>::Ptr newmodel = boost::static_pointer_cast<pcl::SampleConsensusModelRegistration<PointNormalT> >(model);
        pcl::ScopeTime t("ransac-refine");
        // Refine Model
        double refineModelSigma = 3.0;
        double refineModelIterations = 5;

        double inlier_distance_threshold_sqr = inlierThreshold * inlierThreshold;
        double error_threshold = inlierThreshold;
        double sigma_sqr = refineModelSigma * refineModelSigma;
        int refine_iterations = 0;
        bool inlier_changed = false, oscillating = false;
        std::vector<int> new_inliers, prev_inliers = inliers;
        std::vector<size_t> inliers_sizes;
        do
        {
            // Optimize the model coefficients
            //  model.reset(new pcl::SampleConsensusModelRegistration<PointNormalT>(KP_PC_rot, source_indices ));

            newmodel->optimizeModelCoefficients (prev_inliers, model_coefficients, model_coefficients);
            inliers_sizes.push_back (prev_inliers.size ());

            // Select the new inliers based on the optimized coefficients and new threshold
            newmodel->selectWithinDistance (model_coefficients, error_threshold, new_inliers);

            std::cout << QString("RANSAC refineModel: Number of inliers found (before/after): %1/%2, with an error threshold of %3.")
                         .arg((int)prev_inliers.size ()).arg((int)new_inliers.size ()).arg(error_threshold).toStdString() << std::endl;

            if (new_inliers.empty ())
            {
                ++refine_iterations;
                if (refine_iterations >= refineModelIterations)
                {
                    break;
                }
                continue;
            }

            // Estimate the variance and the new threshold
            *variance = newmodel->computeVariance ();
            error_threshold =  std::sqrt (std::min (inlier_distance_threshold_sqr, sigma_sqr * (*variance)));

            std::cout << QString("RANSAC refineModel: New estimated error threshold: %1 (variance=%2) on iteration %3 out of %4.")
                         .arg(error_threshold).arg(*variance).arg(refine_iterations).arg(refineModelIterations).toStdString() << std::endl;

            *variance = *variance/(int)new_inliers.size ();
            inlier_changed = false;
            std::swap (prev_inliers, new_inliers);

            // If the number of inliers changed, then we are still optimizing
            if (new_inliers.size () != prev_inliers.size ())
            {
                // Check if the number of inliers is oscillating in between two values
                if (inliers_sizes.size () >= 4)
                {
                    if (inliers_sizes[inliers_sizes.size () - 1] == inliers_sizes[inliers_sizes.size () - 3] &&
                            inliers_sizes[inliers_sizes.size () - 2] == inliers_sizes[inliers_sizes.size () - 4])
                    {
                        oscillating = true;
                        break;
                    }
                }
                inlier_changed = true;
                continue;
            }

            // Check the values of the inlier set
            for (size_t i = 0; i < prev_inliers.size (); ++i)
            {
                // If the value of the inliers changed, then we are still optimizing
                if (prev_inliers[i] != new_inliers[i])
                {
                    inlier_changed = true;
                    break;
                }
            }
        }
        while (inlier_changed && ++refine_iterations < refineModelIterations);

        // If the new set of inliers is empty, we didn't do a good job refining
        if (new_inliers.empty ())
            std::cout << "RANSAC refineModel: Refinement failed: got an empty set of inliers!" << std::endl;


        if (oscillating)
            std::cout << "RANSAC refineModel: Detected oscillations in the model refinement." << std::endl;

        *n_inliers = (int)new_inliers.size();

        std::swap (inliers, new_inliers);
    }



    Transformation.row (0) = model_coefficients.segment<4>(0);
    Transformation.row (1) = model_coefficients.segment<4>(4);
    Transformation.row (2) = model_coefficients.segment<4>(8);
    Transformation.row (3) = model_coefficients.segment<4>(12);



    std::cout << Transformation << std::endl;



    return Transform(Transformation);
}
