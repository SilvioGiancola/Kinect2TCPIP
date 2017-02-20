
#include <registration.h>
namespace utils
{


Transform getTransformationNormal(PointCloudNormalT::Ptr PC_Target, PointCloudNormalT::Ptr PC_Input )
{
    try
    {/*
        ui->myKinectWidget1->GrabKinect();
        ui->myKinectWidget2->GrabKinect();
        PointCloudNormalT::Ptr PC_Target = ui->myKinectWidget1->getPointCloudNormal();
        PointCloudNormalT::Ptr PC_Input = ui->myKinectWidget2->getPointCloudNormal();
*/

        // NORMAL
        qDebug() << "NORMAL: " << PC_Target->size();
        pcl::IntegralImageNormalEstimation<PointNormalT, PointNormalT> ne;
        ne.setNormalEstimationMethod (ne.AVERAGE_3D_GRADIENT);
        ne.setMaxDepthChangeFactor(0.02);
        ne.setNormalSmoothingSize(10.0);

        ne.setInputCloud(PC_Target);
        ne.compute(*PC_Target);
        ne.setInputCloud(PC_Input);
        ne.compute(*PC_Input);


        // TRANSFORM
        qDebug() << "TRANSFORM: " << PC_Target->size();
        pcl::transformPointCloud(*PC_Target, *PC_Target, PC_Target->sensor_origin_.head(3), PC_Target->sensor_orientation_);
        pcl::transformPointCloud(*PC_Input, *PC_Input, PC_Input->sensor_origin_.head(3), PC_Input->sensor_orientation_);


        //REMOVE NAN
        qDebug() << "REMOVE NAN: " << PC_Target->size();
        std::vector<int> ind;
        pcl::removeNaNFromPointCloud(*PC_Target, *PC_Target, ind);
        pcl::removeNaNNormalsFromPointCloud(*PC_Target, *PC_Target, ind);
        pcl::removeNaNFromPointCloud(*PC_Input, *PC_Input, ind);
        pcl::removeNaNNormalsFromPointCloud(*PC_Input, *PC_Input, ind);




        // Creo il mio elemento ICP
        qDebug() << "ICP: " << PC_Target->size();
        pcl::IterativeClosestPoint<PointNormalT, PointNormalT> icp;




        // Corrispondence Estimation
        // Scelgo il mia stima dei accopiamenti
        qDebug() << "Corrispondence Estimation";
        pcl::registration::CorrespondenceEstimationBase<PointNormalT, PointNormalT>::Ptr cens;
        cens.reset(new pcl::registration::CorrespondenceEstimationNormalShooting<PointNormalT, PointNormalT,PointNormalT>);

        cens->setInputTarget (PC_Target);
        cens->setInputSource (PC_Input);

        icp.setCorrespondenceEstimation (cens);




        // Corrispondence Rejection
        qDebug() << "Corrispondence Rejection";



        pcl::registration::CorrespondenceRejectorOneToOne::Ptr cor_rej_o2o (new pcl::registration::CorrespondenceRejectorOneToOne);
        icp.addCorrespondenceRejector (cor_rej_o2o);

        /*
        pcl::registration::CorrespondenceRejectorMedianDistance::Ptr cor_rej_med (new pcl::registration::CorrespondenceRejectorMedianDistance);
        cor_rej_med->setInputTarget<PointNormalT> (PC_Target);
        cor_rej_med->setInputSource<PointNormalT> (PC_Input);
        icp.addCorrespondenceRejector (cor_rej_med);*/








        // Transformation Estimation
        // Scelgo un metodo per risolvere il problema
        qDebug() << "Transformation Estimation";
        pcl::registration::TransformationEstimation<PointNormalT, PointNormalT>::Ptr te;
        te.reset(new pcl::registration::TransformationEstimationPointToPlaneLLS<PointNormalT, PointNormalT>);

        icp.setTransformationEstimation (te);





        icp.setInputSource(PC_Input);
        icp.setInputTarget(PC_Target);


        // Modalità di fine ICP
        //icp.setEuclideanFitnessEpsilon(10E-9);
        //icp.setTransformationEpsilon(10E-9);
        icp.setMaximumIterations(1);


        qDebug() << "Align";
        PointCloudNormalT::Ptr Final(new PointCloudNormalT);
        icp.align(*Final);
        qDebug() << "has converged:" << icp.hasConverged() << " score: " << icp.getFitnessScore();


        Eigen::Matrix4f ICPtransformation = icp.getFinalTransformation();
     //   std::cout << ICPtransformation  << std::endl;




        return Transform(ICPtransformation);
        /*PointCloudT::Ptr PCnew = ui->myKinectWidget2->getPointCloud();
        T.setOrigin4(ICPtransformation.block<4,1>(0,3) + PCnew->sensor_origin_);
        T.setQuaternion(Eigen::Quaternionf(ICPtransformation.block<3,3>(0,0)) * PCnew->sensor_orientation_);

        T.print();

        ui->myKinectWidget2->setTransform(T);
        //  result->sensor_origin_ = ICPtransformation.block<4,1>(0,3) + PC->sensor_origin_;
        //  result->sensor_orientation_ = Eigen::Quaternionf(ICPtransformation.block<3,3>(0,0)) * PC->sensor_orientation_;

        writeSettings();

        ui->myKinectWidget1->GrabKinect();
        ui->myKinectWidget2->GrabKinect();*/
    }
    catch (std::exception& ex)
    {
        qDebug() << ex.what();
    }





}


Transform getTransformation(PointCloudT::Ptr PC_Target, PointCloudT::Ptr PC_Input )
{
    qDebug() << "getTransformation";
    PointCloudNormalT::Ptr PC_Target_Normal(new PointCloudNormalT);
    PointCloudNormalT::Ptr PC_Input_Normal(new PointCloudNormalT);

    pcl::copyPointCloud(*PC_Target, *PC_Target_Normal);
    pcl::copyPointCloud(*PC_Input, *PC_Input_Normal);

    return getTransformationNormal(PC_Target_Normal, PC_Input_Normal );
}


}
