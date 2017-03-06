#ifndef MatrixOfCloud_H_
#define MatrixOfCloud_H_

#include <QVector3D>

#include <Eigen/Dense>


#include "define.h"
#include <QList>
#include <QStringList>
#include <QTime>

#include <pcl/common/transforms.h>
#include <pcl/io/pcd_io.h>

#include <SetOfCloud.h>


// Normal Estimation
#include <pcl/features/integral_image_normal.h>


// Outliers Removal
#include <pcl/filters/radius_outlier_removal.h>   // remove radius outliers


// Alignment stuff
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/random_sample.h>
#include <pcl/keypoints/brisk_2d.h>
#include <pcl/correspondence.h>
#include <pcl/registration/correspondence_estimation.h>
#include <pcl/registration/correspondence_rejection_distance.h>
#include <pcl/registration/transformation_estimation_point_to_plane_lls.h>


//ICP
#include <pcl/registration/icp.h> //RegistrationICP



class MatrixOfCloud
{
public:
    MatrixOfCloud();
    virtual ~MatrixOfCloud();

    PointCloudNormalT::Ptr getPointCloud(int line, int index);
    PointCloudNormalT::Ptr getPointCloud(int line, QString ID);
    PointCloudNormalT::Ptr getLineOfPointCloud(int line);
    void setPointCloud(int line, int index, PointCloudNormalT::Ptr PC);
    PointCloudNormalT::Ptr getCompletePointCloud();


    int getNumberOfPointCloudLines();


   // void openPointCloud(QStringList path);
    void openPointCloud(QStringList path, QStringList IDlist);
    void savePointClouds(QString newDir);
    void exportReconstruction(QString Path);


    void EstimateNormals();
    void RemoveOutliers();
    void BackBoneAlign(QString ID);
    void AlignLines(int refindex, int newindex);

private:
    QList<SetOfCloud> _MatrixOfCloud;


    Transform AlignICP(PointCloudNormalT::Ptr PC_Target, PointCloudNormalT::Ptr PC_Input, float decimate_percent = 1.0, float corr_max_distance = 0.2);
    Transform AlignRANSAC(PointCloudNormalT::Ptr PC_Target, PointCloudNormalT::Ptr PC_Input);

    void DetectBRISK(PointCloudNormalT::Ptr input, PointCloudNormalT::Ptr output, int paramThreshold, int octave);
    void DescribeCSHOT(PointCloudNormalT::Ptr input,PointCloudNormalT::Ptr keypoints, pcl::PointCloud<pcl::SHOT1344>::Ptr descriptor );

};

#endif /* MatrixOfCloud_H_ */
