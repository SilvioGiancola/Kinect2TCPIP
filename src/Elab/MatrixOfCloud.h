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
#include <pcl/registration/correspondence_rejection_trimmed.h>
#include <pcl/registration/transformation_estimation_point_to_plane_lls.h>



#include <pcl/filters/conditional_removal.h>
#include <pcl/keypoints/brisk_2d.h>
#include <pcl/features/shot.h>
#include <pcl/filters/extract_indices.h>

//ICP
#include <pcl/registration/icp.h> //RegistrationICP

#include <Downsampler.h>
#include <Matcher.h>


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
    void SinglePreOrient(double Rx, double Ry, double Rz);
    void BackBoneAlignICP(int refindex, int newindex, QString ID);
    void BackBoneAlignRANSAC(int refindex, int newindex, QString ID);
    void AlignLinesICP(int refindex, int newindex);
    void AlignLinesRANSAC(int refindex, int newindex);

   // void setRANSACMatchingMethod(MatchingMethod NewMethod){myMatcher.setMethod(NewMethod);}
   // void setRANSACDownamplingMethod(DownsamplingMethod NewMethod){myDownSampler.setMethod(NewMethod);}

   void setMaxDistance(double value) {max_dist = value;}
    void setICPIteration(int value) {iteration = value;}
  //  void setICPDecimation(double value) {decimation = value;}

    bool pt2pl;

    void TransformLine(int l, Transform T, Transform refPose);


private:
    QList<SetOfCloud> _MatrixOfCloud;


    Transform AlignICP(PointCloudNormalT::Ptr PC_Target, PointCloudNormalT::Ptr PC_Input);
    Transform AlignRANSAC(PointCloudNormalT::Ptr PC_Target, PointCloudNormalT::Ptr PC_Input, double *variance, int *n_inliers);

   // void DetectBRISK(PointCloudNormalT::Ptr input, PointCloudNormalT::Ptr output, int paramThreshold, int octave);
  //  void DescribeCSHOT(PointCloudNormalT::Ptr input,PointCloudNormalT::Ptr keypoints, pcl::PointCloud<pcl::SHOT1344>::Ptr descriptor );


    Downsampler myDownSampler;
    Matcher myMatcher;
   double max_dist;
    int iteration;

  //  double decimation;*
};

#endif /* MatrixOfCloud_H_ */
