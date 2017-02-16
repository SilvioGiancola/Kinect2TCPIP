#ifndef MatrixOfCloud_H_
#define MatrixOfCloud_H_

#include <QVector3D>

#include <Eigen/Dense>


#include "define.h"
#include <QList>
#include <QStringList>

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
#include <pcl/correspondence.h>
#include <pcl/registration/correspondence_estimation.h>
#include <pcl/registration/correspondence_rejection_distance.h>
#include <pcl/registration/transformation_estimation_point_to_plane_lls.h>


class MatrixOfCloud
{
public:
    MatrixOfCloud();
    virtual ~MatrixOfCloud();

    PointCloudNormalT::Ptr getPointCloud(int line, int index);
    PointCloudNormalT::Ptr getLineOfPointCloud(int line);
    void setPointCloud(int line, int index, PointCloudNormalT::Ptr PC);
    PointCloudNormalT::Ptr getCompletePointCloud();

    int getNumberOfPointCloudLines();

    void parsePointClouds(QStringList path);


    void AlignLines(int refindex, int newindex);
    void EstimateNormals();
    void RemoveOutliers();

private:
    QList<SetOfCloud> _MatrixOfCloud;
};

#endif /* MatrixOfCloud_H_ */
