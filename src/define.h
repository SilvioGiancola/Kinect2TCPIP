#ifndef DEFINE_H
#define DEFINE_H

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <Eigen/Dense>

typedef pcl::PointXYZRGBNormal PointT;
typedef pcl::PointCloud<PointT> PointCloudT;

typedef Eigen::Transform<float,3,Eigen::Affine> TransformT;

#define SUCCESS 0
#define ERROR 1

#define PROTOCOL_GRAB "Grab"
#define PROTOCOL_OPEN "Open"
#define PROTOCOL_CLOSE "Close"
#define PROTOCOL_PIPELINE "Pipeline_"
#define PROTOCOL_SAVE "SAVE_"

#define DATEFORMAT "yyyy.MM.dd_hh.mm.ss.zzz"


#endif // DEFINE_H
