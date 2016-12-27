#ifndef DEFINE_H
#define DEFINE_H

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <QDebug>
#include "Transform.h"
#include <stdio.h>



// exceptions
#include <iostream>
#include <exception>


/*
template <typename PointT>
class PCL_EXPORTS myPointCloud : public pcl::PointCloud


//class PointCloudT : public pcl::PointCloud<PointT>
{
public:
    myPointCloud() : isVisible(true), showNormal(false), pointsize(1), opacity(1) { }

    bool isVisible;
    bool showNormal;
    int pointsize;
    double opacity;

public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};
*/

typedef pcl::PointXYZRGBA PointT;
typedef pcl::PointCloud<pcl::PointXYZRGBA> PointCloudT;


/*
class PointCloudT : public pcl::PointCloud<PointT>::Ptr
{
public:
    PointCloudT()
    {
        isVisible = true;
        showNormal = false;
        pointsize = 1;
        opacity = 1;
    };
    bool isVisible;
    bool showNormal;
    int pointsize;
    double opacity;
};*/

//

typedef pcl::PointXYZRGBNormal PointNormalT;
typedef pcl::PointCloud<PointNormalT> PointCloudNormalT;


#define SUCCESS 0
#define ERROR 1

#define PROTOCOL_GRAB "Grab"
#define PROTOCOL_OPEN "Open"
#define PROTOCOL_CLOSE "Close"
#define PROTOCOL_REGISTER "Register"
#define PROTOCOL_SAVE_SETTINGS "SaveSettings"
#define PROTOCOL_PIPELINE "Pipeline_" // + Value
#define PROTOCOL_SAVE "Save_" // + Value
#define PROTOCOL_TRANSMIT_POINTCLOUDS "get_PointClouds"
#define PROTOCOL_POSE "Pose_" //+ Pose
#define PROTOCOL_TIME "Time"



#define DATEFORMAT "yyyy.MM.dd"
#define TIMEFORMAT "hh.mm.ss.zzz"


#endif // DEFINE_H
