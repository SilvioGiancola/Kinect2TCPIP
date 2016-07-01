#ifndef DEFINE_H
#define DEFINE_H

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <Eigen/Dense>
#include <QDebug>
#include "Transform.h"
#include <stdio.h>


// exceptions
#include <iostream>
#include <exception>

typedef pcl::PointXYZRGBA PointT;
typedef pcl::PointCloud<PointT> PointCloudT;

typedef pcl::PointXYZRGBNormal PointNormalT;
typedef pcl::PointCloud<PointNormalT> PointCloudNormalT;


#define SUCCESS 0
#define ERROR 1

#define PROTOCOL_GRAB "MY_Grab"
#define PROTOCOL_GRAB_TRANSMIT "MY_Transmit"
#define PROTOCOL_OPEN "MY_Open"
#define PROTOCOL_CLOSE "MY_Close"
#define PROTOCOL_REGISTER "MY_Register"
#define PROTOCOL_SAVE_SETTINGS "MY_SaveSettings"
#define PROTOCOL_PIPELINE "MY_Pipeline_"
#define PROTOCOL_SAVE "MY_SAVE_"

#define DATEFORMAT "yyyy.MMM.dd"
#define TIMEFORMAT "hh.mm.ss.zzz"


#endif // DEFINE_H
