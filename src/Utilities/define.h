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

typedef pcl::PointXYZRGBA PointT;
typedef pcl::PointCloud<PointT> PointCloudT;

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
#define PROTOCOL_POSE "Pose_"
#define PROTOCOL_TIME "Time"



#define DATEFORMAT "yyyy.MM.dd"
#define TIMEFORMAT "hh.mm.ss.zzz"


#endif // DEFINE_H
