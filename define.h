#ifndef DEFINE_H
#define DEFINE_H

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

typedef pcl::PointXYZRGBNormal PointT;
typedef pcl::PointCloud<PointT> PointCloudT;

#define SUCCESS 0
#define ERROR 1

#define PROTOCOL_GRAB "Grab"
#define PROTOCOL_OPEN "Open"
#define PROTOCOL_CLOSE "Close"
#define PROTOCOL_REBOOT "Reboot"
#define PROTOCOL_GITUPDATE "Git_Update"

#endif // DEFINE_H
