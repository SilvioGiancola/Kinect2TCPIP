#ifndef REGISTRATION_H
#define REGISTRATION_H


// TODO : Verify Registration

#include <define.h>


#include <pcl/io/pcd_io.h>
#include <pcl/common/transforms.h>  // transform point cloud
#include <pcl/common/io.h> // copy point cloud

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/openni_grabber.h>
#include <pcl/visualization/cloud_viewer.h>

#include <pcl/features/integral_image_normal.h>
#include <pcl/filters/filter.h>
#include <pcl/registration/icp.h> //RegistrationICP
#include <pcl/registration/correspondence_estimation_organized_projection.h>
#include <pcl/registration/correspondence_estimation_normal_shooting.h>
#include <pcl/registration/correspondence_rejection_one_to_one.h>
#include <pcl/registration/correspondence_rejection_median_distance.h>
#include <pcl/registration/transformation_estimation_point_to_plane.h>


namespace utils
{


Transform getTransformationNormal(PointCloudNormalT::Ptr PC_Target, PointCloudNormalT::Ptr PC_Input );



Transform getTransformation(PointCloudT::Ptr PC_Target, PointCloudT::Ptr PC_Input );
}

#endif // REGISTRATION_H
