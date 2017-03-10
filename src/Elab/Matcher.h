#ifndef Matcher_H_
#define Matcher_H_

#include "define.h"

#include <pcl/correspondence.h>
#include <pcl/registration/correspondence_estimation.h>
#include <pcl/features/fpfh.h>
#include <pcl/features/pfh.h>
#include <pcl/features/pfhrgb.h>
#include <pcl/features/shot.h>


#include <pcl/filters/extract_indices.h>

#include <QElapsedTimer>


enum MatchingMethod { Dist = 0, PFH, PFHRGB, FPFH, SHOT, CSHOT};
enum DescriptionMethod { KSearch= 0, RadiusSearch};

class Matcher
{
public:
    Matcher();
    virtual ~Matcher();


  /*  void setMethod(MatchingMethod newMethod){myMethod = newMethod;}
    MatchingMethod geMethod(){return myMethod;}*/

    pcl::CorrespondencesPtr match(PointCloudNormalT::Ptr Input, PointCloudNormalT::Ptr Input_KP, PointCloudNormalT::Ptr Target, PointCloudNormalT::Ptr Target_KP);

    static MatchingMethod myMethod;
    static DescriptionMethod myDescMethod;
    static bool reciprok;

    static double radiusSearchValue;
    static int kSearchValue;

private:

    // DESCRIPTION
    pcl::PointCloud<pcl::PFHSignature125>::Ptr DescribePFH(PointCloudNormalT::Ptr input,PointCloudNormalT::Ptr keypoints);
    pcl::PointCloud<pcl::PFHRGBSignature250>::Ptr DescribePFHRGB(PointCloudNormalT::Ptr input, PointCloudNormalT::Ptr keypoints);
    pcl::PointCloud<pcl::FPFHSignature33>::Ptr DescribeFPFH(PointCloudNormalT::Ptr input, PointCloudNormalT::Ptr keypoints);
    pcl::PointCloud<pcl::SHOT352>::Ptr DescribeSHOT(PointCloudNormalT::Ptr input, PointCloudNormalT::Ptr keypoints);
    pcl::PointCloud<pcl::SHOT1344>::Ptr DescribeCSHOT(PointCloudNormalT::Ptr input, PointCloudNormalT::Ptr keypoints);


    //MATCHING
    pcl::CorrespondencesPtr MatchPFH(pcl::PointCloud<pcl::PFHSignature125>::Ptr Input_desc, pcl::PointCloud<pcl::PFHSignature125>::Ptr Target_desc);
    pcl::CorrespondencesPtr MatchPFHRGB(pcl::PointCloud<pcl::PFHRGBSignature250>::Ptr Input_desc, pcl::PointCloud<pcl::PFHRGBSignature250>::Ptr Target_desc);
    pcl::CorrespondencesPtr MatchFPFH(pcl::PointCloud<pcl::FPFHSignature33>::Ptr Input_desc, pcl::PointCloud<pcl::FPFHSignature33>::Ptr Target_desc);
    pcl::CorrespondencesPtr MatchSHOT(pcl::PointCloud<pcl::SHOT352>::Ptr Input_desc, pcl::PointCloud<pcl::SHOT352>::Ptr Target_desc);
    pcl::CorrespondencesPtr MatchCSHOT(pcl::PointCloud<pcl::SHOT1344>::Ptr Input_desc, pcl::PointCloud<pcl::SHOT1344>::Ptr Target_desc);
    pcl::CorrespondencesPtr MatchDist(PointCloudNormalT::Ptr Input_desc, PointCloudNormalT::Ptr Target_desc);

    // template <typename Descriptor>
    // pcl::CorrespondencesPtr MatchDescriptor(pcl::PointCloud<Descriptor>::Ptr Input_desc, pcl::PointCloud<Descriptor>::Ptr Target_desc);
  /*  template <typename N>
    pcl::CorrespondencesPtr MatchDescriptor(pcl::PointCloud<N>::Ptr Input_desc, pcl::PointCloud<N>::Ptr Target_desc)
    {
        pcl::CorrespondencesPtr myCorrespondences(new pcl::Correspondences);
        pcl::registration::CorrespondenceEstimationBase<N, N>::Ptr myCorrespondenceEstimation;
        myCorrespondenceEstimation.reset(new pcl::registration::CorrespondenceEstimation<N, N>);
        //  pcl::CorrespondencesPtr correspondences(new pcl::Correspondences);
        myCorrespondenceEstimation->setInputSource (Input_desc);
        myCorrespondenceEstimation->setInputTarget (Target_desc);
        if (reciprok)
            myCorrespondenceEstimation->determineReciprocalCorrespondences (*myCorrespondences);
        else
            myCorrespondenceEstimation->determineCorrespondences (*myCorrespondences);

        return myCorrespondences;
    }*/

};

#endif /* Matcher_H_ */
