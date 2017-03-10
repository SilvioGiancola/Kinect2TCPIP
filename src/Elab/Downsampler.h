#ifndef Downsampler_H_
#define Downsampler_H_

#include "define.h"

#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/random_sample.h>
#include <pcl/keypoints/brisk_2d.h>
#include <pcl/keypoints/sift_keypoint.h>


#include <pcl/filters/conditional_removal.h>

enum DownsamplingMethod { BRISK=0, BRISK_QUAD, AGAST, SIFT3D, RANDOM_SAMPLE, VOXEL_GRID};


class Downsampler
{
public:
    Downsampler();
    virtual ~Downsampler();


 /*   void setMethod(DownsamplingMethod newMethod){myMethod = newMethod;}
    DownsamplingMethod getMethod(){return myMethod;}
*/
    PointCloudNormalT::Ptr downsample(PointCloudNormalT::Ptr Input);


    static double random_sample_decimate_percent;
    static double VoxelGrid_LeafSize;
    static double AGAST_Threshold;
    static double BRISK_Threshold;
    static int BRISK_Octave;
    static float min_scale;
    static int nr_octaves;
    static int nr_scales_per_octave;
    static float min_contrast;

    static DownsamplingMethod myMethod;
private:

    // DETECTION
    void DetectAGAST(PointCloudNormalT::Ptr input, PointCloudNormalT::Ptr output, int paramThreshold = 30);
    void DetectBRISK(PointCloudNormalT::Ptr input, PointCloudNormalT::Ptr output, int paramThreshold = 15, int octave = 4);
    void DetectBRISKQuad(PointCloudNormalT::Ptr input, PointCloudNormalT::Ptr output, int paramThreshold = 15, int octave = 4);
    void DetectSIFT(PointCloudNormalT::Ptr input, PointCloudNormalT::Ptr output);


};


#endif /* Downsampler_H_ */
