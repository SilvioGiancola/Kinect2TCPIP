#include "Downsampler.h"

// Initialization of const variables
double Downsampler::BRISK_Threshold = 2;
int Downsampler::BRISK_Octave = 4;
double Downsampler::AGAST_Threshold=30;
double Downsampler::random_sample_decimate_percent=0.5;
double Downsampler::VoxelGrid_LeafSize=0.01;
DownsamplingMethod Downsampler::myMethod = BRISK;


float Downsampler::min_scale = 0.005f;
int Downsampler::nr_octaves = 6;
int Downsampler::nr_scales_per_octave = 4;
float Downsampler::min_contrast = 0.005f;


// Constructor
Downsampler::Downsampler()
{
   /* random_sample_decimate_percent = 0.5;
    VoxelGrid_LeafSize = 10;
    AGAST_Threshold = 30;
    BRISK_Threshold = 30;
    BRISK_Octave = 4;
    myMethod = BRISK;*/
}


Downsampler::~Downsampler()
{

}

PointCloudNormalT::Ptr Downsampler::downsample(PointCloudNormalT::Ptr Input)
{
    PointCloudNormalT::Ptr Output(new PointCloudNormalT);

    if ( myMethod == RANDOM_SAMPLE)
    {
        pcl::ScopeTime t("RandomSample");
        pcl::RandomSample<PointNormalT> random_sampler;

        random_sampler.setInputCloud(Input);
     //   random_sampler.setSample((int) (random_sample_decimate_percent*Input->points.size()));
        random_sampler.setSample(50000);
        random_sampler.setKeepOrganized(false);
        random_sampler.filter(*Output);
    }

    else if ( myMethod == VOXEL_GRID)
    {
        pcl::ScopeTime t("VoxelGrid");
        pcl::VoxelGrid<PointNormalT> sor;
        sor.setLeafSize (VoxelGrid_LeafSize, VoxelGrid_LeafSize, VoxelGrid_LeafSize);

        sor.setInputCloud (Input);
        sor.filter (*Output);
    }

    else if ( myMethod == SIFT3D)
    {
        DetectSIFT(Input,Output);
    }

    else if ( myMethod == AGAST)
    {
        DetectAGAST(Input, Output, AGAST_Threshold);
    }
    else if ( myMethod == BRISK)
    {
        DetectBRISK(Input, Output, BRISK_Threshold, BRISK_Octave);
    }
    else if ( myMethod == BRISK_QUAD)
    {
        DetectBRISKQuad(Input, Output, BRISK_Threshold, BRISK_Octave);
    }
    else
    {
        pcl::copyPointCloud( *Input, *Output );
        std::vector<int> ind;
        pcl::removeNaNFromPointCloud(*Output, *Output,ind);
    }

    Output->sensor_orientation_ = Input->sensor_orientation_;
    Output->sensor_origin_ = Input->sensor_origin_;


    return Output;
}


void Downsampler::DetectBRISK(PointCloudNormalT::Ptr input, PointCloudNormalT::Ptr output, int paramThreshold, int octave)
{
    std::cout << "BRISK Detector (Thresh=" << paramThreshold << ",Oct=" << octave << ")...";

    pcl::ScopeTime t("BRISK Detector");

    // constructor
    pcl::BriskKeypoint2D<PointNormalT> brisk_keypoint_estimation;

    //output
    pcl::PointCloud<pcl::PointWithScale> brisk_keypoints_2D;

    //parameters
    brisk_keypoint_estimation.setThreshold(paramThreshold);
    brisk_keypoint_estimation.setOctaves(octave);
    brisk_keypoint_estimation.setInputCloud (input);

    //compute
    brisk_keypoint_estimation.compute (brisk_keypoints_2D);

    //convert pointwithscale to 3D
    output->resize(brisk_keypoints_2D.size());

    int k = brisk_keypoints_2D.size();
    for(int i = 0, j = 0; i < k; ++i)
    {
        /// TO DO: improve accuracy
        int u = floor(brisk_keypoints_2D.points[i].x + 0.5);
        int v = floor(brisk_keypoints_2D.points[i].y + 0.5);

        j = u + v * input->width;

        if(isnan(input->points[j].x))
        {
            --k;
        }
        else
        {
            output->points[i]=input->points[j];
           /* output->points[i].b=input->points[j].b;
            output->points[i].g=input->points[j].g;
            output->points[i].r=input->points[j].r;
            output->points[i].x=input->points[j].x;
            output->points[i].y=input->points[j].y;
            output->points[i].z=input->points[j].z;*/
        }
    }

    std::vector<PointNormalT,Eigen::aligned_allocator<PointNormalT> >::iterator    keypointIt=output->begin();

    for(size_t i=k; k<brisk_keypoints_2D.size(); ++k)
        output->erase(keypointIt+i);



    pcl::PointIndices::Ptr indices (new pcl::PointIndices);

    // Remove NAN from keypoints cloud
    pcl::removeNaNFromPointCloud(*output, *output,indices->indices);
    pcl::removeNaNNormalsFromPointCloud(*output, *output,indices->indices);






    // remove 0 points
    pcl::ConditionAnd<PointNormalT>::Ptr range_cond (new pcl::ConditionAnd<PointNormalT> ());
    range_cond->addComparison (pcl::FieldComparison<PointNormalT>::ConstPtr (new pcl::FieldComparison<PointNormalT> ("z", pcl::ComparisonOps::GT, 0.1)));

    // build the filter
    pcl::ConditionalRemoval<PointNormalT> condrem(true);
    condrem.setCondition(range_cond);

    condrem.setInputCloud (output);
    condrem.setKeepOrganized(output->isOrganized());
    condrem.filter (*output);



    std::cout << "DONE " << output->size() << std::endl;
}

void Downsampler::DetectBRISKQuad(PointCloudNormalT::Ptr input, PointCloudNormalT::Ptr output, int paramThreshold, int octave)
{
    std::cout << "BRISK Quad Detector...";

    pcl::ScopeTime t("BRISK Quad Detector");

    // constructor
    pcl::BriskKeypoint2D<PointNormalT> brisk_keypoint_estimation;

    //output
    pcl::PointCloud<pcl::PointWithScale> brisk_keypoints_2D;

    //parameters
    brisk_keypoint_estimation.setThreshold(paramThreshold);
    brisk_keypoint_estimation.setOctaves(octave);
    brisk_keypoint_estimation.setInputCloud (input);

    //compute
    brisk_keypoint_estimation.compute (brisk_keypoints_2D);

    //convert pointwithscale to 3D
    output->resize(brisk_keypoints_2D.size());

    int k = brisk_keypoints_2D.size();
    for(int i = 0, j = 0; i < k; ++i)
    {

        int umin = floor(brisk_keypoints_2D.points[i].x);
        int vmin = floor(brisk_keypoints_2D.points[i].y);
        int umax = ceil(brisk_keypoints_2D.points[i].x);
        int vmax = ceil(brisk_keypoints_2D.points[i].y);
        double ures = brisk_keypoints_2D.points[i].x - floor(brisk_keypoints_2D.points[i].x);
        double vres = brisk_keypoints_2D.points[i].y - floor(brisk_keypoints_2D.points[i].y);

        PointNormalT TL = input->points[umin + vmax * input->width];
        PointNormalT TR = input->points[umax + vmax * input->width];
        PointNormalT BL = input->points[umin + vmin * input->width];
        PointNormalT BR = input->points[umax + vmin * input->width];

        double wTL =    ures +(1-vres);
        double wTR = (1-ures)+(1-vres);
        double wBL =    ures +   vres;
        double wBR = (1-ures)+   vres;


        if(isnan(TL.x) || isnan(TR.x) || isnan(BL.x) || isnan(BR.x))
        {
            --k;
        }
        else
        {
            output->points[i].b = (wTL*TL.b + wTR*TR.b + wBL*BL.b + wBR*BR.b) / (wTL+wTR+wBL+wBR);
            output->points[i].g = (wTL*TL.g + wTR*TR.g + wBL*BL.g + wBR*BR.g) / (wTL+wTR+wBL+wBR);
            output->points[i].r = (wTL*TL.r + wTR*TR.r + wBL*BL.r + wBR*BR.r) / (wTL+wTR+wBL+wBR);

            output->points[i].x = (wTL*TL.x + wTR*TR.x + wBL*BL.x + wBR*BR.x) / (wTL+wTR+wBL+wBR);
            output->points[i].y = (wTL*TL.y + wTR*TR.y + wBL*BL.y + wBR*BR.y) / (wTL+wTR+wBL+wBR);
            output->points[i].z = (wTL*TL.z + wTR*TR.z + wBL*BL.z + wBR*BR.z) / (wTL+wTR+wBL+wBR);
        }
    }

    std::vector<PointNormalT,Eigen::aligned_allocator<PointNormalT> >::iterator    keypointIt=output->begin();

    for(size_t i=k; k<brisk_keypoints_2D.size(); ++k)
        output->erase(keypointIt+i);



    pcl::PointIndices::Ptr indices (new pcl::PointIndices);

    // Remove NAN from keypoints cloud
    pcl::removeNaNFromPointCloud(*output, *output,indices->indices);
    pcl::removeNaNNormalsFromPointCloud(*output, *output,indices->indices);



    // remove 0 points
    pcl::ConditionAnd<PointNormalT>::Ptr range_cond (new pcl::ConditionAnd<PointNormalT> ());
    range_cond->addComparison (pcl::FieldComparison<PointNormalT>::ConstPtr (new pcl::FieldComparison<PointNormalT> ("z", pcl::ComparisonOps::GT, 0.1)));

    // build the filter
    pcl::ConditionalRemoval<PointNormalT> condrem(true);
    condrem.setCondition(range_cond);

    condrem.setInputCloud (output);
    condrem.setKeepOrganized(output->isOrganized());
    condrem.filter (*output);



    std::cout << "DONE " << output->size() << std::endl;
}



void Downsampler::DetectAGAST(PointCloudNormalT::Ptr input, PointCloudNormalT::Ptr output, int paramThreshold)
{
    pcl::ScopeTime t("AGAST Detector");
    std::cout << "AGAST Detector...";

    // constructor
    pcl::AgastKeypoint2D<PointNormalT> agast_keypoint_estimation;

    //output
    pcl::PointCloud<pcl::PointUV> agast_keypoints_2D;

    //parameters
    agast_keypoint_estimation.setThreshold (paramThreshold);
    agast_keypoint_estimation.setInputCloud (input);

    //compute
    agast_keypoint_estimation.compute (agast_keypoints_2D);

    //convert UV to 3D
    output->resize(agast_keypoints_2D.size());
    int k = agast_keypoints_2D.size();
    for(int i = 0, j = 0; i < k; ++i)
    {
        j = agast_keypoints_2D.points[i].u + agast_keypoints_2D.points[i].v * input->width;

        if(isnan(input->points[j].x))
        {
            --k;
        }
        else
        {
            output->points[i].b=input->points[j].b;
            output->points[i].g=input->points[j].g;
            output->points[i].r=input->points[j].r;
            output->points[i].x=input->points[j].x;
            output->points[i].y=input->points[j].y;
            output->points[i].z=input->points[j].z;
        }
    }

    std::vector<PointNormalT,Eigen::aligned_allocator<PointNormalT> >::iterator    keypointIt=output->begin();

    for(int i=k; k<agast_keypoints_2D.size(); ++k)
        output->erase(keypointIt+i);



    pcl::PointIndices::Ptr indices (new pcl::PointIndices);

    // Remove NAN from keypoints cloud
    pcl::removeNaNFromPointCloud(*output, *output,indices->indices);
    pcl::removeNaNNormalsFromPointCloud(*output, *output,indices->indices);



    // remove 0 points
    pcl::ConditionAnd<PointNormalT>::Ptr range_cond (new pcl::ConditionAnd<PointNormalT> ());
    range_cond->addComparison (pcl::FieldComparison<PointNormalT>::ConstPtr (new pcl::FieldComparison<PointNormalT> ("z", pcl::ComparisonOps::GT, 0.1)));

    // build the filter
    pcl::ConditionalRemoval<PointNormalT> condrem(true);
    condrem.setCondition(range_cond);

    condrem.setInputCloud (output);
    condrem.setKeepOrganized(output->isOrganized());
    condrem.filter (*output);


    std::cout << "DONE " << output->size() << std::endl;
}

void Downsampler::DetectSIFT(PointCloudNormalT::Ptr input, PointCloudNormalT::Ptr output)
{
    pcl::ScopeTime t("SIFT Detector");
    std::cout << "SIFT Detector...";

    pcl::SIFTKeypoint<pcl::PointXYZRGB, pcl::PointXYZI> sift_detect;

    const float min_scale = 0.005f;
    const int nr_octaves = 6;
    const int nr_scales_per_octave = 4;
    const float min_contrast = 0.005f;


    sift_detect.setSearchMethod(pcl::search::KdTree<pcl::PointXYZRGB>::Ptr (new pcl::search::KdTree<pcl::PointXYZRGB>));
    sift_detect.setScales (min_scale, nr_octaves, nr_scales_per_octave);
    sift_detect.setMinimumContrast (min_contrast);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr inputXYZRGB(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::copyPointCloud(*input, *inputXYZRGB);
    sift_detect.setInputCloud (inputXYZRGB);

    pcl::PointCloud<pcl::PointXYZI> keypoints_temp;
    sift_detect.compute (keypoints_temp);

    //OPT1
    // pcl::PointIndices::Ptr ind (new pcl::PointIndices ());
    //OPT2
    // std::vector<int> ind2;
    // ind2 = sift_detect.getKeypointsIndices();
    //
    // pcl::ExtractIndices<PointNormalT> extract;
    //extract.setInputCloud (input);
    // extract.setIndices (ind);
    //  extract.setNegative (false);
    //  extract.filter (*output);


    output->resize(keypoints_temp.size());
    pcl::copyPointCloud (keypoints_temp, *output);




    pcl::PointIndices::Ptr indices (new pcl::PointIndices);

    // Remove NAN from keypoints cloud
    pcl::removeNaNFromPointCloud(*output, *output,indices->indices);
    pcl::removeNaNNormalsFromPointCloud(*output, *output,indices->indices);



    // remove 0 points
    pcl::ConditionAnd<PointNormalT>::Ptr range_cond (new pcl::ConditionAnd<PointNormalT> ());
    range_cond->addComparison (pcl::FieldComparison<PointNormalT>::ConstPtr (new pcl::FieldComparison<PointNormalT> ("z", pcl::ComparisonOps::GT, 0.1)));

    // build the filter
    pcl::ConditionalRemoval<PointNormalT> condrem(true);
    condrem.setCondition(range_cond);

    condrem.setInputCloud (output);
    condrem.setKeepOrganized(output->isOrganized());
    condrem.filter (*output);



    std::cout << "DONE " << output->size() << std::endl;
    return ;
}

