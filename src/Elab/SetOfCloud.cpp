#include "SetOfCloud.h"

SetOfCloud::SetOfCloud()
{
    PointCloudNormalT::Ptr PC1(new PointCloudNormalT);
    my4PC.append(PC1);
    PointCloudNormalT::Ptr PC2(new PointCloudNormalT);
    my4PC.append(PC2);
    PointCloudNormalT::Ptr PC3(new PointCloudNormalT);
    my4PC.append(PC3);
    PointCloudNormalT::Ptr PC4(new PointCloudNormalT);
    my4PC.append(PC4);
}


SetOfCloud::~SetOfCloud()
{

}

PointCloudNormalT::Ptr SetOfCloud::getPointCloud(int i)
{
    return my4PC.at(i);
}

void SetOfCloud::setPointCloud(int i, PointCloudNormalT::Ptr PC)
{
    my4PC.replace(i,PC);
}


PointCloudNormalT::Ptr SetOfCloud::getMergedPointCloud()
{
   /* PointCloudNormalT::Ptr ConcatenatedPC (new PointCloudNormalT());

    ConcatenatedPC->header.frame_id = QString("%1/PointClouds/merged.pcd").arg(QDir::homePath()).toStdString();


    for (int i = 0; i < _PCList.size() ; i++)

    addPointCloud(ConcatenatedPC);

*/



    PointCloudNormalT::Ptr ConcatenatedPC ( new PointCloudNormalT);
    for (int i = 0; i < 4; i++)
    {
        PointCloudNormalT::Ptr temp(new PointCloudNormalT());

        Eigen::Matrix4f trans = Eigen::Matrix4f::Identity();
        trans.block(0,0,3,3) = getPointCloud(i)->sensor_orientation_.matrix();
        trans.block(0,3,4,1) = getPointCloud(i)->sensor_origin_;

        pcl::transformPointCloudWithNormals(*getPointCloud(i), *temp, trans);

        *ConcatenatedPC += *temp;

    }
       // *mergedPC += *getPointCloud(i);

    return ConcatenatedPC;
}
