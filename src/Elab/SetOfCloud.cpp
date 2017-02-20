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
    IDlist.append("006670253647");
    IDlist.append("006662153647");
    IDlist.append("500875340242");
    IDlist.append("507040542542");
}


SetOfCloud::~SetOfCloud()
{

}


// Handle PC (get/set with index/ID)
PointCloudNormalT::Ptr SetOfCloud::getPointCloud(int i)
{
    PointCloudNormalT::Ptr resPC ( new PointCloudNormalT);
    if (i >= 0 && i < 4)
        resPC = my4PC.at(i);
    return resPC;
}

PointCloudNormalT::Ptr SetOfCloud::getPointCloud(QString ID)
{
    PointCloudNormalT::Ptr resPC ( new PointCloudNormalT);
    int index = IDlist.lastIndexOf(ID);
    resPC = getPointCloud(index);
    return resPC;
}

void SetOfCloud::setPointCloud(int i, PointCloudNormalT::Ptr PC)
{
    if (i >= 0 && i < 4)
        my4PC.replace(i,PC);
}

void SetOfCloud::setPointCloud(QString ID, PointCloudNormalT::Ptr PC)
{
    int index = IDlist.lastIndexOf(ID);
    setPointCloud(index, PC);
}




void SetOfCloud::setIDlist(QStringList IDlist)
{
    this->IDlist = IDlist;
}


PointCloudNormalT::Ptr SetOfCloud::getMergedPointCloud()
{
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

    std::vector<int> ind;
    pcl::removeNaNFromPointCloud(*ConcatenatedPC, *ConcatenatedPC, ind);
    // pcl::removeNaNNormalsFromPointCloud(*ConcatenatedPC, *ConcatenatedPC, ind);

    return ConcatenatedPC;
}
