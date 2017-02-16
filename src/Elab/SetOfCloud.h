#ifndef SetOfCloud_H_
#define SetOfCloud_H_

#include <QVector3D>

#include <Eigen/Dense>

#include <pcl/common/transforms.h>

#include "define.h"
#include <QList>
#include <QStringList>

class SetOfCloud
{
public:
    SetOfCloud();
    virtual ~SetOfCloud();




    PointCloudNormalT::Ptr getPointCloud(int i);
    void setPointCloud(int i, PointCloudNormalT::Ptr PC);
    PointCloudNormalT::Ptr getMergedPointCloud();


private:
    QList<PointCloudNormalT::Ptr> my4PC;
};

#endif /* SetOfCloud_H_ */
