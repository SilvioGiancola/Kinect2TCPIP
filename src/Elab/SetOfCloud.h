#ifndef SetOfCloud_H_
#define SetOfCloud_H_

#include <QVector3D>

#include <Eigen/Dense>

#include <pcl/common/transforms.h>
#include <pcl/filters/filter.h>

#include "define.h"
#include <QList>
#include <QStringList>

class SetOfCloud
{
public:
    SetOfCloud();
    virtual ~SetOfCloud();




    PointCloudNormalT::Ptr getPointCloud(int i);
    PointCloudNormalT::Ptr getPointCloud(QString ID);
    void setPointCloud(int i, PointCloudNormalT::Ptr PC);
    void setPointCloud(QString ID, PointCloudNormalT::Ptr PC);
    PointCloudNormalT::Ptr getMergedPointCloud();

    void setIDlist(QStringList IDlist);


private:
    QList<PointCloudNormalT::Ptr> my4PC;

    QStringList IDlist;
};

#endif /* SetOfCloud_H_ */
