#ifndef CustomPointCloud_H
#define CustomPointCloud_H

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <QDebug>
#include "define.h"

class CustomPointCloud //: public PointCloudT::Ptr
{
public:
    CustomPointCloud() :
        isVisible (true),
        isNormalVisible (false),
        pointSize (1),
        opacity (1)
    { PC.reset(new PointCloudT());}


    CustomPointCloud(PointCloudT::Ptr newPC) :
        isVisible (true),
        isNormalVisible (false),
        pointSize (1),
        opacity (1)
    { PC = newPC;}

    PointCloudT::Ptr PC;

    bool isOrganized() {return PC->isOrganized();}
    bool isDense() {return PC->is_dense;}
    bool size() {return PC->size();}
    bool height() {return PC->height;}
    bool width() {return PC->width;}
    pcl::PCLHeader header() {return PC->header;}


    void setVisibile(bool value) {isVisible = value;}
    void setNormalVisibile(bool value) {isNormalVisible = value;}
   // void setPointSize(int value) {pointSize = value;}
    void setOpacity(int value) {opacity = value;}

    bool getVisibile( ) {return isVisible;}
    bool getNormalVisibile() {return isNormalVisible;}
    int getPointSize( ) {return pointSize;}
    double getOpacity( ) {return opacity;}


    bool isVisible;
    bool isNormalVisible;
    int const pointSize;
    double opacity;
};

//


#endif // CustomPointCloud_H
