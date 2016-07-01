#ifndef Transform_H_
#define Transform_H_

#include <QVector3D>

#include <Eigen/Dense>

#include "define.h"

class Transform
{
public:
    Transform();
    Transform(QVector3D translation, QVector3D eulerangles);
    Transform(Eigen::Vector3f translation, Eigen::Quaternionf quaternion);
    Transform(Eigen::Vector4f translation, Eigen::Quaternionf quaternion);
    Transform(float Tx,float Ty,float Tz,float Rx,float Ry,float Rz);
    virtual ~Transform();


    Eigen::Vector4f getOrigin4(){return matrix4().block<4,1>(0,3);}
    Eigen::Vector3f getOrigin3(){return matrix4().block<3,1>(0,3);}
    Eigen::Quaternionf getQuaternion(){return Eigen::Quaternionf(matrix3());}
    Eigen::Vector3f getEulerAngles(){return matrix3().eulerAngles(0,1,2);}

    QVector3D getEulerAnglesQVector3D(){Eigen::Vector3f vect = getEulerAngles(); return QVector3D(vect(0), vect(1), vect(2));}
    QVector3D getOriginQVector3D(){Eigen::Vector3f vect = getOrigin3(); return QVector3D(vect(0), vect(1), vect(2));}


    void setOrigin4(Eigen::Vector4f orig){_transformation.matrix().block(0,3,4,1) = orig;}
    void setOrigin3(Eigen::Vector3f orig){_transformation.matrix().block(0,3,3,1) = orig;}
    void setQuaternion(Eigen::Quaternionf quat){_transformation.matrix().block(0,0,3,3) = quat.matrix();}
    void setEulerAngles(Eigen::Vector3f ea){_transformation.matrix().block(0,0,3,3) = (Eigen::AngleAxisf(ea[0], Eigen::Vector3f::UnitX()) * Eigen::AngleAxisf(ea[1], Eigen::Vector3f::UnitY()) * Eigen::AngleAxisf(ea[2], Eigen::Vector3f::UnitZ())).matrix();}
    void setMatrix3(Eigen::Matrix3f mat){_transformation.matrix().block(0,0,3,3) = mat;}

    void setOriginFromQVector3D(QVector3D vect){setOrigin3(Eigen::Vector3f(vect.x(),vect.y(), vect.z()));}
    void setEulerAnglesFromQVector3D(QVector3D vect){setEulerAngles(Eigen::Vector3f(vect.x(),vect.y(), vect.z()));}


    void print(){ std::cout << matrix4() << std::endl; }

private:
    Eigen::Transform<float,3,Eigen::Affine> _transformation;

    Eigen::Matrix4f matrix4(){return _transformation.matrix();}
    Eigen::Matrix3f matrix3(){return matrix4().block<3,3>(0,0);}

};

#endif /* Transform_H_ */
