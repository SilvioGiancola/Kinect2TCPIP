#include "Transform.h"

Transform::Transform()
{
    _transformation.matrix() = Eigen::Matrix4f::Identity();
}

Transform::Transform(QVector3D translation, QVector3D eulerangles)
{
    _transformation.matrix() = Eigen::Matrix4f::Identity();
    setOriginFromQVector3D(translation);
    setEulerAnglesFromQVector3D(eulerangles);
}

Transform::Transform(Eigen::Vector3f translation, Eigen::Quaternionf quaternion)
{
    _transformation.matrix() = Eigen::Matrix4f::Identity();
    setOrigin3(translation);
    setQuaternion(quaternion);
}

Transform::Transform(Eigen::Vector4f translation, Eigen::Quaternionf quaternion)
{
    _transformation.matrix() = Eigen::Matrix4f::Identity();
    setOrigin4(translation);
    setQuaternion(quaternion);
}

Transform::Transform(float Tx,float Ty,float Tz,float Rx,float Ry,float Rz)
{
    _transformation.matrix() = Eigen::Matrix4f::Identity();
    setOrigin3(Eigen::Vector3f(Tx, Ty, Tz));
    setEulerAngles(Eigen::Vector3f(Rx, Ry, Rz));
}

Transform::~Transform()
{

}


