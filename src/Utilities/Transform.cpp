#include "Transform.h"

Transform::Transform()
{
    matrix = Eigen::Matrix4f::Identity();
}

Transform::Transform(QVector3D translation, QVector3D eulerangles)
{
    matrix = Eigen::Matrix4f::Identity();
    setOriginFromQVector3D(translation);
    setEulerAnglesFromQVector3D(eulerangles);
}

Transform::Transform(Eigen::Vector3f translation, Eigen::Quaternionf quaternion)
{
    matrix = Eigen::Matrix4f::Identity();
    setOrigin3(translation);
    setQuaternion(quaternion);
}

Transform::Transform(Eigen::Vector4f translation, Eigen::Quaternionf quaternion)
{
    matrix = Eigen::Matrix4f::Identity();
    setOrigin4(translation);
    setQuaternion(quaternion);
}

Transform::Transform(float Tx,float Ty,float Tz,float Rx,float Ry,float Rz)
{
    matrix = Eigen::Matrix4f::Identity();
    setOrigin3(Eigen::Vector3f(Tx, Ty, Tz));
    setEulerAngles(Eigen::Vector3f(Rx, Ry, Rz));
}

Transform::Transform(QString str)
{
    matrix = Eigen::Matrix4f::Identity();
    fromPrettyPrint(str);
}
Transform::Transform(Eigen::Matrix4f mat4)
{
    matrix = mat4;
}

Transform::~Transform()
{

}


QString Transform::prettyprint()
{
    Eigen::Vector3f trans = getOrigin3();
    Eigen::Vector3f euler = getEulerAngles();
    return QString("%1_%2_%3_%4_%5_%6")
            .arg(trans(0)).arg(trans(1)).arg(trans(2))
            .arg(euler(0)).arg(euler(1)).arg(euler(2));
}

void Transform::fromPrettyPrint(QString str)
{
    matrix = Eigen::Matrix4f::Identity();
    QStringList strlst = str.split("_");
    if (strlst.length() < 6) return;
    setOrigin3(Eigen::Vector3f(strlst.at(0).toFloat(), strlst.at(1).toFloat(), strlst.at(2).toFloat()));
    setEulerAngles(Eigen::Vector3f(strlst.at(3).toFloat(), strlst.at(4).toFloat(), strlst.at(5).toFloat()));
}


