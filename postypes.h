#ifndef POSTYPES_H
#define POSTYPES_H
#include <vector>

namespace PosTypes{
struct t_XYZ{
    double xyz[3];
    double x,y,z;
};

struct t_Quat{
    double q[4];
    double x,y,z,w;
};

struct t_XY{
    double xy[2];
    double x,y;
};

struct Pose2D{
    Pose2D(const double& x, const double& y, const double& heading){
        xy.x = x;
        xy.y = y;
        this->heading = heading;
    }

    double heading;
    t_XY xy;
};

struct Pose3D{
    Pose3D(){
        quat.x =quat.y=quat.z= 0.0;
        quat.z = 1.0;
        xyz.x = xyz.y = xyz.z = 0.0;
    }

    Pose3D(const double& x, const double& y, const double& z,
           const double& qx, const double& qy, const double& qz, const double& qw ){
        xyz.x = x;
        xyz.y = y;
        xyz.z = z;
        quat.x = qx;
        quat.y = qy;
        quat.z = qz;
        quat.w = qw;
    }

    Pose3D(const double* data7d):
        Pose3D(data7d[0], data7d[1], data7d[2], data7d[3], data7d[4], data7d[5], data7d[6]){
    }

    void copyTo(double* data) const{
        data[0] = xyz.x;
        data[1] = xyz.y;
        data[2] = xyz.z;
        data[3] = quat.x;
        data[4] = quat.y;
        data[5] = quat.z;
        data[6] = quat.w;
    }

    t_Quat quat;
    t_XYZ xyz;
};

}

#endif // POSTYPES_H
