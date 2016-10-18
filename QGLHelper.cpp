#include "QGLHelper.h"


using namespace qglviewer;

namespace QGLHelper {
qglviewer::Frame getLocalTransformation(const qglviewer::Frame& from, const qglviewer::Frame& to){
    Frame ret = from.inverse();
    Quaternion q = ret.rotation();
    ret.rotate(to.rotation());
    ret.translate(q.rotate(to.translation()));
    return ret;
}

PosTypes::Pose3D toPosTypesPose3D(const Frame &frame)
{
    Vec v = frame.position();
    Quaternion q = frame.orientation();
    return PosTypes::Pose3D(v[0], v[1], v[2], q[0], q[1], q[2], q[3]);
}

Frame concatenateTransforms(const Frame &T1, const Frame &T2)
{
    libicp::Matrix m1(4,4), m2(4,4);
    qglFrameToLibicpMatrix(T1, m1);
    qglFrameToLibicpMatrix(T2, m2);
    libicp::Matrix m12(4,4);
    m12 = m1* m2;
    Frame T12;
    std::cerr<<m12<<std::endl;
    libicpMatrixToqglFrame(m12.getMat(0,0,2,2),m12.getMat(0,3,2,3), T12);
    return T12;
}

void qglFrameToLibicpMatrix(const qglviewer::Frame& frame, libicp::Matrix& R, libicp::Matrix& t){
    double r[3][3];
    frame.orientation().getRotationMatrix(r);
    for(size_t icol=0;icol<3;icol++){
        for(size_t irow=0;irow<3;irow++){
            R(irow, icol) = r[irow][icol];
        }
    }
    const qglviewer::Vec v = frame.position();
    t(0,0) = v[0];
    t(1,0) = v[1];
    t(2,0) = v[2];
}

void qglFrameToLibicpMatrix(const qglviewer::Frame& frame, libicp::Matrix& T){
    GLdouble m[4][4];
    frame.getMatrix(m);
    for(size_t icol=0;icol<4;icol++){
        for(size_t irow=0;irow<4;irow++){
            T(irow, icol) = m[icol][irow];
        }
    }
}

void libicpMatrixToqglFrame( const libicp::Matrix& R, const libicp::Matrix& t, qglviewer::Frame& frame){
    libicp::FLOAT qx, qy, qz, qw;
    libicp::toQuaternion(R,qx,qy,qz,qw);
    frame.setOrientation(qx, qy, qz, qw);
    frame.setPosition(t(0,0), t(1,0), t(2,0));
}


}
