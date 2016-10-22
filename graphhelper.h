#ifndef G2O_QGL_HELPER_H
#define G2O_QGL_HELPER_H
#include <QGLViewer/qglviewer.h>
#include <g2o/types/slam3d/types_slam3d.h>
#include "postypes.h"
namespace GraphHelper{
g2o::VertexSE3* createVertexSE3(const int& id, const PosTypes::Pose3D& p);

g2o::VertexSE3* createVertexSE3(const int& id, const g2o::VertexSE3* v_prev,
                                const PosTypes::Pose3D& delta);

g2o::EdgeSE3* createEdgeSE3FromVertices(const int& id, g2o::VertexSE3* vi, g2o::VertexSE3* vj,
                                        const g2o::EdgeSE3::InformationType& info_mat);

g2o::EdgeSE3* createEdgeSE3(const int& id, g2o::VertexSE3* vi, g2o::VertexSE3* vj,
                            const PosTypes::Pose3D& delta,
                            const g2o::EdgeSE3::InformationType& info_mat);

template <typename RealType>
void convertSE3To4x4Matrix(const g2o::VertexSE3::EstimateType &in, RealType *out);

void convertSE3ToqglviewerFrame(const g2o::VertexSE3::EstimateType &in, qglviewer::Frame &out);

template <typename RealType>
void convertEdgeSE3To4X4Matrix(const g2o::EdgeSE3::Measurement& measurement, RealType* out);

void convertEdgeSE3ToqglviewerFrame(const g2o::EdgeSE3::Measurement &in, qglviewer::Frame &out);

void setInformationMatrixDiagonal(g2o::EdgeSE3::InformationType& info, const double& v);
};


namespace GraphHelper {
template <typename RealType>
void convertSE3To4x4Matrix(const g2o::VertexSE3::EstimateType &in, RealType *out)
{
    g2o::VertexSE3::EstimateType::MatrixType vm = in.matrix();
    for(size_t irow = 0;irow<4;irow++){
        for(size_t icol=0;icol<4;icol++){
            out[4*icol+irow] = vm(irow, icol);
        }
    }
}

template <typename RealType>
void convertEdgeSE3To4X4Matrix(const g2o::EdgeSE3::Measurement& measurement, RealType* out){
//    g2o::VertexSE3::EstimateType::MatrixType vm = measurement.
    Eigen::Isometry3d::MatrixType m = measurement.matrix();
    for(size_t irow = 0;irow<4;irow++){
        for(size_t icol=0;icol<4;icol++){
            out[4*icol+irow] = m(irow, icol);
        }
    }
}
}
#endif // G2O_QGL_HELPER_H
