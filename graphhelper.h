#ifndef G2O_QGL_HELPER_H
#define G2O_QGL_HELPER_H
#include "Global.h"
#include <QGLViewer/qglviewer.h>
#include <g2o/types/slam3d/types_slam3d.h>
#include <g2o/types/slam2d/types_slam2d.h>
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

g2o::EdgeSE2 *createEdgeSE2(const int& id, g2o::VertexSE2* vi, g2o::VertexSE2* vj,
                            const PosTypes::Pose2D& delta,
                            const g2o::EdgeSE2::InformationType& info_mat);



template <typename RealType>
void convertSE3To4x4Matrix(const g2o::VertexSE3::EstimateType &in, RealType *out);

template <typename RealType>
void convertSE2To4x4Matrix(const g2o::VertexSE2::EstimateType &in, RealType *out);

void convertSE3ToqglviewerFrame(const g2o::VertexSE3::EstimateType &in, qglviewer::Frame &out);
void convertSE2ToqglviewerFrame(const g2o::VertexSE2::EstimateType &in, qglviewer::Frame &out);
void convertEdgeSE3ToqglviewerFrame(const g2o::EdgeSE3::Measurement &in, qglviewer::Frame &out);
void convertEdgeSE2ToqglviewerFrame(const g2o::EdgeSE2::Measurement &in, qglviewer::Frame &out);

template <typename RealType>
void convertEdgeSE3To4X4Matrix(const g2o::EdgeSE3::Measurement& measurement, RealType* out);

template <typename RealType>
void convertEdgeSE2To4X4Matrix(const g2o::EdgeSE2::Measurement& in, RealType* out);

g2o::VertexSE2::EstimateType getDifference(const g2o::VertexSE2::EstimateType& from, const g2o::VertexSE2::EstimateType& to);
void setInformationMatrixDiagonal(g2o::EdgeSE3::InformationType& info, const double& v);
void getInformationMatrix(const g2o::OptimizableGraph::Edge* e, GeneralMatrixType& m);
};


namespace GraphHelper {
template <typename RealType>
void convertSE3To4x4Matrix(const g2o::VertexSE3::EstimateType &in, RealType *out)
{
    g2o::VertexSE3::EstimateType::MatrixType vm = in.matrix();
    //copy matrix data into an array(column major)
    for(size_t irow = 0;irow<4;irow++){
        for(size_t icol=0;icol<4;icol++){
            out[4*icol+irow] = vm(irow, icol);
        }
    }
}

template <typename RealType>
void convertSE2To4x4Matrix(const g2o::VertexSE2::EstimateType &in, RealType *out){
    double c=cos(in[2]),s=sin(in[2]);
    memset((void*)out, 0 ,sizeof(RealType)*16);
    out[4*0+0] = c;
    out[4*0+1] = s;
    out[4*1+0] = -s;
    out[4*1+1] = c;
    out[4*2+2] = 1.0;//out[2][2]
    out[4*3+0] = in[0]; //x
    out[4*3+1] = in[1];//y
    out[4*3+3] = 1.0;//out[3][3]
}

template <typename RealType>
void convertEdgeSE2To4X4Matrix(const g2o::EdgeSE2::Measurement& in, RealType* out){
    double c=cos(in[2]),s=sin(in[2]);
    memset((void*)out, 0 ,sizeof(RealType)*16);
    out[4*0+0] = c;
    out[4*0+1] = s;
    out[4*1+0] = -s;
    out[4*1+1] = c;
    out[4*2+2] = 1.0;//out[2][2]
    out[4*3+0] = in[0]; //x
    out[4*3+1] = in[1];//y
    out[4*3+3] = 1.0;//out[3][3]
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
