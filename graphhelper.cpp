#include "graphhelper.h"

namespace GraphHelper{
g2o::VertexSE3 *createVertexSE3(const int &id, const PosTypes::Pose3D &p){
    g2o::VertexSE3 *v = new g2o::VertexSE3();
    double data[7];
    p.copyTo(data);
    v->setEstimateData(data);
    v->setId(id);
    return v;
}


g2o::VertexSE3 *createVertexSE3(const int &id, const g2o::VertexSE3 *v_prev, const PosTypes::Pose3D &delta){
    g2o::VertexSE3 *v = new g2o::VertexSE3();
    v->setEstimate(v_prev->estimate());
    double data[7];
    delta.copyTo(data);
    v->oplus(data); //actually the implementation of oplus requires 6 values(x,y,z, qx, qy, qz).
    v->setId(id);
    return v;
}

g2o::EdgeSE3 *createEdgeSE3FromVertices(const int& id, g2o::VertexSE3 *vi, g2o::VertexSE3 *vj,
                                        const g2o::EdgeSE3::InformationType &info_mat){
    g2o::EdgeSE3* e = new g2o::EdgeSE3();
    e->setId(id);
    e->setVertex(0, vi);
    e->setVertex(1, vj);
    e->setMeasurementFromState();
    e->setInformation(info_mat);
    return e;
}

g2o::EdgeSE3 *createEdgeSE3(const int& id, g2o::VertexSE3 *vi, g2o::VertexSE3 *vj,
                            const PosTypes::Pose3D &delta, const g2o::EdgeSE3::InformationType &info_mat){
    g2o::EdgeSE3* e = new g2o::EdgeSE3();
    e->setId(id);
    e->setVertex(0, vi);
    e->setVertex(1, vj);
    double data[7];
    delta.copyTo(data);
    e->setMeasurementData(data);
    e->setInformation(info_mat);
    return e;
}
g2o::EdgeSE2 *createEdgeSE2(const int& id, g2o::VertexSE2* vi, g2o::VertexSE2* vj,
                            const PosTypes::Pose2D& delta,
                            const g2o::EdgeSE2::InformationType& info_mat){
    g2o::EdgeSE2 * e=new g2o::EdgeSE2();
    e->setId(id);
    e->setVertex(0, vi);
    e->setVertex(1, vj);
    e->setMeasurement(g2o::EdgeSE2::Measurement(delta.xy.x, delta.xy.y, delta.heading));
    e->setInformation(info_mat);
    return e;
}

void convertSE3ToqglviewerFrame(const g2o::VertexSE3::EstimateType &in, qglviewer::Frame &out)
{
    static GLdouble m[16];
    convertSE3To4x4Matrix(in, m);
    out.setFromMatrix(m);
}

void convertSE2ToqglviewerFrame(const g2o::VertexSE2::EstimateType &in, qglviewer::Frame &out){
    static GLdouble m[16];
    convertSE2To4x4Matrix(in, m);
    out.setFromMatrix(m);
}


void setInformationMatrixDiagonal(g2o::EdgeSE3::InformationType& info, const double& v){
    info.setZero();
    for(int i=0;i<info.rows();i++){
        info(i,i) = v;
    }
}

void convertEdgeSE3ToqglviewerFrame(const g2o::EdgeSE3::Measurement &in, qglviewer::Frame &out)
{
    static GLdouble m[16];
    convertEdgeSE3To4X4Matrix(in, m);
    out.setFromMatrix(m);
}

void convertEdgeSE2ToqglviewerFrame(const g2o::EdgeSE2::Measurement &in, qglviewer::Frame &out){
    static GLdouble m[16];
    convertEdgeSE2To4X4Matrix(in, m);
    out.setFromMatrix(m);
}

void getInformationMatrix(const g2o::OptimizableGraph::Edge *e, GeneralMatrixType &m)
{
    int dim = e->dimension();
    m.resize(dim,dim);
    const double* data = e->informationData();

    //Assuming column major order
    for(int i=0;i<dim;i++){
        for(int j=0;j<dim;j++){
            m(j,i) = data[i*dim+j];
        }
    }
}

g2o::VertexSE2::EstimateType getDifference(const g2o::VertexSE2::EstimateType &from, const g2o::VertexSE2::EstimateType &to)
{
    return from.inverse() * to;
}

}
