#ifndef GRAPHDISPLAYER_H
#define GRAPHDISPLAYER_H
#include <memory>
#include <vector>
#include <postypes.h>
#include <Eigen/Dense>
#include <QGLViewer/qglviewer.h>
namespace g2o{
class SparseOptimizer;
}
class GraphSLAM;
class GraphDisplayer
{
public:
    struct t_EdgeDisplayType{
        int vi, vj;
        t_EdgeDisplayType(){}

        t_EdgeDisplayType(int vi, int vj):vi(vi), vj(vj){}
    };
    struct t_VertexDisplayType{
        GLdouble m[16];//tranformation matrix for 3D
        t_VertexDisplayType(){}

        t_VertexDisplayType(double* data){
            for(size_t i=0;i<16;i++){
                m[i] = data[i];
            }
        }
    };

    typedef std::shared_ptr<GraphDisplayer> Ptr;
    typedef const std::shared_ptr<const GraphDisplayer> ConstPtr;
    typedef t_VertexDisplayType VertexDisplayType;
    typedef t_EdgeDisplayType EdgeDisplayType;
public:
    GraphDisplayer();

    void drawVertices();
    void drawLoopEdges();
    void drawMotionEdges();
    void reset();
    const QVector<EdgeDisplayType>& loop_edges()const {return m_loop_edges;}
protected:
    QVector<VertexDisplayType> m_vertices;
    QVector<EdgeDisplayType> m_loop_edges;
    QVector<EdgeDisplayType> m_motion_edges;

    friend class GraphSLAM;
};

#endif // GRAPHDISPLAYER_H
