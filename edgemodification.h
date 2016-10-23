#ifndef EDGEMODIFICATION_H
#define EDGEMODIFICATION_H
#include "Global.h"
#include <memory>
#include <g2o/types/slam3d/types_slam3d.h>
#include <g2o/core/robust_kernel.h>
#include <g2o/types/slam2d/types_slam2d.h>
class EdgeModifyAction{
public:
    typedef std::shared_ptr<EdgeModifyAction> Ptr;
    EdgeModifyAction(const g2o::OptimizableGraph::Edge* edge):edge(edge){}
    virtual ~EdgeModifyAction(){}
    virtual void action(g2o::OptimizableGraph::Edge* pe) = 0;
    const g2o::OptimizableGraph::Edge* getEdge() const{return edge;}
protected:
    const g2o::OptimizableGraph::Edge* edge;
};

class EdgeModifyActionChangeInformation:public EdgeModifyAction{
public:
    EdgeModifyActionChangeInformation(const g2o::OptimizableGraph::Edge* edge, const GeneralMatrixType& matrix)
        :EdgeModifyAction(edge), info(matrix){        }
    virtual ~EdgeModifyActionChangeInformation(){}
    void action(g2o::OptimizableGraph::Edge *pe){
        g2o::EdgeSE2 * se2 = dynamic_cast<g2o::EdgeSE2*>(pe);
        if(se2){
            se2->setInformation(info);
            return;
        }
        g2o::EdgeSE3 * se3 = dynamic_cast<g2o::EdgeSE3*>(pe);
        if(se3){
            se3->setInformation(info);
            return;
        }
    }
protected:
    GeneralMatrixType info;
};

class EdgeModifySetRobustKernel:public EdgeModifyAction{
public:
    //nullptr when no robust kernel is used.
    EdgeModifySetRobustKernel(const g2o::OptimizableGraph::Edge* edge, g2o::RobustKernel* kernel)
        :EdgeModifyAction(edge), robust_kernel(kernel)    {    }

    void action(g2o::OptimizableGraph::Edge *pe){
        pe->setRobustKernel(robust_kernel);
    }
protected:
    g2o::RobustKernel * robust_kernel;
};

#endif // EDGEMODIFICATION_H
