#include "graphdisplayer.h"
#include <g2o/core/sparse_optimizer.h>
#include <g2o/types/slam3d/types_slam3d.h>


GraphDisplayer::GraphDisplayer()
{

}

void GraphDisplayer::drawVertices(){
#define SCALE 0.3
    const static GLdouble triangles[9] = {
        SCALE*1.0, SCALE*0.0, SCALE*0.0,
        SCALE*-0.3, SCALE*0.5, SCALE*0.0,
        SCALE*-0.3, SCALE*-0.5, SCALE*0.0
    };

    for(size_t i=0;i<m_vertices.size();i++){
        glPushMatrix();
        glMultMatrixd(m_vertices[i].m);
        glBegin(GL_TRIANGLES);
        glNormal3dv(&m_vertices[i].m[8]);
        //        glNormal3d(0,0,1);
        glVertex3dv(&triangles[0]);
        glVertex3dv(&triangles[3]);
        glVertex3dv(&triangles[6]);
        glEnd();
        glPopMatrix();
    }
}

void GraphDisplayer::drawMotionEdges(){
    glColor3d(0.0, 0.5, 1.0);
    glLineWidth(0.5);
    glBegin(GL_LINE_STRIP);
    for(size_t i=0;i<m_vertices.size();i++){
        glVertex3dv(&(m_vertices[i].m[12]));
    }
    glEnd();
}

void GraphDisplayer::drawLoopEdges(){
    if(m_loop_edges.empty()){
        return;
    }
    glColor3d(1.0, 0.3, 0.3);
    glLineWidth(2.0);
    glBegin(GL_LINES);
    for(size_t i=0;i<m_loop_edges.size();i++){
        glVertex3dv(&(m_vertices[m_loop_edges[i].vi].m[12]));
        glVertex3dv(&(m_vertices[m_loop_edges[i].vj].m[12]));
    }
    glEnd();
}

void GraphDisplayer::reset()
{
    m_loop_edges.clear();
    m_motion_edges.clear();
    m_vertices.clear();
}
