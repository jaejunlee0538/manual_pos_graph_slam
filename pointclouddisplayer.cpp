#include "pointclouddisplayer.h"
#include <GL/glx.h>
#include <GL/gl.h>
#include <GL/glu.h>
#include <QGLViewer/qglviewer.h>
PointCloudDisplayer::PointCloudDisplayer()
    :selectable(true)
{
    start = 0;
}

void PointCloudDisplayer::drawPointCloud(const bool &selected,
                                         const GLfloat &pointSize,
                                         const GLdouble &alpha)
{
    glEnable(GL_BLEND);
    glDisable(GL_LIGHTING);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    size_t n_points = this->data.size() / fields.size();
    if(n_points == 0){
        return;
    }
    glPushMatrix();
    glMultMatrixd(this->frame.matrix());
    glPointSize(pointSize);

    glBegin(GL_POINTS);
    CloudDataType::value_type* p = &this->data[0]+start;
    size_t n_fields = fields.size();

    if(selected){
        for(size_t i=0;i<n_points;i++,p+=n_fields){
            GLdouble gray = p[3] / 255.0;
//            glColor3d(cos(gray*1.57079630), gray, gray);
            glColor4d(cos(gray*1.57079630), gray, gray, alpha);
            glVertex3fv(p);
        }
    }
    else{
        for(size_t i=0;i<n_points;i++,p+=n_fields){
            GLdouble gray = p[3] / 255.0;
            glColor4d(gray, gray, gray,alpha);
//            glColor3d(gray, gray, gray);
            glVertex3fv(p);
        }
    }
    glEnd();
    glPopMatrix();
    glEnable(GL_LIGHTING);
    glDisable(GL_BLEND);
}

void PointCloudDisplayer::drawFrame(const bool &selected)
{
#define SCALE 0.3
    const static GLdouble triangles[9] = {
        SCALE*1.0, SCALE*0.0, SCALE*0.0,
        SCALE*-0.3, SCALE*0.5, SCALE*0.0,
        SCALE*-0.3, SCALE*-0.5, SCALE*0.0
    };
    glPushMatrix();
    glMultMatrixd(frame.matrix());
    glBegin(GL_TRIANGLES);
    if(selected)
        glColor3f(1.0f, 0.0f, 0.6f);
    else
        glColor3f(0.2f, 0.2f, 1.0f);
    glNormal3dv(&frame.matrix()[8]);
    glVertex3dv(&triangles[0]);
    glVertex3dv(&triangles[3]);
    glVertex3dv(&triangles[6]);
    glEnd();
    glPopMatrix();
}


void PointCloudDisplayer::copyXYZTo(std::vector<double> &output){
    size_t n = nPoints(), nf = fields.size();

    output.reserve(n*3);
    for(size_t i=0;i<n;i++){
        size_t idx = i * nf;
        output.push_back(data[idx]);
        output.push_back(data[idx+1]);
        output.push_back(data[idx+2]);
    }
}

void PointCloudDisplayer::pushBack(const PointCloudDisplayer::CloudDataType &p)
{
    if(p.size() != fields.size()){
        throw std::runtime_error("point size does not match with number of fields.[PointCloudDisplayer::pushBack]");
    }
    for(size_t i=0;i<fields.size();i++){
        data.push_back(p[i]);
    }
}

PointCloudDisplayer::CloudDataType PointCloudDisplayer::getPoint(size_t i) const
{
    CloudDataType p;
    p.resize(fields.size());
    size_t idx = fields.size()*i;
    for(size_t i=0;i<fields.size();i++){
        p[i] = data[idx+i];
    }
    return p;
}

size_t PointCloudDisplayer::nPoints() const{
    return data.size() / fields.size();
}
