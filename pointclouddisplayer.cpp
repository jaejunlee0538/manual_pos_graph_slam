#include "pointclouddisplayer.h"



PointCloudDisplayer::PointCloudDisplayer()
{
    start = 0;
}

void PointCloudDisplayer::drawPointCloud(const bool &selected,
                                         const double &pointSize,
                                         const double &alpha)
{
    size_t n_points = this->data.size() / fields.size();
    if(n_points == 0){
        return;
    }
    glPushMatrix();
    glMultMatrixd(this->frame.matrix());

    glBegin(GL_POINTS);
    if(selected)
        glColor3f(0.1f, 1.0f, 0.1f);
    else
        glColor3f(1.0f, 1.0f, 1.0f);

    CloudDataType::value_type* p = &this->data[0]+start;
    size_t n_fields = fields.size();

    for(size_t i=0;i<n_points;i++,p+=n_fields){
        glVertex3fv(p);
    }
    glEnd();
    glPopMatrix();
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
