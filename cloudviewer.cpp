#include "cloudviewer.h"
#include "manipulatedframesetconstraint.h"
#include <QMouseEvent>

void CloudViewer::init(){
    setManipulatedFrame(new qglviewer::ManipulatedFrame());
    manipulatedFrame()->setConstraint(new ManipulatedFrameSetConstraint());

    glBlendFunc(GL_ONE, GL_ONE);
    setGridIsDrawn();
    //    setAxisIsDrawn();
    setFPSIsDisplayed();
}

void CloudViewer::draw(){

    glMatrixMode(GL_MODELVIEW);
    if(graph){
        //        graph->drawVertices();
    }
    if(!clouds.empty()){
        double x,y,z;
        clouds[0]->frame.getTranslation(x,y,z);
        std::cerr<<"Hood "<<x<<", "<<y<<", "<<z<<std::endl;
        for(size_t i=0;i<clouds.size();i++){
            bool selected = selections.contains(i);
            clouds[i]->drawPointCloud(selected, 2.0, 0.8);
            clouds[i]->drawFrame(selected);
        }
    }
    if(selection_mode != NONE){
        drawSelectionRectangle();
        drawAxis(0.1);
    }
}

void CloudViewer::drawWithNames(){
    if(!clouds.empty()){
        for(size_t i=0;i<clouds.size();i++){
            glPushName(i);
            clouds[i]->drawFrame(false);
            glPopName();
        }
    }
}

void CloudViewer::endSelection(const QPoint &point)
{
    glFlush();

    GLint nb_hits = glRenderMode(GL_RENDER);
    if(nb_hits){
        for(int i=0;i<nb_hits;i++){
            switch(selection_mode){
            case ADD:
                addIDToSelections((selectBuffer())[4*i+3]);
                break;
            case REMOVE:
                removeIDFromSelections((selectBuffer())[4*i+3]);
                break;
            }
        }
    }
    selection_mode = NONE;
}

void CloudViewer::mousePressEvent(QMouseEvent *e)
{
    // Start selection. Mode is ADD with Shift key and TOGGLE with Alt key.
    rectangle = QRect(e->pos(), e->pos());
    if(e->button() == Qt::LeftButton){
        if(e->modifiers()&Qt::ShiftModifier){
            if(e->modifiers() & Qt::ControlModifier){
                selection_mode = REMOVE;
                return;
            }else{
                selection_mode = ADD;
                return;
            }
        }else{
            if(e->modifiers() == Qt::ControlModifier){
                startManipulation();
            }
        }
    }else{
    }
    QGLViewer::mousePressEvent(e);
}

void CloudViewer::mouseMoveEvent(QMouseEvent *e)
{
    if (selection_mode!= NONE)
    {
        // Updates rectangle coordinates and redraws rectangle
        rectangle.setBottomRight(e->pos());
        updateGL();
    }
    else
        QGLViewer::mouseMoveEvent(e);
}

void CloudViewer::mouseReleaseEvent(QMouseEvent *e)
{
    if (selection_mode != NONE)
    {
        // Actual selection on the rectangular area.
        // Possibly swap left/right and top/bottom to make rectangle valid.
        rectangle = rectangle.normalized();
        // Define selection window dimensions
        setSelectRegionWidth(rectangle.width());
        setSelectRegionHeight(rectangle.height());
        // Compute rectangle center and perform selection
        select(rectangle.center());
        // Update display to show new selected objects
        updateGL();
    }
    else
        QGLViewer::mouseReleaseEvent(e);
}

void CloudViewer::startManipulation()
{
    qglviewer::Vec control_point;
    ManipulatedFrameSetConstraint* mfsc = (ManipulatedFrameSetConstraint*)(manipulatedFrame()->constraint());
    mfsc->clear();

    for (QList<int>::const_iterator it=selections.begin(), end=selections.end(); it != end; ++it)
    {
        mfsc->add(&(clouds[*it]->frame));
        control_point += clouds[*it]->frame.position();
    }

    if (selections.size() > 0)
        manipulatedFrame()->setPosition(control_point / selections.size());
}

void CloudViewer::drawSelectionRectangle() const
{
    startScreenCoordinatesSystem();
    glDisable(GL_LIGHTING);
    glEnable(GL_BLEND);

    glColor4f(0.0, 0.0, 0.3f, 0.3f);
    glBegin(GL_QUADS);
    glVertex2i(rectangle.left(),  rectangle.top());
    glVertex2i(rectangle.right(), rectangle.top());
    glVertex2i(rectangle.right(), rectangle.bottom());
    glVertex2i(rectangle.left(),  rectangle.bottom());
    glEnd();

    glLineWidth(2.0);
    glColor4f(0.4f, 0.4f, 0.5f, 0.5f);
    glBegin(GL_LINE_LOOP);
    glVertex2i(rectangle.left(),  rectangle.top());
    glVertex2i(rectangle.right(), rectangle.top());
    glVertex2i(rectangle.right(), rectangle.bottom());
    glVertex2i(rectangle.left(),  rectangle.bottom());
    glEnd();

    glDisable(GL_BLEND);
    glEnable(GL_LIGHTING);
    stopScreenCoordinatesSystem();
}

void CloudViewer::addIDToSelections(int id)
{
    if(!selections.contains(id)){
        selections.push_back(id);
    }
}

void CloudViewer::removeIDFromSelections(int id)
{
    this->selections.removeAll(id);
}
