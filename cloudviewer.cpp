#include "cloudviewer.h"
#include "manipulatedframesetconstraint.h"
#include <QMouseEvent>
#include "standardcamera.h"
#include <icpdialog.h>
#include "QGLHelper.h"
#define USE_CONSOLE_DEBUG
using namespace qglviewer;
void generateClusters(const QList<int>& input,  QVector<QVector<int>>& clusters){
    auto sel = input;
    qSort(sel.begin(), sel.end());
    int prev = -1000;
    for(const auto&a:sel){
        if(a-prev > 5){
            clusters.push_back(QVector<int>());
        }
        clusters.back().push_back(a);
        prev = a;
    }
#ifdef USE_CONSOLE_DEBUG
    std::cerr<<clusters.size()<<" clusters are generated."<<std::endl;
#endif
}

CloudViewer::CloudViewer(QWidget *parent):
    QGLViewer(parent),clouds_visible_all(true){

}

void CloudViewer::setGraphDisplayer(GraphDisplayer::Ptr graph){
    logger->logMessage("setGraphDisplayer");
    this->graph = graph;
    updateGL();
}

void CloudViewer::setPointCloudDisplayers(QVector<PointCloudDisplayer::Ptr> clouds){
    logger->logMessage("setPointCloudDisplayers");
    this->clouds = clouds;
    initPointCloudVisibleVector();
    updateGL();
}

void CloudViewer::slot_manualLoopClosing(){
#if 0
    if(selections.size() == 2){
        ICPDialog icp_dialog;
        auto sel = selections;
        qSort(sel.begin(), sel.end());
        icp_dialog.setInputClouds(*clouds[sel[0]],* clouds[sel[1]]);
        icp_dialog.setModal(true);
        if(icp_dialog.exec()){
            qglviewer::Frame icp_result =  icp_dialog.getICPResult();
            g2o::EdgeSE3::InformationType info;
            info.setIdentity();
            Q_EMIT loopClosingAdded(sel[0], sel[1], QGLHelper::toPosTypesPose3D(icp_result), info);
        }
    }else{
        logger->logMessage("Please, choose only 2 vertices to perform ICP.");
    }
#else
    if(selections.size() < 2){
        logger->logMessage("Please, choose at least 2 vertices.");
        return;
    }
    QVector<QVector<int>> clusters;
    generateClusters(selections, clusters);

    if(clusters.size() != 2){
        logger->logMessage(QString("2 clusters are needed.(%1 generated)").arg(clusters.size()).toStdString().c_str());
        return;
    }
    PointCloudDisplayerVector cloud_pts[2];
    for(size_t i=0;i<2;i++){
        for(const auto& idx:clusters[i]){
            cloud_pts[i].push_back(this->clouds[idx]);
        }
    }

    ICPDialog icp_dialog;
    icp_dialog.setModelClouds(cloud_pts[0]);
    icp_dialog.setTemplateClouds(cloud_pts[1]);
    icp_dialog.initialize();
    icp_dialog.setModal(true);
    if(icp_dialog.exec()){
        qglviewer::Frame icp_result =  icp_dialog.getICPResult();
        std::ostringstream oss;
        oss<<"ICP Result : \n"<<std::endl;
        oss<<"\tPosition : "<<icp_result.position()<<std::endl;
        oss<<"\tOrientation : "<<icp_result.orientation()<<std::endl;
        logger->logMessage(oss.str().c_str());
        g2o::EdgeSE3::InformationType info;
        info.setIdentity();
        Q_EMIT loopClosingAdded(clusters[0][0], clusters[1][0], QGLHelper::toPosTypesPose3D(icp_result), info);
    }
#endif
}

void CloudViewer::slot_setPointAlpha(const double &alpha_point_cloud){
    this->alpha_points = alpha_point_cloud;
    updateGL();
}

void CloudViewer::slot_setPointSize(const double &point_size){
    this->size_points = point_size;
    updateGL();
}

void CloudViewer::slot_clearSelections()
{
    selections.clear();
    Q_EMIT verticesSelected(selections);
    updateGL();
}

void CloudViewer::slot_setSelectedCloudsVisible(bool inverse)
{
    bool init, val;
    if(inverse){
        init = true;
        val = false;
    }else{
        init = false;
        val = true;
    }
    std::fill(clouds_visible.begin(), clouds_visible.end(), init);
    for(const auto& idx:selections){
        clouds_visible[idx] = val;
    }
    updateGL();
}

void CloudViewer::slot_clearVisibleFlags(bool value)
{
    std::fill(clouds_visible.begin(), clouds_visible.end(), value);
    updateGL();
}

void CloudViewer::slot_setCloudsVisible(bool visible)
{
    clouds_visible_all = visible;
    updateGL();
}

void CloudViewer::init(){
    initConstraint();

    this->size_points = 1.0;
    this->alpha_points = 1.0;

    glBlendFunc(GL_ONE, GL_ONE);

    qglviewer::Camera* old_cam = camera();
    qglviewer::Camera* new_cam = new StandardCamera();
    new_cam->setPosition(qglviewer::Vec(0,0,100));
    setCamera(new_cam);
    delete old_cam;
    setGridIsDrawn();
    //    setAxisIsDrawn();
    setFPSIsDisplayed();
}

void CloudViewer::draw(){
    glMatrixMode(GL_MODELVIEW);
    if(graph){
        //        graph->drawVertices();
        graph->drawLoopEdges();
        graph->drawMotionEdges();
    }
    if(!clouds.empty()){
        for(size_t i=0;i<clouds.size();i++){
            bool selected = selections.contains(i);
            if(clouds_visible_all&&clouds_visible[i]){
                clouds[i]->drawPointCloud(selected, size_points,alpha_points);
            }
            clouds[i]->drawFrame(selected);
        }
    }
    if(selection_mode != NONE){
        drawSelectionRectangle();
        drawAxis(0.1);
    }
    drawText();
}

void CloudViewer::drawWithNames(){
    if(!clouds.empty()){
        for(size_t i=0;i<clouds.size();i++){
            if(clouds[i]->isSelectable()){
                glPushName(i);
                clouds[i]->drawFrame(false);
                glPopName();
            }
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
        Q_EMIT verticesSelected(selections);
    }
    selection_mode = NONE;
}

void CloudViewer::mousePressEvent(QMouseEvent *e)
{
    // Start selection. Mode is ADD with Shift key and TOGGLE with Alt key.
    rectangle = QRect(e->pos(), e->pos());
    if(e->button() == Qt::LeftButton){
        switch(e->modifiers()){
        case Qt::ShiftModifier|Qt::ControlModifier:
            selection_mode = REMOVE;
            return;
        case Qt::ShiftModifier:
            selection_mode = ADD;
            return;
        case Qt::ControlModifier:
            startManipulation();//Rotation.
            break;
        }
    }else if(e->button()==Qt::RightButton){
        switch(e->modifiers()){
        case Qt::ControlModifier:
            startManipulation();//Translation
            break;
        }
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

void CloudViewer::keyPressEvent(QKeyEvent *key)
{
    switch(key->key()){
    case Qt::Key_Q:        switchTranslationConstraintType();         break;
    case Qt::Key_W:         switchConstraintDirection(false);        break;
    case Qt::Key_E:         switchRotationConstraintType();        break;
    case Qt::Key_R:        switchConstraintDirection(true);        break;
    case Qt::Key_Space:
        if(key->modifiers() == Qt::CTRL){
            switchConstraint();
            break;
        }
    default:
        QGLViewer::keyPressEvent(key);
        return;
    }
    updateGL();
}

void CloudViewer::startManipulation()
{

    qglviewer::Vec control_point;
    std::cerr<<"1"<<std::endl;
    ManipulatedFrameSet *mfs = NULL;

    LocalManipulatedFrameSetConstraint* Lmfsc =
            dynamic_cast<LocalManipulatedFrameSetConstraint*>(manipulatedFrame()->constraint());
    if(Lmfsc){
        mfs = Lmfsc;
    }else{
        WorldManipulatedFrameSetConstraint* Wmfsc =
                dynamic_cast<WorldManipulatedFrameSetConstraint*>(manipulatedFrame()->constraint());
        if(Wmfsc){
            mfs = Wmfsc;
        }else{
            throw std::runtime_error("Cannot dynamic_cast constraint into Local or World constraint.");
        }
    }

    mfs->clear();
    for (QList<int>::const_iterator it=selections.begin(), end=selections.end(); it != end; ++it)
    {
        mfs->add(&(clouds[*it]->frame));
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

void CloudViewer::drawText()
{
    qglColor(foregroundColor());
    glDisable(GL_LIGHTING);
    switch(active_constraint){
    case 0:
        constraint_info_text.drawText(0,0,"Constraint : LOCAL");
        break;
    case 1:
        constraint_info_text.drawText(0,0,"Constraint : GLOBAL");
        break;
    }
    switch(constraints[active_constraint]->translationConstraintType()){
    case AxisPlaneConstraint::PLANE:
        constraint_info_text.drawText(0,20,"Translation Type :  PLANE ");
        break;
    case AxisPlaneConstraint::AXIS:
        constraint_info_text.drawText(0,20,"Translation Type :  AXIS ");
        break;
    case AxisPlaneConstraint::FORBIDDEN:
        constraint_info_text.drawText(0,20,"Translation Type :  FORBIDDEN ");
        break;
    case AxisPlaneConstraint::FREE:
        constraint_info_text.drawText(0,20,"Translation Type :  FREE ");
        break;
    }
    switch(translation_dir){
    case 0:
        constraint_info_text.drawText(0,40,"Translation Dir : X");
        break;
    case 1:
        constraint_info_text.drawText(0,40,"Translation Dir : Y");
        break;
    case 2:
        constraint_info_text.drawText(0,40,"Translation Dir : Z");
        break;
    }

    switch(constraints[active_constraint]->rotationConstraintType()){
    case AxisPlaneConstraint::PLANE:
        constraint_info_text.drawText(0,60,"Rotation Type :  PLANE ");
        break;
    case AxisPlaneConstraint::AXIS:
        constraint_info_text.drawText(0,60,"Rotation Type :  AXIS ");
        break;
    case AxisPlaneConstraint::FORBIDDEN:
        constraint_info_text.drawText(0,60,"Rotation Type :  FORBIDDEN ");
        break;
    case AxisPlaneConstraint::FREE:
        constraint_info_text.drawText(0,60,"Rotation Type :  FREE ");
        break;
    }
    switch(rotation_dir){
    case 0:
        constraint_info_text.drawText(0,80,"Rotation Dir : X");
        break;
    case 1:
        constraint_info_text.drawText(0,80,"Rotation Dir : Y");
        break;
    case 2:
        constraint_info_text.drawText(0,80,"Rotation Dir : Z");
        break;
    }
    glEnable(GL_LIGHTING);
}

void CloudViewer::initConstraint()
{
    constraints[0] = new LocalManipulatedFrameSetConstraint();
    constraints[1] = new WorldManipulatedFrameSetConstraint();
    translation_dir = 2;
    rotation_dir = 2;
    active_constraint = 0;
    qglviewer::Vec vec_tr(0,0,0), vec_rot(0,0,0);
    vec_tr[translation_dir] = 1.0;
    vec_rot[rotation_dir] = 1.0;
    constraints[active_constraint]->setRotationConstraintType(AxisPlaneConstraint::AXIS);
    constraints[active_constraint]->setTranslationConstraintType(AxisPlaneConstraint::PLANE);
    constraints[active_constraint]->setRotationConstraintDirection(vec_rot);
    constraints[active_constraint]->setTranslationConstraintDirection(vec_tr);
    constraint_info_text.setOffset(50, 20);
    constraint_info_text.setViewer(this);

    setManipulatedFrame(new qglviewer::ManipulatedFrame());
    manipulatedFrame()->setConstraint(constraints[active_constraint]);
}

void CloudViewer::switchTranslationConstraintType()
{
    AxisPlaneConstraint::Type type= constraints[active_constraint]->translationConstraintType();
    switch(type){
    case AxisPlaneConstraint::FREE:
        type = AxisPlaneConstraint::PLANE;
        break;
    case AxisPlaneConstraint::PLANE:
        type = AxisPlaneConstraint::AXIS;
        break;
    case AxisPlaneConstraint::AXIS:
        type = AxisPlaneConstraint::FORBIDDEN;
        break;
    case AxisPlaneConstraint::FORBIDDEN: default:
        type = AxisPlaneConstraint::FREE;
        break;
    }
    constraints[active_constraint]->setTranslationConstraintType(type);
}

void CloudViewer::switchRotationConstraintType()
{
    AxisPlaneConstraint::Type type= constraints[active_constraint]->rotationConstraintType();
    switch(type){
    case AxisPlaneConstraint::FREE:
        type = AxisPlaneConstraint::AXIS;
        break;
    case AxisPlaneConstraint::AXIS:
        type = AxisPlaneConstraint::FORBIDDEN;
        break;
    case AxisPlaneConstraint::PLANE:
    case AxisPlaneConstraint::FORBIDDEN:
    default:
        type = AxisPlaneConstraint::FREE;
        break;
    }
    constraints[active_constraint]->setRotationConstraintType(type);
}

void CloudViewer::switchConstraint()
{
    int prev = active_constraint;
    active_constraint = (active_constraint+1)%2;
    constraints[active_constraint]->setRotationConstraintType(constraints[prev]->rotationConstraintType());
    constraints[active_constraint]->setTranslationConstraintType(constraints[prev]->translationConstraintType());
    constraints[active_constraint]->setRotationConstraintDirection(constraints[prev]->rotationConstraintDirection());
    constraints[active_constraint]->setTranslationConstraintDirection(constraints[prev]->translationConstraintDirection());
    manipulatedFrame()->setConstraint(constraints[active_constraint]);
}

void CloudViewer::switchConstraintDirection(bool rotation)
{
    Vec dir(0,0,0);
    if(rotation){
        rotation_dir = (rotation_dir+1)%3;
        dir[rotation_dir] = 1.0;
        constraints[active_constraint]->setRotationConstraintDirection(dir);
    }else{
        translation_dir = (translation_dir+1)%3;
        dir[translation_dir] = 1.0;
        constraints[active_constraint]->setTranslationConstraintDirection(dir);
    }
}

void CloudViewer::initPointCloudVisibleVector()
{
    if(clouds_visible.size() == clouds.size())
        return;
    clouds_visible.resize(clouds.size());
    std::fill(clouds_visible.begin(), clouds_visible.end(), true);
}
