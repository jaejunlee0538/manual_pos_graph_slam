#include "icpdialog.h"
#include "ui_icpdialog.h"
#include <QGLViewer/frame.h>
#include <QGLViewer/quaternion.h>
#include "QGLHelper.h"
#include <libicp/icpPointToPlane.h>
#include <libicp/icpPointToPoint.h>
//#define USE_CONSOLE_DEBUG



ICPDialog::ICPDialog(QWidget *parent) :
    QDialog(parent),    ui(new Ui::ICPDialog)
{
    ui->setupUi(this);
    ui->widget_cloudViewer->setTextIsEnabled();
}

ICPDialog::~ICPDialog()
{
    delete ui;
}

void ICPDialog::setModelClouds(const QVector<PointCloudDisplayer::Ptr> &inputs)
{
    clouds[0].reset(new PointCloudDisplayer(*inputs[0]));
    clouds[0]->setSelectable(false);
    for(size_t i=0;i<inputs.size();i++){
        qglviewer::Frame f = QGLHelper::getLocalTransformation(clouds[0]->frame, inputs[i]->frame);
        for(size_t ip=0;ip<inputs[i]->nPoints();ip++){
            PointCloudDisplayer::CloudDataType p = inputs[i]->getPoint(ip);
            qglviewer::Vec pv(p[0], p[1], p[2]);
            pv =inputs[i]->frame.coordinatesOfIn(pv, &(clouds[0]->frame));
            p[0] = pv[0]; p[1] = pv[1]; p[2] = pv[2];
            clouds[0]->pushBack(p);
        }
    }
}

void ICPDialog::setTemplateClouds(const QVector<PointCloudDisplayer::Ptr> &inputs)
{
    clouds[1].reset(new PointCloudDisplayer(*inputs[0]));
    for(size_t i=0;i<inputs.size();i++){
        qglviewer::Frame f = QGLHelper::getLocalTransformation(clouds[1]->frame, inputs[i]->frame);
        for(size_t ip=0;ip<inputs[i]->nPoints();ip++){
            PointCloudDisplayer::CloudDataType p = inputs[i]->getPoint(ip);
            qglviewer::Vec pv(p[0], p[1], p[2]);
            pv =inputs[i]->frame.coordinatesOfIn(pv, &(clouds[1]->frame));
            p[0] = pv[0]; p[1] = pv[1]; p[2] = pv[2];
            clouds[1]->pushBack(p);
        }
    }
}

void ICPDialog::setModelCloud(PointCloudDisplayer::Ptr &input)
{
    clouds[0] = input;
    clouds[0]->setSelectable(false);
}

void ICPDialog::setTemplateCloud(PointCloudDisplayer::Ptr &input)
{
    clouds[1] = input;
    clouds[1]->setSelectable(true);
}

void ICPDialog::initialize()
{
    //express 'input' cloud w.r.t 'model' frame.
    //note that 'model' frame is the origin in ICPDialog.
    clouds[1]->frame.setPosition(clouds[1]->frame.position() - clouds[0]->frame.position());
    clouds[0]->frame.setPosition(0,0,0);
    original = clouds[1]->frame;
    ui->widget_cloudViewer->setPointCloudDisplayers({clouds[0], clouds[1]});
    ui->widget_cloudViewer->slot_setPointSize(1.0);
    ui->widget_cloudViewer->slot_setPointAlpha(0.8);

}

qglviewer::Frame ICPDialog::getICPResult()
{
    return result;
}

void ICPDialog::on_pushButton_DoICP_clicked()
{
#ifdef USE_CONSOLE_DEBUG
    std::cerr<<"Do ICP clicked"<<std::endl;
#endif

    int max_iters = ui->spinBox_MaxIterations->value();
    double inlier_dist = ui->doubleSpinBox_InlierDistance->value();
    double min_delta = ui->doubleSpinBox_MinDelta->value();
    std::vector<double> M, T;
    clouds[0]->copyXYZTo(M);
    clouds[1]->copyXYZTo(T);

    qglviewer::Frame res = QGLHelper::getLocalTransformation(clouds[0]->frame, clouds[1]->frame);
#if 0
    //Use Hand-manipulated data.
    result = res;
    return;
#endif

    libicp::Matrix R = libicp::Matrix::eye(3);
    libicp::Matrix t(3, 1);
    int icp_res=-1;
    libicp::IcpPointToPoint icp(M.data(), M.size()/3, 3);
    icp.setMaxIterations(1);
    icp.setMinDeltaParam(min_delta);
    try{
    for(size_t iterations=0;iterations<max_iters;iterations++){
        QGLHelper::qglFrameToLibicpMatrix(res, R, t);
        icp_res = icp.fit(T.data(), T.size()/3, R, t, inlier_dist);
        QGLHelper::libicpMatrixToqglFrame(R, t, res);
        clouds[1]->frame = QGLHelper::concatenateTransforms(clouds[0]->frame, res);
        drawTextInViewer(ICP_ITERATIONS,QString("ICP : %1 iterations").arg(iterations), 50,20);
        updateViewer();
        if(icp_res == libicp::Icp::CONVERGED){
            break;
        }
        std::cerr<<iterations<<std::endl;
    }
    }catch(std::runtime_error& e){
        std::cerr<<"Error while doing ICP"<<e.what()<<std::endl;
        return;
    }
    if(icp_res == libicp::Icp::CONVERGED){
        drawTextInViewer(ICP_ITERATIONS,QString("ICP : converged"), 50,20);
        updateViewer();
    }
}

void ICPDialog::on_pushButton_Reset_clicked()
{
#ifdef USE_CONSOLE_DEBUG
    std::cerr<<"Reset ICP result"<<std::endl;
#endif
    clouds[1]->frame = original;
    updateViewer();
}

void ICPDialog::on_pushButton_Accept_clicked()
{
#ifdef USE_CONSOLE_DEBUG
    std::cerr<<"ICP result is accepted"<<std::endl;
#endif
    this->result = QGLHelper::getLocalTransformation(this->clouds[0]->frame, this->clouds[1]->frame);
    this->accept();
}

void ICPDialog::on_pushButton_Cancel_clicked()
{
#ifdef USE_CONSOLE_DEBUG
    std::cerr<<"ICP result is rejected"<<std::endl;
#endif
    this->reject();
}

void ICPDialog::updateViewer()
{
    this->ui->widget_cloudViewer->updateGL();
}

void ICPDialog::drawTextInViewer(const ICPDialog::TextDrawingID &id, const QString &txt, const int &posx, const int &posy)
{

}
