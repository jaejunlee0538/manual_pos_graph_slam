#ifndef ICPDIALOG_H
#define ICPDIALOG_H

#include <QDialog>
#include "pointclouddisplayer.h"
#include "postypes.h"
namespace Ui {
class ICPDialog;
}

class ICPDialog : public QDialog
{
    Q_OBJECT
private:
    enum TextDrawingID{
        ICP_ITERATIONS= 1
    };
public:
    explicit ICPDialog(QWidget *parent = 0);
    ~ICPDialog();
    void setModelClouds(const QVector<PointCloudDisplayer::Ptr>& inputs);
    void setTemplateClouds(const QVector<PointCloudDisplayer::Ptr>& inputs);
    void initialize();
    qglviewer::Frame getICPResult();
Q_SIGNALS:
    void inputCloudUpdated(QVector<PointCloudDisplayer::Ptr> clouds);

private Q_SLOTS:
    void on_pushButton_DoICP_clicked();

    void on_pushButton_Reset_clicked();
    /*
     *  Accept current nodes configuration.
     */
    void on_pushButton_Accept_clicked();

    void on_pushButton_Cancel_clicked();

private:

    void updateViewer();
    void drawTextInViewer(const TextDrawingID& id, const QString& txt, const int& posx, const int& posy);
private:
    Ui::ICPDialog *ui;
    PointCloudDisplayer::Ptr clouds[2];//2 length array=>[model_cloud,  input_cloud(manipulated)]
    qglviewer::Frame result;
    qglviewer::Frame original;
};

#endif // ICPDIALOG_H
