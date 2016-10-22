#ifndef MAINWINDOW_H
#define MAINWINDOW_H
#include <QMainWindow>
#include <graphslam.h>
#include <QDir>
#include "exceptions.h"
#include "graphtabledialog.h"

namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = 0);
    ~MainWindow();
    void init();
private:
    void loadGraphFromPCDDatabase(const QDir& db_dir);

public Q_SLOTS:
    void resetMessage();
    void appendMessage(QString msg);
private Q_SLOTS://because of no_keywords flag, we have to use Q_SLOTS instead of slots.
    //called when graph table dialog is being closed.
    void slot_graphTableDialogClosed();

    //Push button slot
    void on_pushButton_Optimize_clicked();
    void on_pushButton_Reset_clicked();
    void on_pushButton_ClearLoopClosings_clicked();

    //Menu action
    void on_action_File_Open_triggered();
    void on_action_Save_As_G2O_triggered();
    void on_action_Edges_Table_triggered();
    void on_action_Open_From_G2O_triggered();
    void selectVertices(QList<int> vertices_id);
    void on_pushButton_ClearSelection_clicked();
    void on_pushButton_SetSelectedVerticesVisible_clicked();
    void on_pushButton_SetVisibleAll_clicked();
    void on_checkBox_ShowPointCloud_toggled(bool checked);

    void slot_autoSave();
    void on_pushButton_ManualLoopClosing_clicked();

    void on_action2D_Project_triggered();

Q_SIGNALS:
    void loopClosingAdded(const int& vi, const int& vj, const PosTypes::Pose3D& data ,const g2o::EdgeSE3::InformationType& info_mat);

private:
    //ui
    Ui::MainWindow *ui;
    //graph table dialog. nullptr while it is closed.
    GraphTableDialog *  graph_table_dialog;
    //graph slam instance
    GraphSLAM graph_slam;
    QTimer timer;
};

#endif // MAINWINDOW_H
