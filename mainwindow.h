#ifndef MAINWINDOW_H
#define MAINWINDOW_H
#include <QMainWindow>
#include <graphslam.h>
#include <QDir>
#include "exceptions.h"
#include "selectioninfo.h"
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
    void loadGraphFromG2ODatabase(const QDir& db_dir);
public Q_SLOTS:
    void resetMessage();
    void appendMessage(QString msg);
    void setSelections(GraphSelectionInfo info);
    void slot_updateGraph(EdgeTableModel::Ptr loop_edges);

private Q_SLOTS://because of no_keywords flag, we have to use Q_SLOTS instead of slots.
    void slot_edgesTableDialogClosed();
    void on_action_File_Open_triggered();
    void on_pushButton_Optimize_clicked();
    void on_pushButton_Reset_clicked();
    void on_pushButton_ClearLoopClosings_clicked();
    void on_pushButton_ApplyInfoMatrix_clicked();
    void on_pushButton_DeleteLoopEdge_clicked();
    void on_pushButton_DeleteSelectedLoopClosings_clicked();
    void on_action_Save_As_G2O_triggered();
    void on_action_Edges_Table_triggered();

    void on_action_Open_From_G2O_triggered();

private:
    Ui::MainWindow *ui;

    GraphTableDialog *  graph_table_dialog;

    GraphSLAM graph_slam;
    GraphSelectionInfo selection_info;
    EdgeTableModel::Ptr loop_table_model;
};

#endif // MAINWINDOW_H
