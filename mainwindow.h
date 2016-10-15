#ifndef MAINWINDOW_H
#define MAINWINDOW_H
#include <QMainWindow>
#include <graphslam.h>
#include <QDir>
#include "exceptions.h"
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
    void on_action_File_Open_triggered();

    void on_pushButton_Optimize_clicked();

    void on_pushButton_Reset_clicked();

    void on_pushButton_SearchLoopClosings_clicked();

private:
    Ui::MainWindow *ui;

    GraphSLAM graph_slam;
};

#endif // MAINWINDOW_H
