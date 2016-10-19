#ifndef GRAPHTABLEDIALOG_H
#define GRAPHTABLEDIALOG_H

#include <QDialog>

namespace Ui {
class GraphTableDialog;
}

class GraphTableDialog : public QDialog
{
    Q_OBJECT

public:
    explicit GraphTableDialog(QWidget *parent = 0);
    ~GraphTableDialog();

private:
    Ui::GraphTableDialog *ui;
};

#endif // GRAPHTABLEDIALOG_H
