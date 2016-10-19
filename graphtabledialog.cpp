#include "graphtabledialog.h"
#include "ui_graphtabledialog.h"

GraphTableDialog::GraphTableDialog(QWidget *parent) :
    QDialog(parent),
    ui(new Ui::GraphTableDialog)
{
    ui->setupUi(this);
}

GraphTableDialog::~GraphTableDialog()
{
    delete ui;
}
