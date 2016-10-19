#include "graphtabledialog.h"
#include "ui_graphtabledialog.h"

GraphTableDialog::GraphTableDialog(QWidget *parent) :
    QDialog(parent),
    ui(new Ui::GraphTableDialog)
{
    ui->setupUi(this);

    //delete this dialog when ESC key or 'X' button is pressed.
    this->setAttribute(Qt::WA_DeleteOnClose);
}

GraphTableDialog::~GraphTableDialog()
{
    delete ui;
}

