#include "dialogexport2dprojection.h"
#include "ui_dialogexport2dprojection.h"

DialogExport2DProjection::DialogExport2DProjection(QWidget *parent) :
    QDialog(parent),
    ui(new Ui::DialogExport2DProjection)
{
    ui->setupUi(this);
}

DialogExport2DProjection::~DialogExport2DProjection()
{
    delete ui;
}

double DialogExport2DProjection::minHeight() const{
    return ui->doubleSpinBox_minHeight->value();
}

double DialogExport2DProjection::maxHeight() const{
    return ui->doubleSpinBox_maxHeight->value();
}

bool DialogExport2DProjection::isHeightCutEnabled() const{
    return ui->groupBox_CutWithHeight->isChecked();
}

bool DialogExport2DProjection::isIntensityCutEnalbed() const
{
return ui->groupBox_CutWithIntensity->isChecked();
}

double DialogExport2DProjection::minIntensity() const
{
    return ui->doubleSpinBox_minIntensity->value();
}

double DialogExport2DProjection::maxIntensity() const
{
    return ui->doubleSpinBox_maxIntensity->value();
}
