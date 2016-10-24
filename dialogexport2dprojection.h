#ifndef DIALOGEXPORT2DPROJECTION_H
#define DIALOGEXPORT2DPROJECTION_H

#include <QDialog>

namespace Ui {
class DialogExport2DProjection;
}

class DialogExport2DProjection : public QDialog
{
    Q_OBJECT

public:
    explicit DialogExport2DProjection(QWidget *parent = 0);
    ~DialogExport2DProjection();
    double minHeight()const;
    double maxHeight()const;
    bool isHeightCutEnabled()const;

    bool isIntensityCutEnalbed() const;
    double minIntensity() const;
    double maxIntensity() const;
private:
    Ui::DialogExport2DProjection *ui;
};

#endif // DIALOGEXPORT2DPROJECTION_H
