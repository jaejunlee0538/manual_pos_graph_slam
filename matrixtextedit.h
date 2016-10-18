#ifndef MATRIXTEXTEDIT_H
#define MATRIXTEXTEDIT_H
#include <QPlainTextEdit>
#include <Eigen/Dense>
class MatrixTextEdit : public QPlainTextEdit
{
    Q_OBJECT
public:
    explicit MatrixTextEdit(QWidget *parent = 0);
    Eigen::MatrixXd value() const;
Q_SIGNALS:

public Q_SLOTS:

};

#endif // MATRIXTEXTEDIT_H
