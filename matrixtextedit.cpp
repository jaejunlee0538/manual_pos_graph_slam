#include "matrixtextedit.h"
#include <stdexcept>
MatrixTextEdit::MatrixTextEdit(QWidget *parent) :
    QPlainTextEdit(parent)
{
}


Eigen::MatrixXd MatrixTextEdit::value() const
{
    Eigen::MatrixXd matrix;
//    QString str = plainText;
//    QStringList lines = str.split("\n", QString::SplitBehavior::SkipEmptyParts);
//    std::vector<double> data;
//    int m=0,n=0;
//    for(QStringList::iterator iter = lines.begin();iter!=lines.end();iter++){
//        m++;
//        QStringList eles = iter->split(' ', QString::SplitBehavior::SkipEmptyParts);
//        if(n == 0)
//            n = eles.size();
//        if(n != eles.size()){
//            throw std::runtime_error("Row size is different.");
//        }
//        for(QStringList::iterator ele=eles.begin();ele!=eles.end();ele++){
//            bool valid;
//            double v = ele->toDouble(&valid);
//            if(!valid){
//                throw std::runtime_error(QString("Invalid element - %1").arg(*ele).toStdString());
//            }
//            data.push_back(v);
//        }
//    }
//    matrix.resize(m, n);
//    for(size_t irow=0;irow<m;irow++){
//        for(size_t icol=0;icol<n;icol++){
//            matrix(irow, icol) = data[irow*n+icol];
//        }
//    }
    return matrix;
}
