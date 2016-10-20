#ifndef TEXTDRAWHELPER_H
#define TEXTDRAWHELPER_H
#include <QGLViewer/qglviewer.h>

class TextDrawHelper
{
public:
    TextDrawHelper(){
            viewer = NULL;
            xy[0] = xy[1] = 0;
    }

    TextDrawHelper(QGLViewer* viewer, const int& offset_x, const int& offset_y)
        :viewer(viewer){
        xy[0] = offset_x;
        xy[1] = offset_y;
    }

    void setViewer(QGLViewer* viewer){
        this->viewer = viewer;
    }

    void setOffset(const int& offset_x, const int& offset_y){
        xy[0] = offset_x;
        xy[1] = offset_y;
    }

    void drawText(const int& posx, const int& posy, const QString& text, const QFont& font=QFont()){
        viewer->drawText(xy[0]+posx, xy[1]+posy, text, font);
    }

private:
    int xy[2];
    QGLViewer * viewer;
};

#endif // TEXTDRAWHELPER_H
