#include "mainwindow.h"
#include <QApplication>
#include "graphslam.h"
#include "LoggerQEditBox.h"
#include "Global.h"
#include <QGLViewer/frame.h>
#include <QGLHelper.h>
void printFrame(qglviewer::Frame& f){
    std::cerr<<f.orientation()<<std::endl;
    std::cerr<<f.position()<<std::endl;
}

int main(int argc, char *argv[])
{
    QApplication a(argc, argv);

    MessageLoggerQEditBox *q_edit_logger = new MessageLoggerQEditBox();
    logger = q_edit_logger;

    MainWindow w;
    w.show();

    QObject::connect(q_edit_logger, SIGNAL(sendMessage(QString)), &w, SLOT(appendMessage(QString)));
    w.init();
    return a.exec();
}
