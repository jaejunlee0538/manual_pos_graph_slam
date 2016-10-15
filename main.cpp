#include "mainwindow.h"
#include <QApplication>
#include "graphslam.h"
#include "LoggerQEditBox.h"
#include "Global.h"
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
