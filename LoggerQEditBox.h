#ifndef LOGGERQEDITBOX_H
#define LOGGERQEDITBOX_H
#include "Logger.h"
#include <QObject>
#include <QString>

class MessageLoggerQEditBox:public QObject, public MessageLogger{
Q_OBJECT
public:
    virtual void logMessage(const char* msg){
        Q_EMIT sendMessage(QString(msg));
    }

Q_SIGNALS:
    void sendMessage(QString msg);
};

#endif // LOGGERQEDITBOX_H
