#ifndef LOGGER_H
#define LOGGER_H

class MessageLogger{
public:
    virtual void logMessage(const char* msg) = 0;
};

#endif // LOGGER_H
