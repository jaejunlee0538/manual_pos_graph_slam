#ifndef LOGGER_H
#define LOGGER_H
#include <iostream>

class MessageLogger{
public:
    virtual void logMessage(const char* msg) = 0;
};

#define DEBUG_FUNC_STAMP       std::cerr<<Q_FUNC_INFO<<std::endl
#define DEBUG_MUST_NOT_RUN      std::cerr << "\033[1;31m"<<"This code should not run["<<Q_FUNC_INFO<<"]\033[0m"<<std::endl
#endif // LOGGER_H
