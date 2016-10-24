#ifndef LOGGER_H
#define LOGGER_H
#include <iostream>

class MessageLogger{
public:
    virtual void logMessage(const char* msg) = 0;
};

#define DEBUG_MESSAGE_WITH_FUNC_INFO(msg)   std::cerr<<msg<<" ["<<Q_FUNC_INFO<<"]"<<std::endl
#define ERROR_MESSAGE_WITH_FUNC_INFO(msg)   std::cerr<<msg<<" ["<<Q_FUNC_INFO<<"]"<<std::endl
#define DEBUG_FUNC_STAMP       std::cerr<<Q_FUNC_INFO<<std::endl
#define DEBUG_MUST_NOT_RUN      std::cerr << "\033[1;31m"<<"This code should not run["<<Q_FUNC_INFO<<"]\033[0m"<<std::endl
#define RED_MESSAGE(msg)    std::cerr<<"\033[1;31m"<<msg<<" ["<<Q_FUNC_INFO<<"]\033[0m"<<std::endl
#define CONSOLE_WRITE_RED                   "\033[1;31m"
#define CONSOLE_WRITE_RED_END        "]\033[0m"
#endif // LOGGER_H
