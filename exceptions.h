#ifndef EXCEPTIONS_H
#define EXCEPTIONS_H
#include <stdexcept>

class PathNotExist:public std::runtime_error{
public:
    PathNotExist(const std::string& path)
        :std::runtime_error(path+std::string(" does not exist.")){

    }
};

#endif // EXCEPTIONS_H
