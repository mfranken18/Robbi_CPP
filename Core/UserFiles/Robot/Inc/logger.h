#ifndef LOGGER_H_
#define LOGGER_H_

//#include <Arduino.h>
//Max und Fabi
#include <string>


#define LOG_ERROR 0
#define LOG_INFO 1
#define LOG_DEBUG 2

class Logger {
  public:
    static void log(std::string message, int level);
    static void logINFO(std::string message);
    static void logERROR(std::string message);
    static void logDEBUG(std::string message);
};
#endif

