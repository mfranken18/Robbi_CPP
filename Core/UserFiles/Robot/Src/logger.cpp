#include "config.h"
#include "logger.h"


void Logger::log(std::string message, int level) {
  if(LOG_LEVEL >= level) {
	  std::string logMsg;
    switch(level) {
      case LOG_ERROR:
        logMsg = "ERROR: ";
      break;
      case LOG_INFO:
        logMsg = "INFO: ";
      break;
      case LOG_DEBUG:
        logMsg = "DEBUG: ";
      break;
    }
    logMsg = logMsg + message;

    //Max und Fabi
    //Serial.println(logMsg);
  }
}

void Logger::logERROR(std::string message) {
  log(message, LOG_ERROR);
}
void Logger::logINFO(std::string message) {
  log(message, LOG_INFO);
}
void Logger::logDEBUG(std::string message) {
  log(message, LOG_DEBUG);
}
