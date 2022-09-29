#ifndef COMMAND_H_
#define COMMAND_H_

//#include <Arduino.h>
#include "interpolation.h"

//Max und Fabi
#ifdef __cplusplus
#include <string>

typedef struct{
  char id;
  int num;
  float valueX;
  float valueY;
  float valueZ;
  float valueF;
  float valueE;
  float valueS; 
}r_Cmd;


class Command {
  public:
    Command();
    uint8_t handleGcode();
    uint8_t processMessage(std::string msg);
    void value_segment(std::string msg_segment);
    r_Cmd getCmd() const;
    void cmdGetPosition(Point pos, Point pos_offset, float highRad, float lowRad, float rotRad);
    void cmdToRelative();
    void cmdToAbsolute();
    uint8_t isRelativeCoord;
    r_Cmd new_command;

  private: 
    //void replaceAll(std::string& str, const std::string& from, const std::string& to); //Max und Fabi
    std::string message;
};

void cmdMove(r_Cmd(&cmd), Point pos, Point pos_offset, uint8_t isRelativeCoord);
void cmdDwell(r_Cmd(&cmd));
void printErr();
void replaceAll(std::string& str, const std::string& from, const std::string& to);

#endif
#endif












