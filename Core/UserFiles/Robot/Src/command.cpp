#include "command.h"
#include "logger.h"
//#include <Arduino.h>

//Max und Fabi
#include "cmsis_os.h"
#include <bits/stdc++.h>
#include "math.h" // wegen isnan


Command::Command() {
  //initialize Command to a zero-move value;
  new_command.valueX = NAN; 
  new_command.valueY = NAN;
  new_command.valueZ = NAN;
  new_command.valueF = 0;
  new_command.valueE = NAN;
  new_command.valueS = 0;
  message = "";
  isRelativeCoord = false;
}

uint8_t Command::handleGcode() {


//	if (Serial.available()) //Max: der Quark ist eine Arduino Funktion
//  {
//    char c =Serial.read();
//    if (c == '\n') {
//       return false;
//    }
//    if (c == '\r') {
//       uint8_t b = processMessage(message);
//       message = "";
//       return b;
//    } else {
//       message += c;
//    }
//  }
//  return false;


	message = "E12.00";
	uint8_t b = processMessage(message);
	return b;

}

uint8_t Command::processMessage(std::string msg){

  new_command.valueX = NAN; 
  new_command.valueY = NAN;
  new_command.valueZ = NAN;
  new_command.valueE = NAN;
  new_command.valueF = 0;
  new_command.valueS = 0;  

  //msg.toUpperCase();
  std::transform(msg.begin(), msg.end(), msg.begin(), ::toupper);
  //msg.replace(" ", "");
  replaceAll(msg, " ", ""); //neue Funktion


  int active_index = 0;
  new_command.id = msg[active_index];
  if((new_command.id != 'G') && (new_command.id != 'M')){
    printErr();
    return false;
  }

  active_index++;
  int temp_index = active_index;
  while (temp_index<msg.length() && !isalpha(msg[temp_index]))
  {
    temp_index++;
  }
  //new_command.num = msg.substring(active_index, temp_index).toInt();
  new_command.num = std::stoi(msg.substr(active_index, temp_index));
  active_index = temp_index;
  temp_index++;
  while (temp_index<msg.length()){
    while (!isalpha(msg[temp_index]) || msg[temp_index]=='.')
    {
      temp_index++;
      if (temp_index == msg.length()){
        break;
      }
    }
    //value_segment(msg.substring(active_index, temp_index));
    value_segment(msg.substr(active_index, temp_index));
    active_index = temp_index;
    temp_index++;
  }
  return true;
}

void Command::value_segment(std::string msg_segment){
  //float msg_value = msg_segment.substring(1).toFloat();
	float msg_value = std::stof(msg_segment.substr(1));
  switch (msg_segment[0]){
    case 'X': new_command.valueX = msg_value; break;
    case 'Y': new_command.valueY = msg_value; break;
    case 'Z': new_command.valueZ = msg_value; break;
    case 'E': new_command.valueE = msg_value; break;
    case 'F': new_command.valueF = msg_value; break;
    case 'S': new_command.valueS = msg_value; break;
  }
}


r_Cmd Command::getCmd() const {
  return new_command; 
}

void Command::cmdGetPosition(Point pos, Point pos_offset, float highRad, float lowRad, float rotRad){
  if(isRelativeCoord) {
    Logger::logINFO("RELATIVE MODE");
  } else {
    Logger::logINFO("ABSOLUTE MODE");
  }
  //std::to_string nicht optimal
  Logger::logINFO("CURRENT POSITION: [X:" + std::to_string(pos.xmm - pos_offset.xmm) + " Y:" + std::to_string(pos.ymm - pos_offset.ymm) + " Z:" + std::to_string(pos.zmm - pos_offset.zmm) + " E:" + std::to_string(pos.emm - pos_offset.emm) + "]");
  Logger::logINFO("RADIANS: [HIGH:" + std::to_string(highRad) + " LOW:" + std::to_string(lowRad) + " ROT:" + std::to_string(rotRad));
}

void Command::cmdToRelative(){
  isRelativeCoord = true;
  Logger::logINFO("RELATIVE MODE ON");
}

void Command::cmdToAbsolute(){
  isRelativeCoord = false;
  Logger::logINFO("ABSOLUTE MODE ON");
}

void cmdMove(r_Cmd(&cmd), Point pos, Point pos_offset, uint8_t isRelativeCoord){

  if(isRelativeCoord == true){
    cmd.valueX = isnan(cmd.valueX) ? pos.xmm : cmd.valueX + pos.xmm;
    cmd.valueY = isnan(cmd.valueY) ? pos.ymm : cmd.valueY + pos.ymm;
    cmd.valueZ = isnan(cmd.valueZ) ? pos.zmm : cmd.valueZ + pos.zmm;
    cmd.valueE = isnan(cmd.valueE) ? pos.emm : cmd.valueE + pos.emm; 
  } else {
    cmd.valueX = isnan(cmd.valueX) ? pos.xmm : cmd.valueX + pos_offset.xmm;
    cmd.valueY = isnan(cmd.valueY) ? pos.ymm : cmd.valueY + pos_offset.ymm;
    cmd.valueZ = isnan(cmd.valueZ) ? pos.zmm : cmd.valueZ + pos_offset.zmm;
    cmd.valueE = isnan(cmd.valueE) ? pos.emm : cmd.valueE + pos_offset.emm;
  }
}

void cmdDwell(r_Cmd(&cmd)){
  osDelay(int(cmd.valueS * 1000));
}

void printErr() {
  Logger::logERROR("COMMAND NOT RECOGNIZED");
}

void replaceAll(std::string& str, const std::string& from, const std::string& to) {
    if(from.empty())
        return;
    size_t start_pos = 0;
    while((start_pos = str.find(from, start_pos)) != std::string::npos) {
        str.replace(start_pos, from.length(), to);
        start_pos += to.length(); // In case 'to' contains 'from', like replacing 'x' with 'yx'
    }
}
