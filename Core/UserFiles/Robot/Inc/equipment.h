#ifndef EQUIPMENT_H_
#define EQUIPMENT_H_

//Max und Fabi
#include <stdint.h> //wegen uint16_t
#include "stm32f411xe.h" // wegen ports


class Equipment {
public:
  Equipment(uint16_t equipment_pin, GPIO_TypeDef* equipment_port);
  void cmdOn();
  void cmdOff();
private:
  uint16_t pin;
  //Max und Fabi
  GPIO_TypeDef* port;
};

#endif
