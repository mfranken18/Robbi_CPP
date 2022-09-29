#include "equipment.h"

//Max und Fabi
//#include <Arduino.h>
#include "main.h" //wegen HAL GPIO


Equipment::Equipment(uint16_t equipment_pin, GPIO_TypeDef* equipment_port){
  pin = equipment_pin;
  //Max und Fabi
  port = equipment_port;

  //pinMode(pin, OUTPUT); //Max: müsste bei uns automatisch durch HAL gesetzt sein oder?
}

void Equipment::cmdOn(){
  //digitalWrite(pin, HIGH);
  HAL_GPIO_WritePin(port, pin, GPIO_PIN_SET);
}

void Equipment::cmdOff(){
  //digitalWrite(pin, LOW);
  HAL_GPIO_WritePin(port, pin, GPIO_PIN_RESET);
}
