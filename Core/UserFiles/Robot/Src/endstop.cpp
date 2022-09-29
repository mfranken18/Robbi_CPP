#include "endstop.h"
//#include <Arduino.h>

//Max:
#include "config.h" //wegen HIGH und LOW
#include "cmsis_os.h" //wegen osDelay
#include "main.h"


Endstop::Endstop(uint16_t a_min_pin, uint16_t a_dir_pin, uint16_t a_step_pin, uint16_t a_en_pin, uint16_t a_switch_input, uint16_t a_step_offset, uint16_t a_home_dwell, uint8_t does_swap_pin, GPIO_TypeDef* a_min_port, GPIO_TypeDef* a_dir_port, GPIO_TypeDef* a_step_port, GPIO_TypeDef* a_en_port){
  min_pin = a_min_pin;
  dir_pin = a_dir_pin;
  step_pin = a_step_pin;
  en_pin = a_en_pin;
  switch_input = a_switch_input;
  home_dwell = a_home_dwell;
  step_offset = a_step_offset;
  swap_pin = does_swap_pin;

  //Max und Fabi
  min_port = a_min_port;
  dir_port = a_dir_port;
  step_port = a_step_port;
  en_port = a_en_port;

  //if (swap_pin == FALSE){
    //pinMode(min_pin, INPUT_PULLUP);
  //}
}

void Endstop::home(uint8_t dir) {
  //if (swap_pin == TRUE){
    //pinMode(min_pin, INPUT_PULLUP);
    //osDelay(5);
  //}
  //digitalWrite(en_pin, LOW);
  HAL_GPIO_WritePin(en_port, en_pin, GPIO_PIN_RESET);
  osDelay(5);
  if (dir==1){
    //digitalWrite(dir_pin, HIGH);
    HAL_GPIO_WritePin(dir_port, dir_pin, GPIO_PIN_SET);
  } else {
    //digitalWrite(dir_pin, LOW);
    HAL_GPIO_WritePin(dir_port, dir_pin, GPIO_PIN_RESET);
  }
  osDelay(5);
  //Max und Fabi
  //bState = !(digitalRead(min_pin) ^ switch_input);
  bState = !(HAL_GPIO_ReadPin(min_port, min_pin) ^ switch_input);
  while (!bState) {
    //digitalWrite(step_pin, HIGH);
    //digitalWrite(step_pin, LOW);
    HAL_GPIO_WritePin(step_port, step_pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(step_port, step_pin, GPIO_PIN_RESET);

    osDelay(home_dwell);
    //Max und Fabi
    //bState = !(digitalRead(min_pin) ^ switch_input);
    bState = !(HAL_GPIO_ReadPin(min_port, min_pin) ^ switch_input);
  }
  homeOffset(dir);
  //if (swap_pin == TRUE){
    //pinMode(min_pin, OUTPUT);
    //osDelay(5);
  //}
}

void Endstop::homeOffset(uint8_t dir){
  if (dir==1){
    //digitalWrite(dir_pin, LOW);
    HAL_GPIO_WritePin(dir_port, dir_pin, GPIO_PIN_RESET);
  }
  else{
    //digitalWrite(dir_pin, HIGH);
    HAL_GPIO_WritePin(dir_port, dir_pin, GPIO_PIN_SET);
  }
  osDelay(5);
  for (int i = 1; i <= step_offset; i++) {
    //digitalWrite(step_pin, HIGH);
    //digitalWrite(step_pin, LOW);
    HAL_GPIO_WritePin(step_port, step_pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(step_port, step_pin, GPIO_PIN_RESET);

    osDelay(home_dwell);
  }
}

void Endstop::oneStepToEndstop(uint8_t dir){
  //if (swap_pin == TRUE){
    //pinMode(min_pin, INPUT_PULLUP);
  //}
  //digitalWrite(en_pin, LOW);
  HAL_GPIO_WritePin(en_port, en_pin, GPIO_PIN_RESET);
  osDelay(5);
  if (dir==1){
    //digitalWrite(dir_pin, HIGH);
    HAL_GPIO_WritePin(dir_port, dir_pin, GPIO_PIN_SET);
  } else {
    //digitalWrite(dir_pin, LOW);
    HAL_GPIO_WritePin(dir_port, dir_pin, GPIO_PIN_RESET);
  }
  osDelay(5);
  //Max und Fabi
  //bState = !(digitalRead(min_pin) ^ switch_input);
  bState = !(HAL_GPIO_ReadPin(min_port, min_pin) ^ switch_input);

  if (!bState) {
    //digitalWrite(step_pin, HIGH);
    //digitalWrite(step_pin, LOW);
	  HAL_GPIO_WritePin(step_port, step_pin, GPIO_PIN_RESET);
	  HAL_GPIO_WritePin(step_port, step_pin, GPIO_PIN_RESET);

    osDelay(home_dwell);
  }
  //Max und Fabi
  //bState = !(digitalRead(min_pin) ^ switch_input);
  bState = !(HAL_GPIO_ReadPin(min_port, min_pin) ^ switch_input);
}

bool Endstop::state(){
  //if (swap_pin == TRUE){
    //pinMode(min_pin, INPUT_PULLUP);
    //osDelay(5);
  //}

	//Max und Fabi
  //bState = !(digitalRead(min_pin) ^ switch_input);
  bState = !(HAL_GPIO_ReadPin(min_port, min_pin) ^ switch_input);
  //if (swap_pin == TRUE){
    //pinMode(min_pin, OUTPUT);
    //osDelay(5);
  //}
  return bState;
}
