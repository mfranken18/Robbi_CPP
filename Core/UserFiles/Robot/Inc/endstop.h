#ifndef ENDSTOP_H_
#define ENDSTOP_H_

#include "stdint.h" //wegen uint16
#include "stm32f411xe.h" //wegen Ports

class Endstop {
  public:
    Endstop(uint16_t a_min_pin, uint16_t a_dir_pin, uint16_t a_step_pin, uint16_t a_en_pin, uint16_t a_switch_input, uint16_t a_step_offset, uint16_t a_home_dwell, uint8_t does_swap_pin, GPIO_TypeDef* a_min_port, GPIO_TypeDef* a_dir_port, GPIO_TypeDef* a_step_port, GPIO_TypeDef* a_en_port);
    void home(uint8_t dir);
    void homeOffset(uint8_t dir);
    void oneStepToEndstop(uint8_t dir);
    bool state();
    uint8_t bState;

  private:
    uint16_t min_pin;
    uint16_t dir_pin;
    uint16_t step_pin;
    uint16_t en_pin;
    uint16_t switch_input;
    uint16_t home_dwell;
    uint16_t step_offset;
    uint8_t swap_pin;

    //Max und Fabi
    GPIO_TypeDef* min_port;
    GPIO_TypeDef* dir_port;
    GPIO_TypeDef* step_port;
    GPIO_TypeDef* en_port;

};

#endif
