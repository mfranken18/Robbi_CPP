#ifndef RAMPSSTEPPER_H_
#define RAMPSSTEPPER_H_

//Max und Fabi
#include "config.h" //wegen HIGH und LOW
#include <stdint.h> //wegen uint8_t
#include "stm32f411xe.h" //wegen Ports

class RampsStepper {
public:
  RampsStepper(uint16_t aStepPin, uint16_t aDirPin, uint16_t aEnablePin, uint8_t aInverse, float main_gear_teeth, float motor_gear_teeth, uint16_t microsteps, uint16_t steps_per_rev, GPIO_TypeDef* astep_port, GPIO_TypeDef* adir_port, GPIO_TypeDef* aen_port);
  void enable(uint8_t value = TRUE);
    
  uint8_t isOnPosition() const;
  uint16_t getPosition() const;
  void setPosition(uint16_t value);
  void stepToPosition(uint16_t value);
  void stepToPositionMM(float mm, float steps_per_mm);
  void stepRelative(uint16_t value);
  float getPositionRad() const;
  void setPositionRad(float rad);
  void stepToPositionRad(float rad);
  void stepRelativeRad(float rad);
  
  void update();
  
  void setReductionRatio(float gearRatio, uint16_t stepsPerRev);
private:
  uint16_t stepperStepTargetPosition;
  uint16_t stepperStepPosition;
  uint16_t stepPin;
  uint16_t dirPin;
  uint16_t enablePin;
  uint8_t inverse;
  float radToStepFactor;

  //Max und Fabi
  GPIO_TypeDef* step_port;
  GPIO_TypeDef* dir_port;
  GPIO_TypeDef* en_port;
};

#endif
