//#include <Arduino.h>
#include "RampsStepper.h"

//max und Fabi
#include "main.h" //wegen HAL und PINS

RampsStepper::RampsStepper(uint16_t aStepPin, uint16_t aDirPin, uint16_t aEnablePin, uint8_t aInverse, float main_gear_teeth, float motor_gear_teeth, uint16_t microsteps, uint16_t steps_per_rev, GPIO_TypeDef* astep_port, GPIO_TypeDef* adir_port, GPIO_TypeDef* aen_port)
{
  setReductionRatio(main_gear_teeth / motor_gear_teeth, microsteps * steps_per_rev);
  stepPin = aStepPin;
  dirPin = aDirPin;
  enablePin = aEnablePin;
  inverse = aInverse;
  stepperStepPosition = 0;
  stepperStepTargetPosition;

  //Max und Fabi
  step_port = astep_port;
  dir_port = adir_port;
  en_port = aen_port;

  //pinMode(stepPin, OUTPUT);
  //pinMode(dirPin, OUTPUT);
  //pinMode(enablePin, OUTPUT);
  enable(false);
}

void RampsStepper::enable(uint8_t value) {
  //digitalWrite(enablePin, !value);

  //Max und Fabi
	HAL_GPIO_WritePin(en_port, enablePin, (GPIO_PinState)(!value));
}

uint8_t RampsStepper::isOnPosition() const {
  return stepperStepPosition == stepperStepTargetPosition;
}

uint16_t RampsStepper::getPosition() const {
  return stepperStepPosition;
}

void RampsStepper::setPosition(uint16_t value) {
  stepperStepPosition = value;
  stepperStepTargetPosition = value;
}

void RampsStepper::stepToPosition(uint16_t value) {
  stepperStepTargetPosition = value;
}

void RampsStepper::stepToPositionMM(float mm, float steps_per_mm) {
  stepperStepTargetPosition = mm * steps_per_mm;
}

void RampsStepper::stepRelative(uint16_t value) {
  value += stepperStepPosition;
  stepToPosition(value);
}

float RampsStepper::getPositionRad() const {
  return stepperStepPosition / radToStepFactor;
}

void RampsStepper::setPositionRad(float rad) {
  setPosition(rad * radToStepFactor);
}

void RampsStepper::stepToPositionRad(float rad) {
  stepperStepTargetPosition = rad * radToStepFactor;
}

void RampsStepper::stepRelativeRad(float rad) {
  stepRelative(rad * radToStepFactor);
}

void RampsStepper::update() {   
  while (stepperStepTargetPosition < stepperStepPosition) {  
	//digitalWrite(dirPin, !inverse);
    //digitalWrite(stepPin, HIGH); //Max: WTF???
    //digitalWrite(stepPin, LOW); //Max: WTF???
    //Max und Fabi
    HAL_GPIO_WritePin(dir_port, dirPin, (GPIO_PinState)(!inverse));
    HAL_GPIO_WritePin(step_port, stepPin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(step_port, stepPin, GPIO_PIN_RESET);

    stepperStepPosition--;
  }
  
  while (stepperStepTargetPosition > stepperStepPosition) {    
    //digitalWrite(dirPin, inverse);
    //digitalWrite(stepPin, HIGH); //Max: WTF???
    //digitalWrite(stepPin, LOW); //Max: WTF???
    //Max und Fabi
    HAL_GPIO_WritePin(dir_port, dirPin, (GPIO_PinState)(!inverse));
    HAL_GPIO_WritePin(step_port, stepPin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(step_port, stepPin, GPIO_PIN_RESET);

    stepperStepPosition++;
  }
}

void RampsStepper::setReductionRatio(float gearRatio, uint16_t stepsPerRev) {
  //radToStepFactor = gearRatio * stepsPerRev / 2 / PI;
	//Max und Fabi
	radToStepFactor = gearRatio * stepsPerRev / PI_2;
};
