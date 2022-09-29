/*
 * robotArm_own.cpp
 *
 *  Created on: Sep 19, 2022
 *      Author: maxfrankenhauser
 */
//20SFFACTORY COMMUNITY ROBOT FIRMWARE

//MAINTAINER: LEOYEUNG@20SFFACTORY
//CONTACT: yeung.kl.leo@gmail.com
//FORUM: www.facebook.com/groups/robotarm

//VERSION: V0.81

//VERSION HISTORY:
//   V0.31 WITH G92, M114, LOGGER, LIMIT_CHECK FUNCTIONS
//   V0.41 WITH DUAL SHANK LENGTH SUPPORT
//   V0.51 WITH SERVO GRIPPER
//   V0.61 WITH ARDUINO UNO OPTION
//   V0.71 WITH:
//       ESP32(WEMOS D1R32) WITH PS4 JOYSTICK CONTROL OPTION
//       COMMAND TO SET CUSTOM SPEED PROFILE 'M205 S0'
//       UNO OPTION WITH RAIL SUPPORT
//   V0.81 WITH WII REMOTE, JOYSTICK ADJUSTABLE SPEED MULTIPLIER

//Max und Fabi
//#include <Arduino.h>
#include <string>
#include "main.h"
#include "robotArm_own.h"

//GENERAL CONFIG SETTINGS
#include "r_queue.h"
#include "config.h"

#include "robotGeometry.h"
#include "interpolation.h"
#include "RampsStepper.h"
#include "equipment.h"
#include "endstop.h"
#include "logger.h"

//INCLUDE CORRESPONDING GRIPPER MOTOR CLASS
#if GRIPPER == SERVO
  //#include "servo_gripper.h"
#elif GRIPPER == BYJ
  //#include "byj_gripper.h"
#endif


//STEPPER OBJECTS
RampsStepper stepperHigher(X_StepPin_Pin, X_DirPin_Pin, X_EnablePin_Pin, INVERSE_X_STEPPER, MAIN_GEAR_TEETH, MOTOR_GEAR_TEETH, MICROSTEPS, STEPS_PER_REV, X_StepPin_GPIO_Port, X_DirPin_GPIO_Port, X_EnablePin_GPIO_Port);
RampsStepper stepperLower(Y_StepPin_Pin, Y_DirPin_Pin, Y_EnablePin_Pin, INVERSE_Y_STEPPER, MAIN_GEAR_TEETH, MOTOR_GEAR_TEETH, MICROSTEPS, STEPS_PER_REV, Y_StepPin_GPIO_Port, Y_DirPin_GPIO_Port, Y_EnablePin_GPIO_Port);
RampsStepper stepperRotate(Z_StepPin_Pin, Z_DirPin_Pin, Z_EnablePin_Pin, INVERSE_Z_STEPPER, MAIN_GEAR_TEETH, MOTOR_GEAR_TEETH, MICROSTEPS, STEPS_PER_REV, Z_StepPin_GPIO_Port, Z_DirPin_GPIO_Port, Z_EnablePin_GPIO_Port);

//RAIL OBJECTS
#if RAIL
  RampsStepper stepperRail(E0_StepPin_Pin, E0_DirPin_Pin, E0_EnablePin_Pin, INVERSE_E0_STEPPER, MAIN_GEAR_TEETH, MOTOR_GEAR_TEETH, MICROSTEPS, STEPS_PER_REV, E0_StepPin_GPIO_Port, E0_DirPin_GPIO_Port, E0_EnablePin_GPIO_Port);
  //#if BOARD_CHOICE == WEMOSD1R32 //PINSWAP REQIURED ON D1R32 DUE TO INSUFFICIENT DIGIAL PINS
    //#define SERVO_PIN 23 // REDEFINE SERVO_PIN FOR RAIL // SHARE WITH Z_MIN_PIN
    //Endstop endstopE0(E0_MIN_PIN, E0_DIR_PIN, E0_STEP_PIN, E0_ENABLE_PIN, E0_MIN_INPUT, E0_HOME_STEPS, HOME_DWELL, true);
  //#else
    Endstop endstopE0(E0_MinPin_Pin, E0_DirPin_Pin, E0_StepPin_Pin, E0_EnablePin_Pin, E0_MinPin_Pin, E0_HOME_STEPS, HOME_DWELL, false, E0_MinPin_GPIO_Port, E0_DirPin_GPIO_Port, E0_StepPin_GPIO_Port, E0_EnablePin_GPIO_Port);
  //#endif
#endif

//ENDSTOP OBJECTS
Endstop endstopX(X_MinPin_Pin, X_DirPin_Pin, X_StepPin_Pin, X_EnablePin_Pin, X_MinPin_Pin, X_HOME_STEPS, HOME_DWELL, false, X_MinPin_GPIO_Port, X_DirPin_GPIO_Port, X_StepPin_GPIO_Port ,X_EnablePin_GPIO_Port);
Endstop endstopY(Y_MinPin_Pin, Y_DirPin_Pin, Y_StepPin_Pin, Y_EnablePin_Pin, Y_MinPin_Pin, Y_HOME_STEPS, HOME_DWELL, false, Y_MinPin_GPIO_Port, Y_DirPin_GPIO_Port, Y_StepPin_GPIO_Port ,Y_EnablePin_GPIO_Port);
Endstop endstopZ(Z_MinPin_Pin, Z_DirPin_Pin, Z_StepPin_Pin, Z_EnablePin_Pin, Z_MinPin_Pin, Z_HOME_STEPS, HOME_DWELL, false, Z_MinPin_GPIO_Port, Z_DirPin_GPIO_Port, Z_StepPin_GPIO_Port ,Z_EnablePin_GPIO_Port);


//EQUIPMENT OBJECTS
//#if GRIPPER == SERVO
  //Servo_Gripper servo_gripper(SERVO_PIN, SERVO_GRIP_DEGREE, SERVO_UNGRIP_DEGREE);
//#elif GRIPPER == BYJ
  //BYJ_Gripper byj_gripper(BYJ_PIN_0, BYJ_PIN_1, BYJ_PIN_2, BYJ_PIN_3, BYJ_GRIP_STEPS);
//#endif

//Equipment laser(LASER_PIN);
//Equipment pump(PUMP_PIN);
Equipment led(LedRamps_Pin, LedRamps_GPIO_Port);
//FanControl fan(FAN_PIN, FAN_DELAY);

//EXECUTION & COMMAND OBJECTS
RobotGeometry geometry(END_EFFECTOR_OFFSET, LOW_SHANK_LENGTH, HIGH_SHANK_LENGTH);
Interpolation interpolator;
R_Queue<r_Cmd> r_queue(QUEUE_SIZE);
Command command;


void robotSetup()
{
  //Serial.begin(BAUD);

  stepperHigher.setPositionRad(PI / 2.0); // 90°
  stepperLower.setPositionRad(0);         // 0°
  stepperRotate.setPositionRad(0);        // 0°
  #if RAIL
  stepperRail.setPosition(0);
  #endif
  if (HOME_ON_BOOT) { //HOME DURING SETUP() IF HOME_ON_BOOT ENABLED
	robotHomeSequence();
    Logger::logINFO("ROBOT ONLINE");
  } else {
    robotSetStepperEnable(false); //ROBOT ADJUSTABLE BY HAND AFTER TURNING ON
    if (HOME_X_STEPPER && HOME_Y_STEPPER && !HOME_Z_STEPPER){
      Logger::logINFO("ROBOT ONLINE");
      Logger::logINFO("ROTATE ROBOT TO FACE FRONT CENTRE & SEND G28 TO CALIBRATE");
    }
    if (HOME_X_STEPPER && HOME_Y_STEPPER && HOME_Z_STEPPER){
      Logger::logINFO("ROBOT ONLINE");
      Logger::logINFO("SEND G28 TO CALIBRATE");
    }
    if (!HOME_X_STEPPER && !HOME_Y_STEPPER){
      Logger::logINFO("ROBOT ONLINE");
      Logger::logINFO("HOME ROBOT MANUALLY & SEND G28 TO CALIBRATE");
    }
  }
  interpolator.setInterpolation(INITIAL_X, INITIAL_Y, INITIAL_Z, INITIAL_E0, INITIAL_X, INITIAL_Y, INITIAL_Z, INITIAL_E0);
}

void robotLoop() {
  interpolator.updateActualPosition();
  geometry.set(interpolator.getXPosmm(), interpolator.getYPosmm(), interpolator.getZPosmm());
  stepperRotate.stepToPositionRad(geometry.getRotRad());
  stepperLower.stepToPositionRad(geometry.getLowRad());
  stepperHigher.stepToPositionRad(geometry.getHighRad());
  #if RAIL
    stepperRail.stepToPositionMM(interpolator.getEPosmm(), STEPS_PER_MM_RAIL);
  #endif
  stepperRotate.update();
  stepperLower.update();
  stepperHigher.update();
  #if RAIL
    stepperRail.update();
  #endif
  //fan.update();

  if (!r_queue.isFull()) {
    if (command.handleGcode()) {
      r_queue.push(command.getCmd());
    }
  }
  if ((!r_queue.isEmpty()) && interpolator.isFinished()) {
	  robotExecuteCommand(r_queue.pop());
    if (PRINT_REPLY) {
      //Serial.println(PRINT_REPLY_MSG);
    }
  }

  if (HAL_GetTick() % (500 * HAL_GetTickFreq() * 1000) < 250) {
    led.cmdOn();
  }
  else {
    led.cmdOff();
  }
}

void robotExecuteCommand(r_Cmd cmd) {

  if (cmd.id == -1) {
    printErr();
    return;
  }

  if (cmd.id == 'G') {
    switch (cmd.num) {
    case 0:
    case 1:
      //fan.enable(true);
      Point posoffset;
      posoffset = interpolator.getPosOffset();
      cmdMove(cmd, interpolator.getPosmm(), posoffset, command.isRelativeCoord);
      interpolator.setInterpolation(cmd.valueX, cmd.valueY, cmd.valueZ, cmd.valueE, cmd.valueF);
      Logger::logINFO("LINEAR MOVE: [X:" + std::to_string(cmd.valueX-posoffset.xmm) + " Y:" + std::to_string(cmd.valueY-posoffset.ymm) + " Z:" + std::to_string(cmd.valueZ-posoffset.zmm) + " E:" + std::to_string(cmd.valueE-posoffset.emm)+"]");
      break;
    case 4: cmdDwell(cmd); break;
    case 28:
      //if (BOARD_CHOICE == UNO || BOARD_CHOICE == WEMOSD1R32){
        //homeSequence_UNO();
        //break;
      //} else {
        robotHomeSequence();
        break;
      //}
    case 90: command.cmdToAbsolute(); break; // ABSOLUTE COORDINATE MODE
    case 91: command.cmdToRelative(); break; // RELATIVE COORDINATE MODE
    case 92:
      interpolator.resetPosOffset();
      cmdMove(cmd, interpolator.getPosmm(), interpolator.getPosOffset(), false);
      interpolator.setPosOffset(cmd.valueX, cmd.valueY, cmd.valueZ, cmd.valueE);
      break;
    default: printErr();
    }
  }
  else if (cmd.id == 'M') {
    switch (cmd.num) {
    //case 1: pump.cmdOn(); break;
    //case 2: pump.cmdOff(); break;
    case 3:
      #if GRIPPER == BYJ
        //byj_gripper.cmdOn(); break;
      #elif GRIPPER == SERVO
        //servo_gripper.cmdOn(); break;
      #endif
    case 5:
      #if GRIPPER == BYJ
        //byj_gripper.cmdOff(); break;
      #elif GRIPPER == SERVO
        servo_gripper.cmdOff(); break;
      #endif
    //case 6: laser.cmdOn(); break;
    //case 7: laser.cmdOff(); break;
    case 17: robotSetStepperEnable(true); break;
    case 18: robotSetStepperEnable(false); break;
    //case 106: fan.enable(true); break;
    //case 107: fan.enable(false); break;
    case 114: command.cmdGetPosition(interpolator.getPosmm(), interpolator.getPosOffset(), stepperHigher.getPosition(), stepperLower.getPosition(), stepperRotate.getPosition()); break;// Return the current positions of all axis
    case 119: {
      std::string endstopMsg = "ENDSTOP: [X:";
      endstopMsg += std::to_string(endstopX.state());
      endstopMsg += " Y:";
      endstopMsg += std::to_string(endstopY.state());
      endstopMsg += " Z:";
      endstopMsg += std::to_string(endstopZ.state());
      #if RAIL
        endstopMsg += " E:";
        endstopMsg += std::to_string(endstopE0.state());
      #endif
      endstopMsg += "]";
      //ORIGINAL LOG STRING UNDESIRABLE FOR UNO PROCESSING
      //Logger::logINFO("ENDSTOP STATE: [UPPER_SHANK(X):"+std::string(endstopX.state())+" LOWER_SHANK(Y):"+std::string(endstopY.state())+" ROTATE_GEAR(Z):"+std::string(endstopZ.state())+"]");
      Logger::logINFO(endstopMsg);
      break;}
    case 205:
      interpolator.setSpeedProfile(cmd.valueS);
      Logger::logINFO("SPEED PROFILE: [" + std::to_string(interpolator.speed_profile) + "]");
      break;
    default: printErr();
    }
  }
  else {
    printErr();
  }
}

void robotSetStepperEnable(uint8_t enable){
  stepperRotate.enable(enable);
  stepperLower.enable(enable);
  stepperHigher.enable(enable);
  #if RAIL
    stepperRail.enable(enable);
  #endif
  //fan.enable(enable);
}

void robotHomeSequence(){
  robotSetStepperEnable(false);
  //fan.enable(true);
  if (HOME_Y_STEPPER && HOME_X_STEPPER){
    endstopY.home(!INVERSE_Y_STEPPER);
    endstopX.home(!INVERSE_X_STEPPER);
  } else {
    robotSetStepperEnable(true);
    endstopY.homeOffset(!INVERSE_Y_STEPPER);
    endstopX.homeOffset(!INVERSE_X_STEPPER);
  }
  if (HOME_Z_STEPPER){
    endstopZ.home(INVERSE_Z_STEPPER);
  }
  #if RAIL
    if (HOME_E0_STEPPER){
      endstopE0.home(!INVERSE_E0_STEPPER);
    }
  #endif
  interpolator.setInterpolation(INITIAL_X, INITIAL_Y, INITIAL_Z, INITIAL_E0, INITIAL_X, INITIAL_Y, INITIAL_Z, INITIAL_E0);
  Logger::logINFO("HOMING COMPLETE");
}

//DUE TO UNO CNC SHIELD LIMIT, 1 EN PIN SERVES 3 MOTORS, HENCE DIFFERENT HOMESEQUENCE IS REQUIRED
void robotHomeSequence_UNO(){
  #if GRIPPER == SERVO
  if (servo_gripper.readDegree() != SERVO_UNGRIP_DEGREE){
    servo_gripper.cmdOff();
  }
  #endif
  if (HOME_Y_STEPPER && HOME_X_STEPPER){
    while (!endstopY.state() || !endstopX.state()){
      endstopY.oneStepToEndstop(!INVERSE_Y_STEPPER);
      endstopX.oneStepToEndstop(!INVERSE_X_STEPPER);
    }
    endstopY.homeOffset(!INVERSE_Y_STEPPER);
    endstopX.homeOffset(!INVERSE_X_STEPPER);
  } else {
    robotSetStepperEnable(true);
    endstopY.homeOffset(!INVERSE_Y_STEPPER);
    endstopX.homeOffset(!INVERSE_X_STEPPER);
  }
  if (HOME_Z_STEPPER){
    endstopZ.home(INVERSE_Z_STEPPER); //INDICATE STEPPER HOMING DIRECDTION
  }
  #if RAIL
    if (HOME_E0_STEPPER){
      endstopE0.home(!INVERSE_E0_STEPPER);
    }
  #endif
  interpolator.setInterpolation(INITIAL_X, INITIAL_Y, INITIAL_Z, INITIAL_E0, INITIAL_X, INITIAL_Y, INITIAL_Z, INITIAL_E0);
  Logger::logINFO("HOMING COMPLETE");
}



