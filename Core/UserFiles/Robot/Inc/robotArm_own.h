#ifndef ROBOTARM_OWN_H_
#define ROBOTARM_OWN_H_

//Max und Fabi
#include "command.h"
#ifdef __cplusplus

void robotExecuteCommand(r_Cmd cmd);
void robotSetup();
void robotLoop();
void robotSetStepperEnable(uint8_t enable);
void robotHomeSequence();

#endif
#endif



