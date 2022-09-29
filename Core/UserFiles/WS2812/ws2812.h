/*
 * ws2812.h
 *
 *  Created on: Sep 22, 2022
 *      Author: maxfrankenhauser
 */

#ifndef USERFILES_WS2812_WS2812_H_
#define USERFILES_WS2812_WS2812_H_

#include "main.h"

#define PI 3.14159265

#define MAX_LED 20
#define USE_BRIGHTNESS 0

extern TIM_HandleTypeDef htim2;
extern DMA_HandleTypeDef hdma_tim2_ch1;

static uint8_t datasentflag = 0;

uint8_t LED_Data[MAX_LED][4];
uint8_t LED_Mod[MAX_LED][4];  // for brightness


uint16_t pwmData[(24*MAX_LED)+50];


void Set_LED (int LEDnum, int Red, int Green, int Blue);
void Set_Brightness (int brightness) ;
void WS2812_Send (void);
void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef *htim);




#endif /* USERFILES_WS2812_WS2812_H_ */















