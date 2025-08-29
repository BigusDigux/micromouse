#ifndef INIT_H
#define INIT_H

#include "stm32f1xx_hal.h"
#include "config.h"
#include "OLED.h"
#include "encoder.h"
#include "vl6180x.h"
#include "drv8833.h"
#include "buzzer.h"
#include "MPU.h"
#include "floodfill.h"

/* Extern handles */
extern I2C_HandleTypeDef hi2c1;
extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim2;

/* Extern devices */
extern VL6180X tofLeft, tofFront, tofRight;
extern Motor_HandleTypeDef motorL, motorR;

/* Init entry point */
void System_Init(void);

/* Error handler */
void Error_Handler(void);

#endif // INIT_H
