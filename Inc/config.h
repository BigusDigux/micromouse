#ifndef CONFIG_H
#define CONFIG_H

#include "stm32f1xx_hal.h"  // or your specific HAL header

/*=========================== Menu ===========================*/
#define MAIN_MENU_COUNT    4
#define ENCODER_STEP       50

/*=========================== Motion =========================*/
#define LOOP_DT_MS         1
#define TICKS_PER_CELL     440
#define TICK_FAST		   30
#define TICKS_PER_TURN     125

#define SPEED_MEDIUM       175
#define SPEED_FAST         225
#define SPEED_MIN         -255
#define SPEED_MAX          255

#define TURN_BASE_SPEED    150

/*=========================== Buttons ========================*/
#define BTN_CONFIRM_PORT   GPIOB
#define BTN_CONFIRM_PIN    GPIO_PIN_13

#define BTN_BACK_PORT      GPIOB
#define BTN_BACK_PIN       GPIO_PIN_12

#define BTN_RUN_PORT      GPIOA
#define BTN_RUN_PIN       GPIO_PIN_6

/*=========================== Sensors ========================*/
#define ADDR_LEFT          0x29
#define ADDR_FRONT         0x31
#define ADDR_RIGHT         0x33

#define XSHUT_LEFT_PORT    GPIOC
#define XSHUT_LEFT_PIN     GPIO_PIN_14

#define XSHUT_FRONT_PORT   GPIOB
#define XSHUT_FRONT_PIN    GPIO_PIN_9

#define XSHUT_RIGHT_PORT   GPIOB
#define XSHUT_RIGHT_PIN    GPIO_PIN_4

#define SENSOR_FRONT_LIMIT 150
#define SENSOR_SIDE_LIMIT   45

#endif // CONFIG_H
