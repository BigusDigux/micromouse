#ifndef VL6180X_H
#define VL6180X_H

#include "stm32f1xx_hal.h" // Change if you're using a different STM32 family

#define VL6180X_DEFAULT_I2C_ADDR 0x29 << 1  // STM32 HAL uses 8-bit addressing

typedef struct {
    I2C_HandleTypeDef *hi2c;
    uint8_t address;
    float smoothedRange;  // <-- NEW: filtered range value
} VL6180X;


uint8_t VL6180X_Init(VL6180X *dev, I2C_HandleTypeDef *hi2c, uint8_t address);
uint8_t VL6180X_ReadRange(VL6180X *dev);
uint8_t VL6180X_ReadAverage(VL6180X *dev, uint8_t samples);
uint8_t VL6180X_ReadRegister(VL6180X *dev, uint16_t reg);
HAL_StatusTypeDef VL6180X_WriteRegister(VL6180X *dev, uint16_t reg, uint8_t value);
void VL6180X_SetI2CAddress(VL6180X *dev, uint8_t new_address);

#endif
