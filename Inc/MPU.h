#ifndef MPU_H
#define MPU_H

#include "stm32f1xx_hal.h"

// ===== Initialization =====
void MPU_Init(I2C_HandleTypeDef* hi2c);

// ===== Calibration =====
// Call once at startup (robot must be still)
void MPU_CalibrateGyroZ(void);

// ===== Update & Read =====
// Call periodically (e.g., every loop iteration)
void MPU_Update(void);

// Returns yaw angle in degrees (-180 to +180)
float MPU_GetYaw(void);

#endif // MPU_H
