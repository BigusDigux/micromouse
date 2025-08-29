#include "MPU.h"
#include <math.h>

#define MPU_ADDR (0x68 << 1)
#define ACCEL_CONFIG 0x1C
#define GYRO_CONFIG 0x1B
#define PWR_MGMT_1 0x6B
#define ACCEL_XOUT_H 0x3B
#define GYRO_XOUT_H  0x43

I2C_HandleTypeDef* MPU_hi2c;

static float GYRO_SENS_DPS = 32.8f;   // ±1000 dps -> 32.8 LSB per °/s

static float angle_z;
static float gyro_z;
static float gyro_z_offset = 0;

static uint32_t last_time = 0;
static float dt;

static int16_t gz;

// ===== Internal: Read raw gyro Z only =====
static void Read_MPU_GyroZ() {
    uint8_t data[2];
    HAL_I2C_Mem_Read(MPU_hi2c, MPU_ADDR, GYRO_XOUT_H + 4, 1, data, 2, 10);
    gz = (int16_t)(data[0] << 8 | data[1]);
}

// ===== Init =====
void MPU_Init(I2C_HandleTypeDef* hi2c) {
    MPU_hi2c = hi2c;

    uint8_t check;
    HAL_I2C_Mem_Read(MPU_hi2c, MPU_ADDR, 0x75, 1, &check, 1, 10);
    if (check != 0x68) return; // Not detected

    uint8_t data;

    data = 0x00;
    HAL_I2C_Mem_Write(MPU_hi2c, MPU_ADDR, PWR_MGMT_1, 1, &data, 1, 10);

    data = 0x00;
    HAL_I2C_Mem_Write(MPU_hi2c, MPU_ADDR, ACCEL_CONFIG, 1, &data, 1, 10);  // ±2g
    data = 0x00;
    HAL_I2C_Mem_Write(MPU_hi2c, MPU_ADDR, GYRO_CONFIG, 1, &data, 1, 10);   // ±250°/s

    last_time = HAL_GetTick();
}

// ===== Calibrate Gyro Z Drift =====
void MPU_CalibrateGyroZ() {
    int32_t sum_gz = 0;
    const int samples = 1000;

    for (int i = 0; i < samples; i++) {
        Read_MPU_GyroZ();
        sum_gz += gz;
        HAL_Delay(2);
    }

    gyro_z_offset = (float)sum_gz / samples;
}

// ===== Update Yaw =====
void MPU_Update() {
    uint32_t now = HAL_GetTick();
    dt = (now - last_time) / 1000.0f;
    last_time = now;

    Read_MPU_GyroZ();

    // Gyroscope rate (deg/sec) with offset correction
    gyro_z = (gz - gyro_z_offset) / 131.0f;

    // Integrate yaw
    angle_z += gyro_z * dt;

    // Wrap to -180..180
    if (angle_z > 180.0f) angle_z -= 360.0f;
    else if (angle_z < -180.0f) angle_z += 360.0f;
}

// ===== Get Yaw =====
float MPU_GetYaw() { return angle_z; }
