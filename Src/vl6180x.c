#include "vl6180x.h"
#include "main.h"
#include "stdio.h"
#include "string.h"

uint8_t VL6180X_ReadRegister(VL6180X *dev, uint16_t reg) {
    uint8_t tx[2] = { reg >> 8, reg & 0xFF };
    uint8_t rx;

    HAL_I2C_Master_Transmit(dev->hi2c, dev->address, tx, 2, HAL_MAX_DELAY);
    HAL_I2C_Master_Receive(dev->hi2c, dev->address, &rx, 1, HAL_MAX_DELAY);

    return rx;
}

HAL_StatusTypeDef VL6180X_WriteRegister(VL6180X *dev, uint16_t reg, uint8_t value) {
    uint8_t data[3] = { reg >> 8, reg & 0xFF, value };
    return HAL_I2C_Master_Transmit(dev->hi2c, dev->address, data, 3, HAL_MAX_DELAY);
}

uint8_t VL6180X_Init(VL6180X *dev, I2C_HandleTypeDef *hi2c, uint8_t address) {
    dev->hi2c = hi2c;
    dev->address = address;

    uint8_t id = VL6180X_ReadRegister(dev, 0x000);
    if (id != 0xB4) return 0;

    // Initialization sequence (from Adafruit code)
    VL6180X_WriteRegister(dev, 0x0207, 0x01);
    VL6180X_WriteRegister(dev, 0x0208, 0x01);
    VL6180X_WriteRegister(dev, 0x0096, 0x00);
    VL6180X_WriteRegister(dev, 0x0097, 0xfd);
    VL6180X_WriteRegister(dev, 0x00e3, 0x00);
    VL6180X_WriteRegister(dev, 0x00e4, 0x04);
    VL6180X_WriteRegister(dev, 0x00e5, 0x02);
    VL6180X_WriteRegister(dev, 0x00e6, 0x01);
    VL6180X_WriteRegister(dev, 0x00e7, 0x03);
    VL6180X_WriteRegister(dev, 0x00f5, 0x02);
    VL6180X_WriteRegister(dev, 0x00d9, 0x05);
    VL6180X_WriteRegister(dev, 0x00db, 0xce);
    VL6180X_WriteRegister(dev, 0x00dc, 0x03);
    VL6180X_WriteRegister(dev, 0x00dd, 0xf8);
    VL6180X_WriteRegister(dev, 0x009f, 0x00);
    VL6180X_WriteRegister(dev, 0x00a3, 0x3c);
    VL6180X_WriteRegister(dev, 0x00b7, 0x00);
    VL6180X_WriteRegister(dev, 0x00bb, 0x3c);
    VL6180X_WriteRegister(dev, 0x00b2, 0x09);
    VL6180X_WriteRegister(dev, 0x00ca, 0x09);
    VL6180X_WriteRegister(dev, 0x0198, 0x01);
    VL6180X_WriteRegister(dev, 0x01b0, 0x17);
    VL6180X_WriteRegister(dev, 0x01ad, 0x00);
    VL6180X_WriteRegister(dev, 0x00ff, 0x05);
    VL6180X_WriteRegister(dev, 0x0100, 0x05);
    VL6180X_WriteRegister(dev, 0x0199, 0x05);
    VL6180X_WriteRegister(dev, 0x01a6, 0x1b);
    VL6180X_WriteRegister(dev, 0x01ac, 0x3e);
    VL6180X_WriteRegister(dev, 0x01a7, 0x1f);
    VL6180X_WriteRegister(dev, 0x0030, 0x00);

    return 1;
}

uint8_t VL6180X_ReadRange(VL6180X *dev) {
    VL6180X_WriteRegister(dev, 0x0018, 0x01); // SYSRANGE_START
    HAL_Delay(10);
    if (VL6180X_ReadRegister(dev, 0x0062) >= 255){
        return 0xB4;
    } else {
        return VL6180X_ReadRegister(dev, 0x0062);
    }
}

uint8_t VL6180X_ReadAverage(VL6180X *dev, uint8_t samples) {
    uint32_t sum = 0;
    for (uint8_t i = 0; i < samples; i++) {
        sum += VL6180X_ReadRange(dev);
        HAL_Delay(5); // small delay between samples
    }
    return (uint8_t)(sum / samples);
}

void VL6180X_SetI2CAddress(VL6180X *dev, uint8_t new_address)
{
    uint8_t data = new_address >> 1;  // 7-bit address
    HAL_I2C_Mem_Write(dev->hi2c, dev->address, 0x212, 2, &data, 1, 100);
    dev->address = new_address;
}

float VL6180X_ReadFilteredRange(VL6180X *dev, float alpha) {
    uint8_t raw = VL6180X_ReadRange(dev);
    float filtered = alpha * raw + (1.0f - alpha) * dev->smoothedRange;
    dev->smoothedRange = filtered;
    return filtered;
}
