#ifndef __I2C3_H
#define __I2C3_H
#include "common.h"

void I2C3_Init(void);
void I2C3_WriteData(uint8_t address, uint8_t reg, uint8_t data);
uint8_t I2C3_ReadData(uint8_t address, uint8_t reg);

#endif
