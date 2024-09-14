#pragma once

#include <stdint.h>
#include <stdbool.h>

#define TWI_FLAG_AUTOSTOP   1
#define TWI_FLAG_BUSY       2

typedef enum { TWI_OK, TWI_BUSY, TWI_TIMEOUT, TWI_NACK } twi_err;

void TWI_Init(uint32_t speed);
twi_err TWI_Start(uint8_t addr, bool receiver);
void TWI_Stop();
int16_t TWI_Read(bool last);
uint16_t TWI_Reads(uint8_t *buf, uint16_t size, bool last);
void TWI_ReadsAsync(uint8_t *buf, uint16_t size, bool autostop);
bool TWI_Write(uint8_t data);
uint16_t TWI_Writes(const uint8_t *buf, uint16_t size);
void TWI_WritesAsync(const uint8_t *buf, uint16_t size, bool autostop);
bool TWI_Asynced();
