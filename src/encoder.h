#pragma once

#include <stdbool.h>
#include <ch32v00x.h>

#define ENC_GPIO    GPIOC
#define ENC_CLK     6
#define ENC_DT      7
#define ENC_BTN     3

void Encoder_Init();
int8_t Encoder_Read();
bool Encoder_Button();
