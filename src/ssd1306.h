#pragma once

#include "screen.h"

bool ssd1306_begin();
bool ssd1306_flip(bool on);
bool ssd1306_invert(bool on);
bool ssd1306_power(bool on);
bool ssd1306_contrast(uint8_t value);
bool ssd1306_flush(bool wait);
