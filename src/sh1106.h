#pragma once

#include "screen.h"

bool sh1106_begin();
bool sh1106_flip(bool on);
bool sh1106_invert(bool on);
bool sh1106_power(bool on);
bool sh1106_contrast(uint8_t value);
bool sh1106_flush(bool wait);
