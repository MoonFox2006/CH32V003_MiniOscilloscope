#pragma once

#include "font.h"

#define SCREEN_WIDTH    128
#define SCREEN_HEIGHT   64

void screen_clear();
void screen_pixel(uint8_t x, uint8_t y, bool color);
void screen_drawpattern(uint8_t x, uint8_t y, uint8_t w, uint8_t h, uint8_t pattern);
void screen_drawpatterns(uint8_t x, uint8_t y, uint8_t w, uint8_t h, const uint8_t *pattern, bool inverse);
void screen_fillrect(uint8_t x, uint8_t y, uint8_t w, uint8_t h, bool color);
void screen_hline(uint8_t x, uint8_t y, uint8_t w, bool color);
void screen_vline(uint8_t x, uint8_t y, uint8_t h, bool color);
void screen_line(uint8_t x1, uint8_t y1, uint8_t x2, uint8_t y2, bool color);
void screen_printchar(char c, uint8_t x, uint8_t y, bool color);
void screen_printchar_x2(char c, uint8_t x, uint8_t y, bool color);
void screen_printstr(const char *str, uint8_t x, uint8_t y, bool color);
void screen_printstr_x2(const char *str, uint8_t x, uint8_t y, bool color);
