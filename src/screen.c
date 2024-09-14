#include <string.h>
#include "screen.h"

uint8_t _screen[SCREEN_WIDTH * (SCREEN_HEIGHT / 8)];

static void update_byte(uint8_t *data, uint8_t and_mask, uint8_t or_mask) {
    *data &= and_mask;
    *data |= or_mask;
}

static inline uint8_t abs8(int8_t value) {
    return value < 0 ? -value : value;
}

static inline int8_t sign8(int8_t value) {
    return value < 0 ? -1 : value > 0 ? 1 : 0;
}

inline __attribute__((always_inline)) void screen_clear() {
    memset(_screen, 0, sizeof(_screen));
}

void screen_pixel(uint8_t x, uint8_t y, bool color) {
    uint16_t offset = SCREEN_WIDTH * (y >> 3) + x;

    if (color)
        _screen[offset] |= (1 << (y & 0x07));
    else
        _screen[offset] &= ~(1 << (y & 0x07));
}

void screen_drawpattern(uint8_t x, uint8_t y, uint8_t w, uint8_t h, uint8_t pattern) {
    uint8_t offset, mask, data;

    offset = y & 7;
    if (offset) { // First row is incomplete
        mask = 0xFF;
        if (h < 8)
            mask >>= (8 - h);
        mask <<= offset;
        data = pattern << offset;
        for (uint8_t _x = x; (_x < x + w) && (_x < SCREEN_WIDTH); ++_x) {
            update_byte(&_screen[SCREEN_WIDTH * (y >> 3) + _x], ~mask, data & mask);
        }
        if (h <= 8 - offset) // Last row
            return;
        h -= (8 - offset);
        y += (8 - offset);
    }
    while (y < SCREEN_HEIGHT) {
        mask = 0xFF;
        if (h < 8)
            mask >>= (8 - h);
        if (offset)
            data = (pattern >> (8 - offset)) | (pattern << offset);
        else
            data = pattern;
        for (uint8_t _x = x; (_x < x + w) && (_x < SCREEN_WIDTH); ++_x) {
            update_byte(&_screen[SCREEN_WIDTH * (y >> 3) + _x], ~mask, data & mask);
        }
        if (h <= 8) // Last row
            break;
        h -= 8;
        y += 8;
    }
}

void screen_drawpatterns(uint8_t x, uint8_t y, uint8_t w, uint8_t h, const uint8_t *pattern, bool inverse) {
    uint8_t offset, mask, data;

    offset = y & 7;
    if (offset) { // First row is incomplete
        mask = 0xFF;
        if (h < 8)
            mask >>= (8 - h);
        mask <<= offset;
        for (uint8_t _x = x; (_x < x + w) && (_x < SCREEN_WIDTH); ++_x) {
            data = pattern[_x - x] << offset;
            if (inverse)
                data = ~data;
            update_byte(&_screen[SCREEN_WIDTH * (y >> 3) + _x], ~mask, data & mask);
        }
        if (h <= 8 - offset) // Last row
            return;
        h -= (8 - offset);
        y += (8 - offset);
    }
    while (y < SCREEN_HEIGHT) {
        mask = 0xFF;
        if (h < 8)
            mask >>= (8 - h);
        for (uint8_t _x = x; (_x < x + w) && (_x < SCREEN_WIDTH); ++_x) {
            if (offset)
                data = (pattern[_x - x] >> (8 - offset)) | (pattern[w + _x - x] << offset);
            else
                data = pattern[_x - x];
            if (inverse)
                data = ~data;
            update_byte(&_screen[SCREEN_WIDTH * (y >> 3) + _x], ~mask, data & mask);
        }
        if (h <= 8) // Last row
            break;
        h -= 8;
        y += 8;
        pattern += w;
    }
}

inline __attribute__((always_inline)) void screen_fillrect(uint8_t x, uint8_t y, uint8_t w, uint8_t h, bool color) {
    screen_drawpattern(x, y, w, h, (uint8_t)(color ? 0xFF : 0x00));
}

inline __attribute__((always_inline)) void screen_hline(uint8_t x, uint8_t y, uint8_t w, bool color) {
    screen_drawpattern(x, y, w, 1, (uint8_t)(color ? 0xFF : 0x00));
}

inline __attribute__((always_inline)) void screen_vline(uint8_t x, uint8_t y, uint8_t h, bool color) {
    screen_drawpattern(x, y, 1, h, (uint8_t)(color ? 0xFF : 0x00));
}

void screen_line(uint8_t x1, uint8_t y1, uint8_t x2, uint8_t y2, bool color) {
    if (x1 == x2)
        screen_vline(x1, y1, abs8(y2 - y1) + 1, color);
    else if (y1 == y2)
        screen_hline(x1, y1, abs8(x2 - x1) + 1, color);
    else {
        bool steep;
        uint8_t dX, dY;
        int8_t err, yStep;

        steep = abs8(y2 - y1) > abs8(x2 - x1);
        if (steep) {
            uint8_t t;

            t = x1;
            x1 = y1;
            y1 = t;
            t = x2;
            x2 = y2;
            y2 = t;
        }
        if (x2 < x1) {
            uint8_t t;

            t = x1;
            x1 = x2;
            x2 = t;
            t = y1;
            y1 = y2;
            y2 = t;
        }
        dX = x2 - x1;
        dY = abs8(y2 - y1);
        err = dX / 2;
        yStep = sign8(y2 - y1);
        while (x1 <= x2) {
            if (steep) {
                screen_pixel(y1, x1, color);
            } else {
                screen_pixel(x1, y1, color);
            }
            err -= dY;
            if (err < 0) {
                y1 += yStep;
                err += dX;
            }
            ++x1;
        }
    }
}

void screen_printchar(char c, uint8_t x, uint8_t y, bool color) {
    if (c < ' ')
        c = ' ';
    screen_drawpatterns(x, y, FONT_WIDTH, FONT_HEIGHT, &FONT[(c - ' ') * FONT_WIDTH], ! color);
}

void screen_printchar_x2(char c, uint8_t x, uint8_t y, bool color) {
    uint8_t glyph[FONT_WIDTH * 2 * ((FONT_HEIGHT * 2 + 7) / 8)];

    if (c < ' ')
        c = ' ';
    for (uint8_t i = 0; i < FONT_WIDTH; ++i) {
        uint8_t src = FONT[(c - ' ') * FONT_WIDTH + i];
        uint16_t dest = 0;

        for (int8_t j = 0; j < FONT_HEIGHT; ++j) {
            if ((src >> j) & 0x01)
                dest |= (0x03 << (j * 2));
        }
        glyph[i * 2] = (uint8_t)dest;
        glyph[i * 2 + 1] = (uint8_t)dest;
        glyph[(i + FONT_WIDTH) * 2] = (uint8_t)(dest >> 8);
        glyph[(i + FONT_WIDTH) * 2 + 1] = (uint8_t)(dest >> 8);
    }
    screen_drawpatterns(x, y, FONT_WIDTH * 2, FONT_HEIGHT * 2, glyph, ! color);
}

void screen_printstr(const char *str, uint8_t x, uint8_t y, bool color) {
    while (*str && (x < SCREEN_WIDTH)) {
        screen_printchar(*str++, x, y, color);
        x += FONT_WIDTH;
        if (*str && (x < SCREEN_WIDTH))
            screen_drawpattern(x++, y, 1, FONT_HEIGHT, (uint8_t)(color ? 0 : 0xFF));
    }
}

void screen_printstr_x2(const char *str, uint8_t x, uint8_t y, bool color) {
    while (*str && (x < SCREEN_WIDTH)) {
        screen_printchar_x2(*str++, x, y, color);
        x += FONT_WIDTH * 2;
        if (*str && (x < SCREEN_WIDTH))
            screen_drawpattern(x++, y, 1, FONT_HEIGHT * 2, (uint8_t)(color ? 0 : 0xFF));
    }
}
