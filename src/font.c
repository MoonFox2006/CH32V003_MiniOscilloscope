#include "font.h"

uint8_t font_strwidth(const char *str, bool x2) {
    uint8_t result = 0;

    while (*str++) {
        if (result)
            ++result; // Gap
        result += (FONT_WIDTH << x2);
    }
    return result;
}

inline __attribute__((always_inline)) uint8_t font_charheight(bool x2) {
    return FONT_HEIGHT << x2;
}
