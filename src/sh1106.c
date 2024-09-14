#include "twi.h"
#include "sh1106.h"

#define SH1106_ADDR 0x3C

extern uint8_t _screen[SCREEN_WIDTH * (SCREEN_HEIGHT / 8)];

static bool sendcommand(uint8_t cmd) {
    bool result;

    if ((result = TWI_Start(SH1106_ADDR, false) == TWI_OK)) {
        result = TWI_Write(0x00) && TWI_Write(cmd);
        TWI_Stop();
    }
    return result;
}

static bool sendcommands(const uint8_t *cmds, uint8_t size) {
    bool result;

    if ((result = TWI_Start(SH1106_ADDR, false) == TWI_OK)) {
        result = TWI_Write(0x00) && (TWI_Writes(cmds, size) == size);
        TWI_Stop();
    }
    return result;
}

static bool sendpage(uint8_t y, const uint8_t *page) {
    uint8_t cmds[3];
    bool result;

    cmds[0] = 0x10 | (0 >> 4);
    cmds[1] = 0 & 0x0F;
    cmds[2] = 0xB0 | (y & 0x07);

    if ((result = sendcommands(cmds, sizeof(cmds)))) {
        if ((result = TWI_Start(SH1106_ADDR, false) == TWI_OK)) {
            if ((result = TWI_Write(0x40))) {
                for (uint8_t i = 0; i < SCREEN_WIDTH; ++i) {
                    if (! (result = TWI_Write(page[i])))
                        break;
                }
            }
            TWI_Stop();
        }
    }
    return result;
}

bool sh1106_begin() {
    const uint8_t CMDS[] = {
        0xAE, // DISPLAYOFF
        0xD5, 0x80, // SETDISPLAYCLOCKDIV = 0x80
        0xA8, 0x3F, // SETMULTIPLEX
        0xD3, 0x00, // SETDISPLAYOFFSET = 0x00
        0x40 | 0, // SETSTARTLINE = 0
        0x8D, 0x14, // CHARGEPUMP = 0x14
        0xA1, // SEGREMAP = reverse
        0xC8, // COMSCANDEC
        0xDA, 0x12, // SETCOMPINS
        0xD9, 0x22, // SETPRECHARGE = 0x22
        0xDB, 0x20, // SETVCOMDETECT
        0x20, 0x00, // MEMORYMODE = HORIZONTAL_ADDRESSING_MODE
        0x81, 0x80, // SETCONTRAST
        0xA4, // DISPLAYALLON_RESUME
        0xA6, // NORMALDISPLAY
        0xAF // DISPLAYON
    };

    screen_clear();
    return sendcommands(CMDS, sizeof(CMDS));
}

bool sh1106_flip(bool on) {
    const uint8_t CMDS[4] = {
        0xA0, // SEGREMAP = normal
        0xC0, // COMSCANINC
        0xA1, // SEGREMAP = reverse
        0xC8 // COMSCANDEC
    };

    return sendcommands(&CMDS[on ? 0 : 2], 2);
}

inline __attribute__((always_inline)) bool sh1106_invert(bool on) {
    return sendcommand(0xA6 | on); // NORMALDISPLAY/INVERSEDISPLAY
}

inline __attribute__((always_inline)) bool sh1106_power(bool on) {
    return sendcommand(0xAE | on); // DISPLAYON/DISPLAYOFF
}

bool sh1106_contrast(uint8_t value) {
    uint8_t cmds[2];

    cmds[0] = 0x81; // SETCONTRAST
    cmds[1] = value;
    return sendcommands(cmds, sizeof(cmds));
}

bool sh1106_flush(bool wait) {
    for (uint8_t y = 0; y < 8; ++y) {
        if (! sendpage(y, &_screen[SCREEN_WIDTH * y]))
            return false;
    }
    return true;
}
