// pico-speccy — MGT SAM Coupe (A_SAM). Phase 0: ROM 3.0 boots to SAM BASIC.
//
// Model: SimCoupe (github.com/simonowen/simcoupe, Base/SAMIO.cpp, Memory.cpp,
// Frame.cpp) and the SAM Technical Manual (sam-users/sam-coupe-datasheets).
// Plan and status: docs/sam-coupe-plan.md.
//
// Timing: Z80 at 6 MHz, 384 T per line, 312 lines = 119808 T per frame (50.08 Hz,
// the period of the 48K frame — so the 48K video modes and 624 audio samples apply).
// Frame INT at T = 0, 128 T long; STATUS (#F9) bit 3 is low while it is up.
//
// Memory: four 16K sections. A = ROM0 (LMPR D5 = 0) or RAM LMPR&31; B = RAM LMPR+1;
// C = RAM HMPR&31, or external LEPR when HMPR D7; D = ROM1 (LMPR D6), external HEPR
// (HMPR D7) or RAM HMPR+1. Internal page n is ram[n], external page n ram[32 + n].
// Pages the romset does not fit read 0xFF and drop writes. LMPR D7 write-protects A.
//
// Ports (low byte): #FE keyboard/border/beeper, #F9 status/line, #FA LMPR, #FB HMPR,
// #FC VMPR, #F8 CLUT (index = A11..A8), #FF/#1FF SAA1099, #80 LEPR, #81 HEPR, #1F
// Kempston. Line INT: STATUS bit 0, for LINE < 192 at T = (LINE + 68) * 384, 128 T
// long (SimCoupe Events.cpp / SAMIO.cpp LINE_PORT). Not yet: contention, WD1772 drives, mouse, MIDI, clock, DACs.
#pragma once

#include <stdint.h>

namespace Sam {
    extern uint8_t lmpr, hmpr, vmpr, lepr, hepr, border, line, status, attr;
    extern uint8_t clut[16];
    extern volatile bool clutDirty;
    // Keyboard matrix, active low: rows 0..7 bits 0..7 (bits 5..7 = the SAM extra
    // keys), row 8 bits 0..4 (CNTRL, UP, DOWN, LEFT, RIGHT). Written by the host
    // keyboard scan (ESPectrum::processKeyboard), read by #FE / #F9.
    extern uint8_t keys[9];

    void bindRoms();                 // Config::requestMachine
    void reset();                    // CPU::reset
    void remap();                    // sections A..D -> MemESP::ramCurrent

    uint8_t portIn(uint16_t address);
    void portOut(uint16_t address, uint8_t data);

    // Whole frame into VIDEO::vga.frameBuffer (EndFrame).
    void renderFrame();

    // Line interrupt window (SimCoupe: TOP_BORDER_LINES 68, CPU_CYCLES_INT_ACTIVE 128).
    static constexpr uint32_t T_PER_LINE = 384, TOP_LINES = 68, INT_LEN = 128;
    inline uint32_t lineIntStart() { return line < 192 ? ((uint32_t)line + TOP_LINES) * T_PER_LINE : 0xFFFFFFFFu; }
    bool intLine();             // Z80Ops::isActiveINT: frame OR line window (flash)
    inline bool lineIntUp(uint32_t t) { const uint32_t s = lineIntStart(); return t >= s && t < s + INT_LEN; }
    // 7-bit SAM colour -> RGB888 (SimCoupe IO::Palette, full intensity).
    uint32_t colourRgb(uint8_t v);
}
