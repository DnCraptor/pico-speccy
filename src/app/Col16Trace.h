// COL16_TRACE (CMake, default OFF): which of ZX pages 0-7 a Pentagon 16col
// title actually touches, and how often it flips #7FFD D3 / re-pages window 3.
// The data the "keep only two of pages 4-7 in SRAM" decision needs. One
// `[C16]` line every 60 frames from VIDEO::EndFrame. Costs a page lookup per
// guest access, so a trace build is not a speed measurement.
#pragma once

#ifndef COL16_TRACE
#define COL16_TRACE 0
#endif

#if COL16_TRACE
#include <stdint.h>
#include "speccy/core/MemESP.h"

extern uint32_t c16_rd[9], c16_wr[9], c16_fe[9];   // [0..7] = ram page, [8] = ROM/other
extern uint32_t c16_flip, c16_bank;                // #7FFD D3 changes, window-3 page changes

static inline __attribute__((always_inline)) uint8_t c16pg(uint16_t a) {
    switch (a >> 14) {
        case 0:  return MemESP::page0ram ? 0 : 8;
        case 1:  return 5;
        case 2:  return 2;
        default: return MemESP::bankLatch < 8 ? (uint8_t)MemESP::bankLatch : 8;
    }
}
#define C16_RD(a) (c16_rd[c16pg(a)]++)
#define C16_WR(a) (c16_wr[c16pg(a)]++)
#define C16_FE(a) (c16_fe[c16pg(a)]++)
#define C16_FLIP() (c16_flip++)
#define C16_BANK() (c16_bank++)
#else
#define C16_RD(a) do {} while (0)
#define C16_WR(a) do {} while (0)
#define C16_FE(a) do {} while (0)
#define C16_FLIP() do {} while (0)
#define C16_BANK() do {} while (0)
#endif
