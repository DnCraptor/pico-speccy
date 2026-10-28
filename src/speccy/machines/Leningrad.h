// pico-speccy — Leningrad-1 (ZS Research, S. Zonov, 1988) and Leningrad-2: Soviet
// 48K clones, romsets R_48K_LEN1 / R_48K_LEN2 of A_48K on the stock Sinclair 48K ROM.
// FLASH-resident: the hot paths only test the byte globals below (CPU.h).
//
// Every number here was read off the schematics, not taken from an emulator:
//  * Leningrad-1 — micklab.ru Leningrad_sch (P-CAD netlist) cross-checked against the
//    original 1988 scan (Leningrad_original): DD3/DD4/DD8.1 line counter, DD5/DD6/DD8.2
//    frame counter, DD9.1 /WAIT, DD8.2 + C1/R2 INT shaper.
//  * Leningrad-2 — micklab.ru Leningrad_2sch (P-CAD) cross-checked against Prusak's
//    E3 drawing (the P-CAD file loses DD6.2 D = DD9.1 /Q).
//
// Common to both: 14 MHz crystal, 7 MHz pixel clock, CPU = 3.5 MHz, 448 px = 224 T
// per line, 312 lines = 69888 T per frame — the 48K numbers. One RAM bank of eight
// 565RU5, CPU and video alternate memory slots every T-state (H1), so there is no
// ULA-style contention anywhere; the CPU reads RAM through a latch (IR22) that is
// loaded in its slot by WRBUF = H0 & /H1, and /WAIT holds a read until that latch
// strobe — i.e. a RAM read whose T1 falls on the wrong half of the 2 T slot pair
// gets one extra T. Ports are partially decoded, there is no floating bus.
//
//                      Leningrad-1                  Leningrad-2
//  /WAIT on            M1 (opcode fetch) from RAM   every RAM read (M1 and data)
//  wait when T1 at     even T (INT = T 0)           odd T
//  INT position        start of the BOTTOM border   64 lines before the paper,
//                      (paper 120 lines later)      like the 48K
//  INT length          ~15 T (C1 10n / R2 470R)     32 T (DD9.2 reset by H6)
//  OUT                 any port -> #FE latch        A0=0 or A5=0 -> #FE latch
//  IN odd ports        any: Kempston, D5-D7 = 1     A5=0: Kempston, else #FF
//  IN even ports       keyboard (D5, D7 = 1)        keyboard, D5 = D7 = 0
//
// The Leningrad-1 paper sits 120 lines after its INT, so the bottom border lies
// after the frame end. VIDEO::EndFrame renders those lines at the frame end
// (frameEndFlush) with the state as it stands at the INT — exact for anything but
// border colour changes made during the first ~40 lines of the next frame.
#pragma once
#include <stdint.h>

namespace Leningrad {

// Paper start relative to the 48K's (the renderer's TS_SCREEN_48 anchor), in T.
// L1: 120 lines after INT + ~4 T of shift-register pipeline (DD33 -> DD35),
//     against the 48K's 64 lines.
// L2: 64 lines after INT, plus the 96 T border phase that opens every line (the
//     V counter steps at the start of the border phase), plus ~4 T of pipeline.
constexpr int kPaperDelta1 = 120 * 224 + 4 - 64 * 224;   // 12548
constexpr int kPaperDelta2 = 96 + 4;                      // 100
constexpr int kIntLen1 = 15;
constexpr int kIntLen2 = 32;

// CPU::reset / updateStatesInFrame: derive the hot-path globals (CPU.h) from the
// running machine. Clears them all off a Leningrad.
void apply();

// VIDEO::EndFrame, Leningrad-1 only: render the lines that fall past the frame end.
void frameEndFlush();

}
