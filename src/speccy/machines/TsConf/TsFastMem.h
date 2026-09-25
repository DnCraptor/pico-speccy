// TS-Conf fast guest-memory path (2026-09-06).
//
// While a TS-Conf whole-line video mode is live the per-access video hook is a
// pure T-state counter (VIDEO::TsDraw), every CPU bank is a plain POINTER page
// (TS-BIOS ROM included, via TsConf::romPtr), and there are no ROM overlays, no
// DivMMC window and no ProfROM tap. The generic Z80Ops accessors still walked
// all of those tests plus an indirect Draw call on EVERY byte — ~45 ARM
// instructions per guest read against ~12 here. g_ts_fastmem is recomputed by
// VIDEO::tsFastMemRecalc() at the per-frame Draw arming (EndFrame) and whenever
// a condition that must switch it off mid-frame changes (NeoGS ZX-DMA window,
// debugger memory breakpoints). Zero on every other machine, so the cost there
// is one byte load + branch per access.
#pragma once
#include <stdint.h>
#include "speccy/z80/CPU.h"
#include "speccy/video/Video.h"

extern uint8_t g_ts_fastmem;
// g_ts_fastmem values: 0 = off, 2 = TS-Conf (every address), 1 = Spectrum Next
// (0x4000-0xFFFF only). Everything the Next adds to guest memory — the divMMC
// window, the Layer 2 write window (port 0x123B), the write-protected NextZXOS
// ROM and its Alt-ROM capture, the TBBLUE config window, ROM overlays — lives
// in slots 0/1, so above 0x4000 its banks are exactly TS-Conf's case: plain
// pointer pages with a T-state-counter video hook (NEXTVID shares ts_line_t,
// and tsDrawTick hands the crossing to NEXTVID::ScanlineWork). One compare
// against the address keeps TS-Conf's path the same length.
static inline bool tsFastLo8(uint16_t a)  { return a < 0x4000; }
// 16-bit access: the second byte of 0xFFFF wraps to 0x0000 (slot 0).
static inline bool tsFastLo16(uint16_t a) { return (uint16_t)(a - 0x4000) >= 0xBFFF; }

static inline void tsFastTick(uint32_t n) {
    CPU::tstates += n;
    if (__builtin_expect(CPU::tstates >= VIDEO::ts_line_t, 0)) VIDEO::tsDrawTick();
}
