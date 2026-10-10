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

#include "speccy/z80/MemGates.h"   // g_ts_fastmem
// The READ twin (peek8/peek16 — operand bytes included — test this one; pokes,
// the opcode fetch and addressOnBus test g_ts_fastmem). Same encoding. TS-Conf:
// always equal to g_ts_fastmem. Next: 2 (every address) when the NextZXOS ROM
// set came from SD — reads below 0x4000 are then a plain ramCurrent[] load,
// exactly TS-Conf's case, the divMMC window included (it is plugged into
// ramCurrent[0/1] while paged in, MemESP::nextDivPlug) — else 1. Writes stay
// at 1 there (ROM write protect, Layer 2 write window, Alt-ROM capture, MAPRAM).
// The opcode fetch below 0x4000 takes the load too, with the divMMC automap
// hooks kept around it (Z80::exec_nocheck). Re-derived by
// VIDEO::tsFastMemRecalc and on every divMMC mapping change
// (DivMMC::applyMapping -> ts_fastrd_recalc).
extern uint8_t g_ts_fastrd;
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

// always_inline: at -Os (the SRAM core) GCC emitted this out of line and CALLED
// it on every fast fetch/access (PC profile 2026-09-25: ~6% of core0 time).
__attribute__((always_inline)) static inline void tsFastTick(uint32_t n) {
    CPU::tstates += n;
    if (__builtin_expect(CPU::tstates >= VIDEO::ts_line_t, 0)) VIDEO::tsDrawTick();
}
