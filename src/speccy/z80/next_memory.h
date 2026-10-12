// Next operand reads inside the Z80 core. Include after CPU, VIDEO, MemESP
// and Z80Ops are declared. Used in shared LD/ALU, DD/FD/CB and stack/branch
// handlers to bound SRAM growth; individual immediate handlers stay out of line.
// No opcode fetches: those need DivMMC automapping.
#pragma once
#include <stdint.h>

// Derived with the fast-memory gates: Next only, direct reads in all 8 slots,
// no memory breakpoints or NeoGS DMA overlay. Mapping changes update the slots.
extern bool g_next_fastrd;

namespace NextMemory {

__attribute__((always_inline)) inline void addressOnBus(uint16_t address, int32_t cycles) {
    // Match addressOnBus's counter-only fast path: internal cycles defer the
    // raster tick until the next memory/M1 access, even across a line boundary.
    if (g_next_fastrd) CPU::tstates += (uint32_t)cycles;
    else Z80Ops::addressOnBus(address, cycles);
}

__attribute__((always_inline)) inline uint8_t peek8(uint16_t address) {
    const uint32_t end = CPU::tstates + 3;
    if (g_next_fastrd && end < VIDEO::ts_line_t) {
        CPU::tstates = end;
        return MemESP::ramCurrent[address >> 13][address & 0x1FFF];
    }
    // Leave the entire access to CPU.cpp at a line boundary: the copper can
    // change the MMU during the tick, before the byte is read.
    return Z80Ops::peek8(address);
}

__attribute__((always_inline)) inline uint16_t peek16(uint16_t address) {
    const uint32_t end = CPU::tstates + 6;
    if (g_next_fastrd && end < VIDEO::ts_line_t) {
        CPU::tstates = end;
        const uint16_t next = address + 1;
        const uint8_t lo = MemESP::ramCurrent[address >> 13][address & 0x1FFF];
        const uint8_t hi = MemESP::ramCurrent[next >> 13][next & 0x1FFF];
        return (uint16_t)(lo | (hi << 8));
    }
    return Z80Ops::peek16(address);
}

// IR on the bus (INC/DEC rr, LD SP,HL, PUSH ...): on the fast path only the
// T-states, exactly as Z80Ops::addressOnBusIR does there.
__attribute__((always_inline)) inline void addressOnBusIR(int32_t cycles) {
    if (g_next_fastrd) CPU::tstates += (uint32_t)cycles;
    else Z80Ops::addressOnBusIR(cycles);
}

// A write at 0x4000-0xFFFF on the Next fast path: what Z80Ops::poke8's fast
// path does there (+3 T, no line tick until the next M1, a plain store) minus
// two things that cannot apply on the Next — the TS-Conf/ATM write gates (zero
// on this machine) and the ROM-pointer test (slots 2-7 never map ROM: the MMU
// puts ROM in slots 0/1 only) — and the dirty mark, which on the Next goes to
// mem_desc_t::dirty_sink. Everything below 0x4000 (Layer 2 write window, divMMC,
// ROM) keeps the full path. Used by the Next opcode switch only
// (tools/gen_next_dispatch.py), not by the shared handlers.
__attribute__((always_inline)) inline void poke8(uint16_t address, uint8_t value) {
    if (g_next_fastrd && address >= 0x4000) {
        CPU::tstates += 3;
        MemESP::ramCurrent[address >> 13][address & 0x1FFF] = value;
        return;
    }
    Z80Ops::poke8(address, value);
}

// ---- The gate-free twins, for Z80::exec_nocheck_next's pasted opcode bodies
// ONLY (tools/gen_next_dispatch.py substitutes them). That loop is entered
// with g_next_fastrd set, and every drop of the gate ends the slice: the
// generated TABLE cases and the prefix decoders test it right after the one
// kind of handler that can drop it (a port write), so inside a pasted body the
// gate is an invariant and its per-access load + test are not paid.
// The line-boundary test (peek) and the >= 0x4000 test (poke) stay.

__attribute__((always_inline)) inline void addressOnBus_nx(uint16_t, int32_t cycles) {
    CPU::tstates += (uint32_t)cycles;
}
__attribute__((always_inline)) inline void addressOnBusIR_nx(int32_t cycles) {
    CPU::tstates += (uint32_t)cycles;
}

__attribute__((always_inline)) inline uint8_t peek8_nx(uint16_t address) {
    const uint32_t end = CPU::tstates + 3;
    if (end < VIDEO::ts_line_t) {
        CPU::tstates = end;
        return MemESP::ramCurrent[address >> 13][address & 0x1FFF];
    }
    return Z80Ops::peek8(address);
}

// A word inside one 8K slot is one slot lookup; one straddling two keeps the
// per-byte form (the slots need not be adjacent in SRAM).
__attribute__((always_inline)) inline uint16_t peek16_nx(uint16_t address) {
    const uint32_t end = CPU::tstates + 6;
    if (end < VIDEO::ts_line_t) {
        CPU::tstates = end;
        const uint8_t* const p = MemESP::ramCurrent[address >> 13] + (address & 0x1FFF);
        if (__builtin_expect((address & 0x1FFF) != 0x1FFF, 1)) return (uint16_t)(p[0] | (p[1] << 8));
        const uint16_t next = address + 1;
        return (uint16_t)(p[0] | (MemESP::ramCurrent[next >> 13][next & 0x1FFF] << 8));
    }
    return Z80Ops::peek16(address);
}

__attribute__((always_inline)) inline void poke8_nx(uint16_t address, uint8_t value) {
    if (address >= 0x4000) {
        CPU::tstates += 3;
        MemESP::ramCurrent[address >> 13][address & 0x1FFF] = value;
        return;
    }
    Z80Ops::poke8(address, value);
}

// PUSH's two writes as one: low byte at `address`, high at `address + 1`,
// written HIGH FIRST like the two pokes it replaces (the order is visible only
// below 0x4000, which takes the per-byte path anyway).
__attribute__((always_inline)) inline void poke16_nx(uint16_t address, uint16_t value) {
    if (address >= 0x4000 && __builtin_expect((address & 0x1FFF) != 0x1FFF, 1)) {
        CPU::tstates += 6;
        uint8_t* const p = MemESP::ramCurrent[address >> 13] + (address & 0x1FFF);
        p[1] = (uint8_t)(value >> 8);
        p[0] = (uint8_t)value;
        return;
    }
    poke8_nx(address + 1, (uint8_t)(value >> 8));
    poke8_nx(address, (uint8_t)value);
}

} // namespace NextMemory
