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

} // namespace NextMemory
