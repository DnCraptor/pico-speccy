// Next prefix-byte fetch. Include after Z80, Z80Ops, CPU, VIDEO, MemESP and
// DivMMC. The first M1 uses exec_nocheck_next; CB/DD/ED/FD use this second M1.
#pragma once
#include <stdint.h>

extern bool g_next_fastrd;

namespace NextOpcode {
__attribute__((always_inline)) inline uint8_t fetch() {
#if !DEBUG
    const uint32_t end = CPU::tstates + 4;
    if (g_next_fastrd && end < VIDEO::ts_line_t) {
        const uint16_t pc = Z80::getRegPC();
        CPU::tstates = end;
        if (DivMMC::enabled) return DivMMC::nextOpcode(pc);
        return MemESP::ramCurrent[pc >> 13][pc & 0x1FFF];
    }
#endif
    // Let the original fetch process a raster/MMU change before loading the
    // byte. Debug builds also keep its dbg_last_pc instrumentation.
    return Z80Ops::fetchOpcode();
}
}
