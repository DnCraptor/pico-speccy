#pragma once
#include <stdint.h>

// ZX Spectrum Next port decode, called once from Ports::input/output under
// Z80Ops::isNext (cold flash dispatch — the shared RAM decoders keep only the
// test and the call). Every port here honours its NR 0x82-0x85 enable bit
// (NextReg::portEn): an undecoded port falls through to the generic decoder,
// exactly as an FPGA that does not decode it lets the cycle go by.
namespace NextPorts {
    // -1 = not a Next port (the caller carries on), else the byte read
    int  input(uint16_t address);
    // true = consumed; the caller charges the I/O cycle and returns
    bool output(uint16_t address, uint8_t data);
}
