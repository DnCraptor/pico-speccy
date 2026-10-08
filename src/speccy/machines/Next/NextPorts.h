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

    // Sound (TurboSound + DACs), NextPorts.cpp
    bool ayWrite(uint16_t address, uint8_t data);       // A15=1, A1:0=01 family
    void dacWrite(uint8_t mask, uint8_t v);             // bit 0 = A .. bit 3 = D
    void dacNr(uint8_t r, uint8_t v);                   // NR 0x2C-0x2E
    void audioRegWrite(uint8_t r, uint8_t old);         // NR 0x06/0x08/0x09 written
    void audioReset();                                  // machine reset (both types)
    void uartReset(bool hard);                          // UART select / frame / prescaler
}
