/*

pico-speccy: ZX Spectrum Next (TBBlue) NextReg register file

Access from Z80 via port 0x243B (select) / 0x253B (data) and the Z80N
NEXTREG opcodes (ED 91 / ED 92). Legacy Spectrum paging ports
(0x7FFD/0xDFFD/0x1FFD) are implemented on top of the MMU, as on real
hardware. Port 0x123B is the Layer 2 write-through window.

Reference: https://wiki.specnext.dev/TBBlue_Register_Select

*/

#ifndef NEXTREG_h
#define NEXTREG_h

#include <inttypes.h>

class NextReg {
public:
    static uint8_t reg[256];     // raw latched values
    static uint8_t selected;     // port 0x243B latch

    // Legacy paging port latches (Next keeps them readable via nextregs)
    static uint8_t port7FFD;
    static uint8_t portDFFD;
    static uint8_t port1FFD;
    static uint8_t port123B;     // Layer 2 access port

    // Line interrupt state (nextreg 0x22/0x23)
    static bool     lineIrqEnabled;
    static bool     ulaIrqDisabled;
    static uint16_t lineIrqLine;

    static void reset(bool hard);
    static void write(uint8_t r, uint8_t v);
    static uint8_t read(uint8_t r);

    // Legacy paging on Next (called from Ports.cpp with Next decodes)
    static void write7FFD(uint8_t v);
    static void writeDFFD(uint8_t v);
    static void write1FFD(uint8_t v);

    // Layer 2 access port 0x123B (write window + visibility latch)
    static void writeLayer2Port(uint8_t v);
    static void updateLayer2Window();

    // Raster position for reg 0x1E/0x1F reads
    static uint16_t activeLine();
};

#endif
