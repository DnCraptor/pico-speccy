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

// The highest EFFECTIVE CPU-speed multiplier honoured by nextreg 0x07
// (0=3.5, 1=7, 2=14, 3=28 MHz) is Config::next_clk_cap — Machine > Spectrum
// Next > Options > CPU cap, default 14 MHz because 28 is not real-time on the
// RP2350 yet. The programmed value is kept and the read-back reports the
// actual speed in bits 5:4.

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
    // nextreg 0x02 write latches a reset request; ESPectrum::loop performs
    // it at the next frame boundary (0 = none, 1 = soft, 2 = hard)
    static uint8_t resetPending;
    static void write(uint8_t r, uint8_t v);
    // Re-derive the live clock from nextreg 0x07 under Config::next_clk_cap
    // (the menu's CPU cap hook; also the body of a 0x07 write).
    static void applySpeed();
    // A write to NR 0x22/0x23 moves (or arms) the line interrupt: end the
    // running unchecked exec_nocheck slice so CPU::loop re-derives the window.
    static void lineIrqReslice();
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
    static uint8_t machineType();
    // NextZXOS's own tape trap is live: divMMC automap enabled (NR 0x0A b4,
    // NR 0x83 b0), the #056A entry point armed (NR 0xBB b5) and ROM in slot 0
    // counting as ROM3. Our in-ROM fast load must stand aside then.
    static bool osTapeTrapArmed();   // nextreg 0x03 machine type (1 48K, 2 128K, 3 +3/Next, 4 Pentagon)
    // Raster line (from the frame INT) of pixel line 0: 63 at 50 Hz, 26 lower
    // at 60 Hz (nextreg 0x05 bit 2 — shorter top border, NEXTVID::computeGeometry).
    static inline uint32_t paperLine() { return (reg[0x05] & 0x04) ? 63 - 26 : 63; }

    // Timex/SCLD port 0xFF (Next decodes the full low byte). bit 6 disables
    // the ULA frame interrupt (zxnext.vhd port_ff_interrupt_disable — zx_go's
    // conformance matrix flagged it as boot-relevant); bits 5:0 mirror
    // nextreg 0x69 bits 5:0. Reads are gated on nextreg 0x08 bit 2.
    static uint8_t portFF;
    static bool    portFFIntDisable;
    static void writePortFF(uint8_t v);
};

#if NEXT_TRACE
// Diagnostics (CMake -DNEXT_TRACE=ON): 64-entry ring of nextreg / legacy-port
// writes + divMMC automap transitions, dumped once a second from
// ESPectrum::loop next to the state line. Same design rules as the NeoGS
// handshake ring (that session's lesson: an incomplete or oldest-first ring
// costs more than the bug): newest entries always survive the print budget,
// flood sources (palette/copper/sprite streams) are counted, not recorded.
void next_trace_ev(char kind, uint8_t r, uint8_t v);
void next_trace_dump(uint32_t frames);
#endif

#endif
