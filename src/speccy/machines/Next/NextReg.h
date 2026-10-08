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

    // Internal port decode enables, NR 0x82-0x85 packed as zxnext.vhd's
    // internal_port_enable(27:0) (2397-2442): a clear bit means the FPGA does
    // not decode that port at all. TBBLUE.FW clears them per BIOS personality
    // (48K: no 7FFD/DFFD/1FFD/FF, AY optional), NextZXOS writes 0xFF at boot
    // and clears 7FFD & co. itself for its own 48K/128K modes. No expansion
    // bus here, so NR 0x86-0x89 never take part.
    enum : uint32_t {
        PE_FF = 1u << 0, PE_7FFD = 1u << 1, PE_DFFD = 1u << 2, PE_1FFD = 1u << 3,
        PE_P3FLOAT = 1u << 4, PE_DMA6B = 1u << 5, PE_1F = 1u << 6, PE_37 = 1u << 7,
        PE_DIVMMC = 1u << 8, PE_MF = 1u << 9, PE_I2C = 1u << 10, PE_SPI = 1u << 11,
        PE_UART = 1u << 12, PE_MOUSE = 1u << 13, PE_SPRITE = 1u << 14, PE_LAYER2 = 1u << 15,
        PE_AY = 1u << 16, PE_DAC_SD1 = 1u << 17, PE_DAC_SD2 = 1u << 18, PE_DAC_PROFI = 1u << 19,
        PE_DAC_COVOX = 1u << 20, PE_DAC_FB = 1u << 21, PE_DAC_B3 = 1u << 22, PE_DAC_DF = 1u << 23,
        PE_ULAP = 1u << 24, PE_DMA0B = 1u << 25, PE_EFF7 = 1u << 26, PE_CTC = 1u << 27,
    };
    static uint32_t portEn;

    // Contended 16K banks 0-7 as a bitmask (zxnext.vhd mem_contend 4489): 48K
    // timing bank 5, 128K odd banks, +3 banks 4-7, banks 8+ never; 0 while
    // contention is off (Pentagon timing, CPU above 3.5 MHz, NR 0x08 bit 6).
    static uint8_t contendBanks;
    static inline bool pageContended(uint8_t page8k) {
        return page8k < 16 && ((contendBanks >> (page8k >> 1)) & 1);
    }
    // Re-derive contendBanks, NEXTVID's contention window and every slot's
    // MemESP::ramContended, and the fast memory path gate (it bypasses Draw, so
    // it must be off while anything is contended). Called on NR 0x03 / 0x07 /
    // 0x08, the raster change and reset.
    static void contendRecalc();
    static inline bool portOn(uint32_t m) { return (portEn & m) != 0; }
    static void portEnRecalc() {
        portEn = (uint32_t)reg[0x82] | ((uint32_t)reg[0x83] << 8) |
                 ((uint32_t)reg[0x84] << 16) | ((uint32_t)(reg[0x85] & 0x0F) << 24);
    }

    // The line interrupt (NR 0x22/0x23), the hardware-IM2 controller and the
    // CTC live in NextInt (NextInt.h).

    static void reset(bool hard);
    // nextreg 0x02 write latches a reset request; ESPectrum::loop performs
    // it at the next frame boundary (0 = none, 1 = soft, 2 = hard)
    static uint8_t resetPending;
    static void write(uint8_t r, uint8_t v);
    // Re-derive the live clock from nextreg 0x07 under Config::next_clk_cap
    // (the menu's CPU cap hook; also the body of a 0x07 write).
    static void applySpeed();
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
    static uint8_t machineType();     // nextreg 0x03 machine type (1 48K, 2 128K, 3 +3/Next, 4 Pentagon)
    static uint8_t machineTiming();   // nextreg 0x03 bits 6:4, same encoding
    // NextZXOS's own tape trap is live: divMMC automap enabled (NR 0x0A b4,
    // NR 0x83 b0), the #056A entry point armed (NR 0xBB b5) and ROM in slot 0
    // counting as ROM3. Our in-ROM fast load must stand aside then.
    static bool osTapeTrapArmed();
    // DivMMC NMI ("drive" button, F10 on a real Next; NR 0x02 bit 2 from
    // software). 0 = NMI raised, 1 = disabled (NR 0x06 bit 4 / NR 0x83 bit 0),
    // 2 = refused, the previous NMI is still being served.
    static uint8_t divNmiRequest(bool hotkey);

    // The video frame of the machine TIMING (nextreg 0x03 bits 6:4) and 50/60 Hz
    // (nextreg 0x05 bit 2; Pentagon is 50 Hz only) — wiki.specnext.dev
    // Reference_machines (VGA): 48K 224 T x 312/262 lines, 128K/+3 228 T x
    // 311/261, Pentagon 224 T x 320; the top border at 60 Hz is 26 lines shorter.
    // The one source for CPU::updateStatesInFrame, VIDEO's raster anchors,
    // the frame pacing and the audio set (ESPectrum::nextRasterApply).
    struct Raster {
        uint16_t tpl;        // T-states per line
        uint16_t lines;      // lines per frame
        uint8_t  kind;       // 0 = 48K, 1 = 128K/+3, 2 = Pentagon
        bool     hz60;
        uint8_t  intEnd;     // INT pulse length (base T)
        uint32_t micros;     // frame period, us
    };
    static Raster raster();
    static uint8_t rasterKey();       // changes whenever raster() would
    // Raster line (from the frame INT) of pixel line 0 for the current raster:
    // 48K 64, 128K/+3 63, Pentagon 80, 26 fewer at 60 Hz. Cached (the line-IRQ
    // test in Z80Ops::isActiveINT reads it per instruction), set by
    // applyRaster() with the rest of the raster.
    static uint16_t paperLineV;
    static inline uint32_t paperLine() { return paperLineV; }
    static void applyRasterPaper();   // recompute paperLineV from raster()

    // Timex/SCLD port 0xFF (Next decodes the full low byte). bit 6 disables
    // the ULA frame interrupt (zxnext.vhd port_ff_interrupt_disable) and is ONE
    // latch with NR 0x22 bit 2 and NOT NR 0xC4 bit 0 (zxnext.vhd 3612-3624,
    // last write wins); bits 5:0 mirror nextreg 0x69 bits 5:0. Reads are gated
    // on nextreg 0x08 bit 2.
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
void next_trace_rst8();
void next_trace_dump(uint32_t frames);
#endif

#endif
