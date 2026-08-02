/*

pico-speccy: ZX Spectrum Next (TBBlue) NextReg register file

*/

#include "NextReg.h"
#include "speccy/core/MemESP.h"
#include "speccy/z80/CPU.h"
#include "app/ESPectrum.h"
#include "speccy/video/Video.h"
#include "app/Debug.h"
#include "NextVideo.h"

#pragma GCC optimize("O3")

uint8_t NextReg::reg[256];
uint8_t NextReg::selected = 0;

uint8_t NextReg::port7FFD = 0;
uint8_t NextReg::portDFFD = 0;
uint8_t NextReg::port1FFD = 0;
uint8_t NextReg::port123B = 0;

bool     NextReg::lineIrqEnabled = false;
bool     NextReg::ulaIrqDisabled = false;
uint16_t NextReg::lineIrqLine = 0;

void NextReg::reset(bool hard) {
    if (hard) {
        for (int i = 0; i < 256; i++) reg[i] = 0;
        selected = 0;
    }
    port7FFD = 0;
    portDFFD = 0;
    port1FFD = 0;
    port123B = 0;
    MemESP::wr_overlay_active = false;
    lineIrqEnabled = false;
    ulaIrqDisabled = false;
    lineIrqLine = 0;
    // MMU defaults: ROM in slots 0/1, banks 5,2,0 in RAM slots
    reg[0x50] = 0xFF; reg[0x51] = 0xFF;
    reg[0x52] = 10;   reg[0x53] = 11;
    reg[0x54] = 4;    reg[0x55] = 5;
    reg[0x56] = 0;    reg[0x57] = 1;
    reg[0x07] = 0;    // 3.5 MHz
    reg[0x14] = 0xE3; // global transparency colour
    reg[0x4A] = 0x00; // fallback colour
    reg[0x4B] = 0xE3; // sprite transparency index
    reg[0x4C] = 0x0F; // tilemap transparency index
}

// Raster line for reg 0x1E/0x1F and the line interrupt. Line 0 is the first
// line of the pixel area, matching the Next convention. tstates run 2^m
// faster in turbo, so scale back to base-clock units first.
uint16_t NextReg::activeLine() {
    uint32_t ts = CPU::tstates >> ESPectrum::multiplicator;
    uint32_t line = ts / VIDEO::tStatesPerLine;
    // ULA paper starts 64 lines into the frame on 128K timing (63 border
    // lines + sync); the Next counts video lines from paper start.
    const uint32_t paperStart = 63;
    uint32_t total = CPU::statesInFrame >> ESPectrum::multiplicator;
    uint32_t lines = total / VIDEO::tStatesPerLine;
    return (uint16_t)((line + lines - paperStart) % lines);
}

static void applyRomSelect();
static void applyBankC000();

void NextReg::write(uint8_t r, uint8_t v) {
    reg[r] = v;
    switch (r) {
        case 0x02: // Reset: bit1 = hard, bit0 = soft — latched only for now
            break;
        case 0x05: { // Peripheral 1: bit 2 = 50/60 Hz
            ESPectrum::target = (v & 0x04) ? 17067 : MICROS_PER_FRAME_128;
            CPU::updateStatesInFrame();
            break;
        }
        case 0x07: { // CPU speed: 0=3.5, 1=7, 2=14, 3=28 MHz
            uint8_t m = v & 0x03;
            if (ESPectrum::multiplicator != m) {
                ESPectrum::multiplicator = m;
                CPU::updateStatesInFrame();
            }
            break;
        }
        case 0x22: // Line interrupt control
            ulaIrqDisabled = v & 0x04;
            lineIrqEnabled = v & 0x02;
            lineIrqLine = (lineIrqLine & 0xFF) | ((v & 0x01) << 8);
            break;
        case 0x23: // Line interrupt value LSB
            lineIrqLine = (lineIrqLine & 0x100) | v;
            break;
        case 0x12: case 0x13: // Layer 2 active/shadow bank
            updateLayer2Window();
            break;
        case 0x18: case 0x19: case 0x1A: case 0x1B: // clip windows
            NEXTVID::clipWrite(r - 0x18, v);
            break;
        case 0x1C: // clip window index reset
            NEXTVID::clipIndexReset(v);
            break;
        case 0x34: // sprite slot select mirror
            NEXTVID::spriteSlotSelect(v);
            break;
        case 0x35: case 0x36: case 0x37: case 0x38: case 0x39:
            NEXTVID::spriteAttrDirect(r - 0x35, v);
            break;
        case 0x40: NEXTVID::palIndex(v);   break;
        case 0x41: NEXTVID::palValue8(v);  break;
        case 0x43: NEXTVID::palControl(v); break;
        case 0x44: NEXTVID::palValue9(v);  break;
        case 0x50: case 0x51: case 0x52: case 0x53:
        case 0x54: case 0x55: case 0x56: case 0x57:
            MemESP::applyMMU(r - 0x50, v);
            break;
        case 0x60: NEXTVID::copperDataWrite(v);   break;
        case 0x61: NEXTVID::copperIndexLo(v);     break;
        case 0x62: NEXTVID::copperControl(v);     break;
        case 0x63: NEXTVID::copperData16Write(v); break;
        case 0x8E: { // Spectrum memory mapping (atomic 0x7FFD/0x1FFD write)
            // bit 3 = apply bank bits 7:4 to 0xC000; bit 2 = paging mode
            // (0 normal / 1 +3 all-RAM); bits 1:0 = ROM select or all-RAM map
            if (v & 0x08) {
                port7FFD = (port7FFD & ~0x07) | ((v >> 4) & 0x07);
                portDFFD = (portDFFD & ~0x0F) | ((v >> 7) & 0x01);
                applyBankC000();
            }
            if (v & 0x04) {
                write1FFD(0x01 | ((v & 0x03) << 1)); // all-RAM map
            } else {
                MemESP::romLatch = v & 0x01;
                write1FFD((v & 0x02) << 1);          // normal, ROM high bit
            }
            break;
        }
        default:
            // Remaining registers are latched; video/sprite/palette consumers
            // read NextReg::reg[] directly as they are implemented.
            break;
    }
}

uint8_t NextReg::read(uint8_t r) {
    switch (r) {
        case 0x00: return 10;    // machine ID: ZX Spectrum Next
        case 0x01: return 0x32;  // core major.minor = 3.2
        case 0x0E: return 0x00;  // core sub-minor
        case 0x07: // bits 5:4 = actual speed, 1:0 = programmed
            return ((reg[0x07] & 3) << 4) | (reg[0x07] & 3);
        case 0x1E: return (activeLine() >> 8) & 0x01;
        case 0x1F: return activeLine() & 0xFF;
        case 0x50: case 0x51: case 0x52: case 0x53:
        case 0x54: case 0x55: case 0x56: case 0x57:
            return MemESP::mmu[r - 0x50];
        default:   return reg[r];
    }
}

// ===== Layer 2 access port (0x123B) =====

void NextReg::writeLayer2Port(uint8_t v) {
    port123B = v;
    updateLayer2Window();
}

void NextReg::updateLayer2Window() {
    if (port123B & 0x01) { // write enable
        uint8_t bank = (port123B & 0x08) ? reg[0x13] : reg[0x12];
        uint8_t off = (port123B >> 6) & 3;
        uint16_t page = (uint16_t)(bank + off) * 2;
        if (page + 1 < MemESP::NEXT_PAGES && MemESP::nextRamReady) {
            MemESP::wrOverlay[0] = MemESP::nextRamPtr[page];
            MemESP::wrOverlay[1] = MemESP::nextRamPtr[page + 1];
            MemESP::wr_overlay_active = true;
            return;
        }
    }
    MemESP::wr_overlay_active = false;
}

// ===== Legacy Spectrum paging, Next-style (implemented on top of the MMU) =====

static void applyRomSelect() {
    // ROM select: 0x7FFD bit 4 = low bit, 0x1FFD bit 2 = high bit. With the
    // NextZXOS ROM set loaded from SD all four 16K ROMs are available;
    // otherwise clamp to the two-ROM 128K fallback.
    uint8_t romsel = (MemESP::romLatch & 0x01) | ((NextReg::port1FFD >> 1) & 0x02);
    MemESP::romInUse = MemESP::nextRomBase ? romsel : (romsel & 0x01);
    if (MemESP::mmu[0] == 0xFF) MemESP::applyMMU(0, 0xFF);
    if (MemESP::mmu[1] == 0xFF) MemESP::applyMMU(1, 0xFF);
}

static void applyBankC000() {
    uint8_t bank = (NextReg::port7FFD & 0x07) | ((NextReg::portDFFD & 0x0F) << 3);
    if (bank < MemESP::NEXT_PAGES / 2) {
        MemESP::applyMMU(6, bank * 2);
        MemESP::applyMMU(7, bank * 2 + 1);
    }
}

void NextReg::write7FFD(uint8_t v) {
    // nextreg 0x08 bit 6 releases the 0x7FFD paging lock
    if (MemESP::pagingLock && !(reg[0x08] & 0x40)) return;
    port7FFD = v;
    applyBankC000();
    uint8_t vl = (v >> 3) & 1;
    if (MemESP::videoLatch != vl) {
        MemESP::videoLatch = vl;
        VIDEO::grmem = vl ? MemESP::ram[7].direct() : MemESP::ram[5].direct();
    }
    MemESP::romLatch = (v >> 4) & 1;
    if (!(port1FFD & 0x01)) applyRomSelect();
    if (v & 0x20) MemESP::pagingLock = 1;
}

void NextReg::writeDFFD(uint8_t v) {
    portDFFD = v & 0x0F;
    applyBankC000();
}

void NextReg::write1FFD(uint8_t v) {
    port1FFD = v;
    if (v & 0x01) {
        // +3 special all-RAM configurations
        static const uint8_t maps[4][4] = {
            { 0, 1, 2, 3 }, { 4, 5, 6, 7 }, { 4, 5, 6, 3 }, { 4, 7, 6, 3 }
        };
        const uint8_t* m = maps[(v >> 1) & 3];
        for (int s = 0; s < 4; s++) {
            MemESP::applyMMU(s * 2,     m[s] * 2);
            MemESP::applyMMU(s * 2 + 1, m[s] * 2 + 1);
        }
    } else {
        // Leaving all-RAM mode: restore the normal 128K layout
        applyRomSelect();
        MemESP::applyMMU(2, 10); MemESP::applyMMU(3, 11);
        MemESP::applyMMU(4, 4);  MemESP::applyMMU(5, 5);
        applyBankC000();
    }
}


