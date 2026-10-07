// ZX Spectrum Next port decode — see NextPorts.h.
//
// Port enables: zxnext.vhd internal_port_enable(27:0) = NR 0x85(3:0) & 0x84 &
// 0x83 & 0x82 (2392-2442). TBBLUE.FW (src/firmware/app/src/boot.c,
// init_registers) clears them per BIOS personality — 48K: FF/7FFD/DFFD/1FFD/
// +3 float/DMA 6B off and the AY off unless "AY in 48K"; 128K: FF/DFFD/1FFD;
// +3: FF/DFFD/6B; Pentagon: FF/1FFD and Soundrive 2; sprites and Layer 2 off
// in all of them. NextZXOS writes 0xFF to all four at boot (enNextZX ROM0
// #00FB) and clears 7FFD & co. itself for its own 48K/128K modes (bank 2
// #0DE0).
#include "NextPorts.h"
#include "NextReg.h"
#include "NextVideo.h"
#include "NextRtc.h"
#include "app/ESPectrum.h"
#include "speccy/z80/CPU.h"
#include "speccy/video/Video.h"

// The ULA floating bus on port #FF (zxnext.vhd 2813 / 4513, zxula.vhd 573):
// only in 48K and 128K machine timing — +3 and Pentagon read 0xFF there —
// with the classic ULA's fetch window, i.e. Ports::getFloatBusData48 / 128
// (halt2int-verified on those machines) with the paper 26 lines earlier at
// 60 Hz. Turbo: the T-state count is not the ULA's, read 0xFF.
static uint8_t nxFloatBus() {
    if (ESPectrum::multiplicator) return 0xFF;
    const uint8_t mt = NextReg::machineTiming();   // 1 48K, 2 128K, 3 +3, 4 Pentagon
    if (mt != 1 && mt != 2) return 0xFF;
    const bool hz60 = NextReg::raster().hz60;
    unsigned int line;
    unsigned char halfpix;
    if (mt == 1) {
        const unsigned int ts = CPU::tstates;
        line = ts / 224 - (hz60 ? 38 : 64);
        if (line >= 192) return 0xFF;
        halfpix = (unsigned char)(ts % 224 - 3);
        if (halfpix >= 125 || (halfpix & 0x04)) return 0xFF;
    } else {
        const unsigned int ts = CPU::tstates - 1;
        line = ts / 228 - (hz60 ? 37 : 63);
        if (line >= 192) return 0xFF;
        halfpix = (unsigned char)(ts % 228);
        if (halfpix >= 128 || (halfpix & 0x04)) return 0xFF;
    }
    const int hp = (halfpix >> 2) + ((halfpix >> 1) & 0x01);
    return (halfpix & 0x01) ? VIDEO::grmem[VIDEO::offAtt[line] + hp]
                            : VIDEO::grmem[VIDEO::offBmp[line] + hp];
}

int NextPorts::input(uint16_t address) {
    if (address == 0x243B) return NextReg::selected;
    if (address == 0x253B) return NextReg::read(NextReg::selected);
    if (address == 0x123B && NextReg::portOn(NextReg::PE_LAYER2)) return NextReg::port123B;
    if (address == 0x303B && NextReg::portOn(NextReg::PE_SPRITE)) return NEXTVID::spriteFlagsRead();
    // I2C bus + DS1307 (NextRtc.cpp); NR 0x83 bit 2 undecodes both ports
    if (address == 0x103B && NextReg::portOn(NextReg::PE_I2C)) return NextRtc::readScl();
    if (address == 0x113B && NextReg::portOn(NextReg::PE_I2C)) return NextRtc::readSda();
    // Port 0xFF: the Timex register read-back, gated on nextreg 0x08 bit 2 and
    // the port's own enable; otherwise the ULA floating bus (even with the
    // port undecoded — zxnext.vhd port_ff_rd_dat ignores port_ff_io_en there).
    if ((address & 0xFF) == 0xFF) {
        if ((NextReg::reg[0x08] & 0x04) && NextReg::portOn(NextReg::PE_FF))
            return NextReg::portFF;
        return nxFloatBus();
    }
    return -1;
}

bool NextPorts::output(uint16_t address, uint8_t data) {
    const uint8_t a8 = address & 0xFF;
    if (address == 0x243B) { NextReg::selected = data; return true; }
    if (address == 0x253B) { NextReg::write(NextReg::selected, data); return true; }
    if (address == 0x123B && NextReg::portOn(NextReg::PE_LAYER2)) {
        NextReg::writeLayer2Port(data);
        return true;
    }
    if (address == 0x303B && NextReg::portOn(NextReg::PE_SPRITE)) {
        NEXTVID::spriteSlotSelect(data);
        return true;
    }
    // I2C bus + DS1307 (NextRtc.cpp)
    if (address == 0x103B && NextReg::portOn(NextReg::PE_I2C)) { NextRtc::writeScl(data); return true; }
    if (address == 0x113B && NextReg::portOn(NextReg::PE_I2C)) { NextRtc::writeSda(data); return true; }

    // Next DACs (Covox/SpecDrum/Soundrive ports): A/B = left, C/D = right,
    // routed through the stereo Covox channel. Each port is gated by its
    // NR 0x84 group (zxnext.vhd 2429-2435, 2658-2664): SD1 = 0F/1F/4F/5F,
    // SD2 = F1/F3/F9/FB, Profi = 3F/5F, Covox = 0F/4F, mono FB (only while SD2
    // is off), GS B3, SpecDrum DF. With SD2 on, F1/F9 belong to the DAC and
    // never reach 7FFD (port_fd_conflict_wr) — consumed here first.
    int8_t dacCh = -1;   // 0..3 = one channel, 4 = mono, 5 = B3 pair
    switch (a8) {
        case 0x0F: if (NextReg::portOn(NextReg::PE_DAC_SD1 | NextReg::PE_DAC_COVOX)) dacCh = 0; break;
        case 0x1F: if (NextReg::portOn(NextReg::PE_DAC_SD1)) dacCh = 1; break;
        case 0x4F: if (NextReg::portOn(NextReg::PE_DAC_SD1 | NextReg::PE_DAC_COVOX)) dacCh = 2; break;
        case 0x5F: if (NextReg::portOn(NextReg::PE_DAC_SD1 | NextReg::PE_DAC_PROFI)) dacCh = 3; break;
        case 0x3F: if (NextReg::portOn(NextReg::PE_DAC_PROFI)) dacCh = 1; break;
        case 0xF1: if (NextReg::portOn(NextReg::PE_DAC_SD2)) dacCh = 1; break;
        case 0xF3: if (NextReg::portOn(NextReg::PE_DAC_SD2)) dacCh = 0; break;
        case 0xF9: if (NextReg::portOn(NextReg::PE_DAC_SD2)) dacCh = 2; break;
        case 0xFB: if (NextReg::portOn(NextReg::PE_DAC_SD2)) dacCh = 3;
                   else if (NextReg::portOn(NextReg::PE_DAC_FB)) dacCh = 4; break;
        case 0xDF: if (NextReg::portOn(NextReg::PE_DAC_DF)) dacCh = 4; break;
        case 0xB3: if (NextReg::portOn(NextReg::PE_DAC_B3)) dacCh = 5; break;
    }
    if (dacCh >= 0) {
        static uint8_t nextDac[4];
        if (dacCh < 4)       nextDac[dacCh] = data;
        else if (dacCh == 5) nextDac[0] = nextDac[3] = data;          // stereo pair A/D
        else nextDac[0] = nextDac[1] = nextDac[2] = nextDac[3] = data; // mono
        ESPectrum::lastCovoxVal  = (uint8_t)((nextDac[0] + nextDac[1]) >> 1);
        ESPectrum::lastCovoxValR = (uint8_t)((nextDac[2] + nextDac[3]) >> 1);
        ESPectrum::CovoxGetSample();
        return true;
    }
    // Sprite upload ports decode the low byte only
    if (a8 == 0x57 && NextReg::portOn(NextReg::PE_SPRITE)) { NEXTVID::spriteAttrWrite(data); return true; }
    if (a8 == 0x5B && NextReg::portOn(NextReg::PE_SPRITE)) { NEXTVID::spritePatternWrite(data); return true; }
    if (address == 0xDFFD && NextReg::portOn(NextReg::PE_DFFD)) { NextReg::writeDFFD(data); return true; }
    if ((address & 0xF002) == 0x1000 && NextReg::portOn(NextReg::PE_1FFD)) {   // 0x1FFD (+3 paging)
        NextReg::write1FFD(data);
        return true;
    }
    // AY #FFFD/#BFFD undecoded (NR 0x84 bit 0 — the 48K personality without
    // "AY in 48K"): the write does nothing at all. No port between here and
    // the AY decode in Ports::output has A15=1, A1=0, so taking it here is
    // the same as taking it there.
    if ((address & 0x8002) == 0x8000 && !NextReg::portOn(NextReg::PE_AY)) return true;
    return false;
}
