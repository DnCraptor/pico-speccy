/*

pico-speccy: .NEX file loader (ZX Spectrum Next executable format)

Format reference: https://wiki.specnext.dev/NEX_file_format
Header V1.0-V1.3, 512 bytes; optional palette + loading screens; 16K
banks stored in order 5,2,0,1,3,4,6,7,8...111.

*/

#include "speccy/core/Snapshot.h"
#include "app/Config.h"
#include "app/ESPectrum.h"
#include "speccy/core/MemESP.h"
#include "speccy/z80/CPU.h"
#include "speccy/video/Video.h"
#include "NextReg.h"
#include "ui/OSDMain.h"
#include "fs/FileUtils.h"
#include "speccy/z80/z80.h"
#include "app/Debug.h"

using namespace std;

static bool readExact(FIL* f, void* dst, UINT len) {
    UINT br = 0;
    return f_read(f, dst, len, &br) == FR_OK && br == len;
}

static bool skipBytes(FIL* f, FSIZE_t len) {
    return f_lseek(f, f_tell(f) + len) == FR_OK;
}

// Read one 16K bank into Next RAM (two 8K pages)
static bool readBank(FIL* f, unsigned bank) {
    if (bank >= MemESP::NEXT_PAGES / 2) return false;
    if (!readExact(f, MemESP::nextRamPtr[bank * 2], 0x2000)) return false;
    return readExact(f, MemESP::nextRamPtr[bank * 2 + 1], 0x2000);
}

bool FileNEX::load(const string& nex_fn) {

    // .NEX needs the Next machine with its RAM carve-out in place. If we are
    // on another machine, save the file as the boot snapshot and reboot into
    // Next (setup() skips the pref_arch override when ram_file is set).
    if (!Z80Ops::isNext || !MemESP::nextRamReady) {
        if (butter_psram_size() < (2u << 20)) {
            OSD::osdCenteredMsg("Spectrum Next needs 2MB+ PSRAM", LEVEL_WARN, 2000);
            return false;
        }
        Config::arch = A_NEXT;
        Config::romSet = R_NEXT;
        Config::ram_file = nex_fn;
        Config::save();
        OSD::esp_hard_reset();
        return true; // not reached
    }

    FIL* file = fopen2(nex_fn.c_str(), FA_READ);
    if (!file) {
        OSD::osdCenteredMsg("Error opening file:\n" + nex_fn + "\n", LEVEL_INFO, 5000);
        return false;
    }

    uint8_t hdr[512];
    if (!readExact(file, hdr, sizeof(hdr)) ||
        hdr[0] != 'N' || hdr[1] != 'e' || hdr[2] != 'x' || hdr[3] != 't') {
        OSD::osdCenteredMsg("Bad NEX file:\n" + nex_fn + "\n", LEVEL_INFO, 5000);
        fclose2(file);
        return false;
    }

    uint8_t numBanks = hdr[9];
    uint8_t screens = hdr[10];
    uint8_t border = hdr[11] & 7;
    uint16_t sp = hdr[12] | (hdr[13] << 8);
    uint16_t pc = hdr[14] | (hdr[15] << 8);
    uint8_t preserveRegs = hdr[134];
    uint8_t entryBank = hdr[139];
    (void)numBanks;

    // Full machine reset unless the file asks to keep nextreg state
    if (!preserveRegs) {
        ESPectrum::reset();
    }

    // Optional palette block (for Layer2/LoRes screens, unless bit7 set)
    bool hasPalette = (screens & 0x05) && !(screens & 0x80);
    if (hasPalette) {
        // 256 entries x 2 bytes, same encoding as nextreg 0x44 two-write
        NextReg::write(0x43, 1 << 4); // select Layer 2 first palette for writes
        NextReg::write(0x40, 0);
        for (int i = 0; i < 256; i++) {
            uint8_t pair[2];
            if (!readExact(file, pair, 2)) break;
            NextReg::write(0x44, pair[0]);
            NextReg::write(0x44, pair[1]);
        }
        NextReg::write(0x43, 0);
    }

    // Loading screens
    if (screens & 0x01) { // Layer 2 (48K into banks 9,10,11)
        bool ok = readBank(file, 9) && readBank(file, 10) && readBank(file, 11);
        if (ok) {
            NextReg::write(0x12, 9);
            NextReg::writeLayer2Port(0x02); // visible (rendered from Phase 3 on)
        }
    }
    if (screens & 0x02) { // ULA (6912 into bank 5)
        readExact(file, MemESP::ram[5].direct(), 6912);
    }
    if (screens & 0x04) skipBytes(file, 12288); // LoRes — not rendered yet
    if (screens & 0x08) skipBytes(file, 12288); // Timex HiRes — not rendered yet
    if (screens & 0x10) skipBytes(file, 12288); // Timex HiCol — not rendered yet

    // 16K banks in stored order
    static const uint8_t first8[] = { 5, 2, 0, 1, 3, 4, 6, 7 };
    for (int i = 0; i < 8; i++) {
        if (hdr[18 + first8[i]]) {
            if (!readBank(file, first8[i])) break;
        }
    }
    for (unsigned b = 8; b < 112; b++) {
        if (hdr[18 + b]) {
            if (!readBank(file, b)) break;
        }
    }

    fclose2(file);

    // Entry state
    NextReg::write(0x56, entryBank * 2);
    NextReg::write(0x57, entryBank * 2 + 1);

    VIDEO::borderColor = border;
    VIDEO::brd = VIDEO::border32[border];

    Z80::reset();
    Z80::setRegSP(sp);
    Z80::setIM(Z80::IM1);
    Z80::setIFF1(false);
    Z80::setIFF2(false);
    if (pc != 0)
        Z80::setRegPC(pc);

    CPU::tstates = 0;
    CPU::global_tstates = 0;

    Debug::log("FileNEX: loaded '%s' PC=%04X SP=%04X entryBank=%d",
               nex_fn.c_str(), pc, sp, entryBank);
    return true;
}
