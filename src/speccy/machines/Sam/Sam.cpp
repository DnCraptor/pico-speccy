// pico-speccy — MGT SAM Coupe core. See Sam.h for the model and its sources.
//
// Everything here is FLASH code on purpose: SAM runs only on boards with butter
// PSRAM, and the shared RAM-resident hot paths (Ports::input/output, the accessors)
// keep one predicted-not-taken test that sends a SAM access here.

#include "Sam.h"

#include <string.h>

#include "app/Config.h"
#include "app/ESPectrum.h"
#include "app/Buffer.h"
#include "app/Debug.h"
#include "speccy/core/MemESP.h"
#include "speccy/core/Ports.h"
#include "speccy/z80/CPU.h"
#include "speccy/z80/MemGates.h"   // g_atm_ro: bit n = drop writes to CPU window n
#include "speccy/video/Video.h"
#include "speccy/devices/tape/Tape.h"
#include "speccy/devices/sound/SAASound.h"
#include "ui/LEDIndicators.h"
#include "speccy/roms/sam/sam_roms.h"

namespace Sam {

uint8_t lmpr, hmpr, vmpr, lepr, hepr, border, line = 0xFF, status = 0xFF, attr;
uint8_t clut[16];
volatile bool clutDirty = true;
uint8_t keys[9] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

static const uint8_t* s_rom0 = nullptr;
static const uint8_t* s_rom1 = nullptr;
static uint8_t*       s_ff   = nullptr;   // a 16K page of 0xFF for memory that is not fitted
static uint32_t       s_intPages = 32;
static uint32_t       s_extPages = 0;
static uint8_t        s_ro = 0;           // windows mapped read-only (ROM, absent memory, LMPR D7)
static uint8_t        s_hpen = 192, s_lpen = 0;  // last HPEN/LPEN (held while the screen is off)

static constexpr uint8_t LMPR_PAGE = 0x1F, LMPR_ROM0_OFF = 0x20, LMPR_ROM1 = 0x40, LMPR_WPROT = 0x80;
static constexpr uint8_t HMPR_PAGE = 0x1F, HMPR_MCNTRL = 0x80;
static constexpr uint8_t VMPR_PAGE = 0x1F, VMPR_MODE = 0x60, VMPR_MDE1 = 0x40;
static constexpr uint8_t BORDER_SOFF = 0x80;

void bindRoms() {
    s_rom0 = gb_rom_sam_30;
    s_rom1 = gb_rom_sam_30 + 0x4000;
}

static void mapRo(int w, const uint8_t* p) {
    MemESP::plugPtr16(w, (uint8_t*)p);
    mem_desc_t::bank_dirty[w * 2] = &mem_desc_t::dirty_sink;
    mem_desc_t::bank_dirty[w * 2 + 1] = &mem_desc_t::dirty_sink;
    s_ro |= (uint8_t)(1u << w);
}

static void mapAbsent(int w) {
    if (!s_ff) {
        s_ff = (uint8_t*)Buffer::palloc(0x4000, Buffer::NEED_POINTER | Buffer::PREFER_PSRAM);
        if (s_ff) memset(s_ff, 0xFF, 0x4000);
    }
    if (s_ff) mapRo(w, s_ff);
    else      mapRo(w, s_rom0);      // no memory for the 0xFF page: show ROM, still read-only
}

// Internal page (0..31): ram[page], or absent past the fitted 256K.
static void mapInt(int w, uint8_t page) {
    page &= 0x1F;
    if (page >= s_intPages) { mapAbsent(w); return; }
    MemESP::plugPtr16(w, MemESP::ram[page].sync(w));
}

// External page (LEPR/HEPR, 0..255): ram[32 + page], or absent past the fitted megabytes.
static void mapExt(int w, uint8_t page) {
    if (page >= s_extPages) { mapAbsent(w); return; }
    MemESP::plugPtr16(w, MemESP::ram[32 + page].sync(w));
}

void remap() {
    s_ro = 0;
    // A
    if (!(lmpr & LMPR_ROM0_OFF)) mapRo(0, s_rom0);
    else {
        mapInt(0, lmpr & LMPR_PAGE);
        if (lmpr & LMPR_WPROT) s_ro |= 1;
    }
    // B
    mapInt(1, (lmpr + 1) & LMPR_PAGE);
    // C
    if (hmpr & HMPR_MCNTRL) mapExt(2, lepr);
    else                    mapInt(2, hmpr & HMPR_PAGE);
    // D
    if (hmpr & HMPR_MCNTRL)    mapExt(3, hepr);
    else if (lmpr & LMPR_ROM1) mapRo(3, s_rom1);
    else                       mapInt(3, (hmpr + 1) & HMPR_PAGE);

    for (int s = 0; s < 8; s++) MemESP::ramContended[s] = false;   // contention: phase 1
    g_atm_ro = s_ro;
    MemESP::romInUse = 0;
    MemESP::p3special = 2;        // recoverPage0() keeps out of window 0
    MemESP::pagingLock = 0;
}

void reset() {
    const RomsetIdx r = Config::romSetSam;
    s_intPages = samIntPages(r);
    s_extPages = samExtPages(r);
    if (32 + s_extPages > MEM_PG_CNT) s_extPages = MEM_PG_CNT > 32 ? MEM_PG_CNT - 32 : 0;
    if (!s_rom0) bindRoms();
    lmpr = hmpr = vmpr = lepr = hepr = border = attr = 0;
    line = 0xFF;
    status = 0xFF;
    s_hpen = 192; s_lpen = 0;
    memset(clut, 0, sizeof(clut));
    clutDirty = true;
    remap();
    Debug::log("[SAM] reset: %uK internal + %u MB external", (unsigned)(s_intPages * 16),
               (unsigned)(s_extPages / 64));
}

// ---------------------------------------------------------------------------
// Interrupts

__attribute__((noinline)) bool intLine() {
    uint32_t t = CPU::tstates;
    if (t >= CPU::statesInFrame) t -= CPU::statesInFrame;   // overshoot = next frame
    if (t >= (uint32_t)CPU::IntStart && t < (uint32_t)CPU::IntEnd) return true;
    return lineIntUp(t);
}

// ---------------------------------------------------------------------------
// Ports

static uint8_t keyRows(uint8_t hi, uint8_t mask) {
    uint8_t k = mask;
    for (int r = 0; r < 8; r++)
        if (!(hi & (1u << r))) k &= keys[r];
    return k;
}

uint8_t portIn(uint16_t address) {
    VIDEO::Draw(4, false);
    const uint8_t lo = (uint8_t)address, hi = (uint8_t)(address >> 8);
    switch (lo) {
    case 0xFE: {
        uint8_t k = (hi == 0xFF) ? (uint8_t)(keys[8] & 0x1F) : keyRows(hi, 0x1F);
        return (uint8_t)(k | (border & BORDER_SOFF) | (Tape::tapeEarBit ? 0x40 : 0));
    }
    case 0xF9: {
        // Frame INT: bit 3 low inside the 128 T window (the same one isActiveINT raises).
        uint8_t st = status | 0x09;
        const int32_t t = (int32_t)CPU::tstates;
        if (t >= CPU::IntStart && t < CPU::IntEnd) st &= ~0x08;
        if (lineIntUp((uint32_t)t)) st &= ~0x01;
        return (uint8_t)(keyRows(hi, 0xE0) | (st & 0x1F));
    }
    case 0xFA: return lmpr;
    case 0xFB: return hmpr;
    case 0xFC: return (uint8_t)(0x80 | vmpr);
    case 0xFF: return attr;
    case 0xF8: {
        // HPEN (#1F8) / LPEN (#F8), SimCoupe update_hpen/update_lpen: the raster's
        // screen line and the byte column under it. The ROM's line-interrupt
        // handler polls HPEN until the beam leaves its line (D4EF) — answering a
        // constant hung it there for good. LPEN bit 0 is BCD1 of the colour under
        // the beam; we give the border's (the ASIC data latch is not modelled).
        const bool disabled = (border & BORDER_SOFF) && (vmpr & VMPR_MDE1);
        uint32_t t = CPU::tstates;
        if (t >= CPU::statesInFrame) t -= CPU::statesInFrame;
        const uint32_t ln = t / T_PER_LINE, lc = t % T_PER_LINE;
        const bool scr = ln >= TOP_LINES && ln < TOP_LINES + 192;
        if ((address & 0x1FF) == 0x1F8) {
            if (!disabled)
                s_hpen = (scr && (ln != TOP_LINES || lc >= 128)) ? (uint8_t)(ln - TOP_LINES) : 192;
            return s_hpen;
        }
        if (!disabled) {
            if (scr && lc >= 128) s_lpen = (uint8_t)(((lc - 128) & 0xFC) | (border & 1));
            else                  s_lpen = (uint8_t)(border & 1);
        } else s_lpen = (uint8_t)((s_lpen & ~1) | (border & 1));
        return s_lpen;
    }
    case 0x1F: return Config::joyLive() == JOY_KEMPSTON ? Ports::port[Config::kempstonPort] : 0;
    default:   return 0xFF;
    }
}

void portOut(uint16_t address, uint8_t data) {
    VIDEO::Draw(4, false);
    const uint8_t lo = (uint8_t)address, hi = (uint8_t)(address >> 8);
    switch (lo) {
    case 0xFE: {
        border = data;
        // Beeper: D4 (BEEP), D3 (MIC) — the speaker_values index the ULA path uses.
        const int a = Ports::speaker_values[((data >> 2) & 0x04) | (Tape::tapeEarBit << 1) |
                                            ((data >> 3) & 0x01)];
        if (a != ESPectrum::lastaudioBit) {
            ESPectrum::BeeperGetSample();
            ESPectrum::lastaudioBit = a;
            LED::touchW(LED::BEEPER);
        }
        break;
    }
    case 0xF9:
        // A new line moves the window: end the running unchecked slice so
        // CPU::samFrameLoop re-derives the next event (TsConf tsWakeLoop shape).
        if (line != data) {
            line = data;
            if (CPU::stFrame > CPU::tstates) CPU::stFrame = CPU::tstates ? CPU::tstates : 1;
        }
        break;
    case 0xFA: if (lmpr != data) { lmpr = data; remap(); } break;
    case 0xFB: if (hmpr != data) { hmpr = data; remap(); } break;
    case 0xFC: vmpr = data; break;
    case 0xF8: clut[hi & 0x0F] = data & 0x7F; clutDirty = true; break;
    case 0x80: lepr = data; if (hmpr & HMPR_MCNTRL) remap(); break;
    case 0x81: hepr = data; if (hmpr & HMPR_MCNTRL) remap(); break;
    case 0xFF:
        if (ESPectrum::SAA_emu && saaChip) {
            LED::touchW(LED::SAA);
            ESPectrum::SAAGetSample();
            if (address & 0x0100) saaChip->selectRegister(data);
            else                  saaChip->setRegisterData(data);
        }
        break;
    default: break;
    }
}

// ---------------------------------------------------------------------------
// Video

uint32_t colourRgb(uint8_t i) {
    const uint32_t r = ((i & 0x02) << 0) | ((i & 0x20) >> 3) | ((i & 0x08) >> 3);
    const uint32_t g = ((i & 0x04) >> 1) | ((i & 0x40) >> 4) | ((i & 0x08) >> 3);
    const uint32_t b = ((i & 0x01) << 1) | ((i & 0x10) >> 2) | ((i & 0x08) >> 3);
    return ((r * 255 + 3) / 7) << 16 | ((g * 255 + 3) / 7) << 8 | ((b * 255 + 3) / 7);
}

static inline const uint8_t* pagePtr(uint8_t page) {
    page &= 0x1F;
    if (page >= s_intPages) return nullptr;
    return MemESP::ram[page].direct();
}

// The framebuffer stores pixel x at byte x ^ 2 (the scanout ISR's order).
static inline void px(uint8_t* row, int x, uint8_t c) { row[x ^ 2] = c; }

void renderFrame() {
    auto& vga = VIDEO::vga;
    if (!vga.frameBuffer) return;
    const int xres = (int)vga.xres, yres = (int)vga.yres;
    const int left = ((xres - 256) / 2) & ~3;
    const int top  = (yres - 192) / 2;
    const uint8_t brdIdx = (uint8_t)(((border & 0x20) >> 2) | (border & 7));
    const int mode = (vmpr & VMPR_MODE) >> 5;           // 0 = mode 1 .. 3 = mode 4
    const bool off = (border & BORDER_SOFF) && (vmpr & VMPR_MDE1);
    // Modes 3/4 span 24K: the even page of the pair and the one after it.
    const uint8_t pg = (mode >= 2) ? (vmpr & (VMPR_PAGE & ~1)) : (vmpr & VMPR_PAGE);
    const uint8_t* p0 = pagePtr(pg);
    const uint8_t* p1 = pagePtr((uint8_t)(pg + 1));
    const bool flash = VIDEO::flashing != 0;
    uint8_t m3[4] = {0, 0, 0, 0};
    if (mode == 2) {
        const uint8_t b = (hmpr & 0x60) >> 3;
        m3[0] = b | 0; m3[1] = b | 2; m3[2] = b | 1; m3[3] = b | 3;   // entries 1/2 swapped
    }
    // Mode 3 at its real 512 px: the packed-pair driver, pair index = pixel value
    // 0..3, 4 = the border. Every other mode hands the driver back (no-op when off).
    uint32_t rgb5[5];
    for (int k = 0; k < 4; k++) rgb5[k] = colourRgb(clut[m3[k]]);
    rgb5[4] = colourRgb(clut[brdIdx]);
    const bool pair = VIDEO::samPairSync(mode == 2, rgb5);
    const uint8_t (*pl)[16] = VIDEO::profi_pair_lookup;
    const uint8_t brd = pair ? pl[4][4] : brdIdx;

    auto renderRow = [&](uint8_t* row, int y) {
        const int sl = y - top;
        if (sl < 0 || sl >= 192 || off || !p0) { memset(row, brd, xres); return; }
        // side borders (left is a multiple of 4, so the swizzle stays inside them)
        memset(row, brd, left);
        memset(row + left + 256, brd, xres - left - 256);
        uint8_t* o = row + left;
        switch (mode) {
        case 0:
        case 1: {
            const uint8_t* d;
            const uint8_t* a;
            if (mode == 0) {
                d = p0 + (((sl & 0xC0) << 5) | ((sl & 0x07) << 8) | ((sl & 0x38) << 2));
                a = p0 + 6144 + ((sl & 0xF8) << 2);
            } else {
                d = p0 + (sl << 5);
                a = d + 0x2000;
            }
            for (int c = 0; c < 32; c++) {
                const uint8_t at = a[c], bits = d[c];
                uint8_t ink = (uint8_t)(((at >> 3) & 8) | (at & 7));
                uint8_t pap = (uint8_t)((at >> 3) & 0xF);
                if (flash && (at & 0x80)) { const uint8_t t = ink; ink = pap; pap = t; }
                for (int b = 0; b < 8; b++)
                    px(o, c * 8 + b, (bits & (0x80 >> b)) ? ink : pap);
            }
            break;
        }
        case 2:
        case 3: {
            const uint32_t off128 = (uint32_t)sl << 7;
            const uint8_t* d = (off128 < 0x4000) ? p0 + off128 : (p1 ? p1 + (off128 - 0x4000) : nullptr);
            if (!d) { memset(o, brd, 256); break; }
            if (pair) {
                // 512 px: two pixels per fb byte.
                for (int i = 0; i < 128; i++) {
                    const uint8_t v = d[i];
                    px(o, i * 2,     pl[v >> 6][(v >> 4) & 3]);
                    px(o, i * 2 + 1, pl[(v >> 2) & 3][v & 3]);
                }
            } else if (mode == 3) {
                for (int i = 0; i < 128; i++) {
                    px(o, i * 2,     d[i] >> 4);
                    px(o, i * 2 + 1, d[i] & 0x0F);
                }
            } else {
                // 512 px at 2 bpp, shown at half width: the left pixel of each pair.
                for (int i = 0; i < 128; i++) {
                    px(o, i * 2,     m3[d[i] >> 6]);
                    px(o, i * 2 + 1, m3[(d[i] >> 2) & 3]);
                }
            }
            break;
        }
        }
    };

    for (int y = 0; y < yres; y++) {
        uint8_t* row = vga.frameBuffer[y];
        if (!row) continue;
        // Overlays drawn after EndFrame (F8 stats / volume box, LED panel, notify
        // banner) own their rects: keep those bytes, or every frame erases them.
        int cx[8][2];
        const int nc = VIDEO::rowCarves(y, cx);
        if (!nc) { renderRow(row, y); continue; }
        uint8_t keep[400];
        int kn = 0;
        for (int i = 0; i < nc; i++) {
            const int x0 = cx[i][0] < 0 ? 0 : cx[i][0];
            const int x1 = cx[i][1] > xres ? xres : cx[i][1];
            if (x1 > x0 && kn + (x1 - x0) <= (int)sizeof(keep)) { memcpy(keep + kn, row + x0, x1 - x0); kn += x1 - x0; }
            else cx[i][1] = cx[i][0];
        }
        renderRow(row, y);
        kn = 0;
        for (int i = 0; i < nc; i++) {
            const int x0 = cx[i][0] < 0 ? 0 : cx[i][0];
            const int x1 = cx[i][1] > xres ? xres : cx[i][1];
            if (x1 > x0) { memcpy(row + x0, keep + kn, x1 - x0); kn += x1 - x0; }
        }
    }
}

} // namespace Sam
