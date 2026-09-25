/*

pico-speccy: ZX Spectrum Next scanline video renderer

*/

#include "NextVideo.h"
#include "speccy/z80/z80.h"
#include "NextFramePacing.h"



#include <string.h>
#include <stdlib.h>
#include "speccy/video/Video.h"
#include "speccy/z80/CPU.h"
#include "app/ESPectrum.h"
#include "speccy/core/MemESP.h"
#include "NextReg.h"
#include "app/Debug.h"

// Hot render path in SRAM, like the rest of the video code
#undef IRAM_ATTR
#define IRAM_ATTR __not_in_flash("nextvid")

#pragma GCC optimize("O3")

uint16_t (*NEXTVID::rawPal)[256] = nullptr;
uint8_t  (*NEXTVID::lut)[256] = nullptr;
uint8_t  NEXTVID::remap[256];
bool     NEXTVID::palHasPrio[8];

uint8_t NEXTVID::clip[4][4];
uint8_t NEXTVID::clipIdx[4];

uint8_t NEXTVID::palIdx = 0;
bool    NEXTVID::pal9Second = false;
uint8_t NEXTVID::pal9First = 0;
uint8_t NEXTVID::palCtrl = 0;

uint8_t* NEXTVID::lineBuf = nullptr;
uint8_t* NEXTVID::pairMap = nullptr;
uint8_t* NEXTVID::hiresRow = nullptr;
uint8_t* NEXTVID::sprPatterns = nullptr;
uint8_t  (*NEXTVID::sprAttr)[5] = nullptr;
uint16_t NEXTVID::sprPatWrite = 0;
uint8_t  NEXTVID::sprAttrSlot = 0;
uint8_t  NEXTVID::sprAttrByte = 0;
uint8_t  NEXTVID::sprFlags = 0;

uint8_t* NEXTVID::copperMem = nullptr;
uint16_t NEXTVID::copIndex = 0;
uint16_t NEXTVID::copPC = 0;
uint8_t  NEXTVID::copCtrl = 0;
bool     NEXTVID::copWaiting = false;
uint16_t NEXTVID::copWaitLine = 0;
bool     NEXTVID::cop16Second = false;
uint8_t  NEXTVID::cop16First = 0;

uint32_t NEXTVID::lineIdx = 0;
uint32_t NEXTVID::totalLines = 311;
uint32_t& NEXTVID::nextLineTstate = VIDEO::ts_line_t;   // see NextVideo.h
int  NEXTVID::firstVisibleLine = 0;
int  NEXTVID::paperTopRow = 0;
int  NEXTVID::xPaperOff = 32;
bool NEXTVID::skipFrame = false;
static NextFramePacing framePacing;
// Palette index 16 is never produced (buildRemap skips it: 0..15 ZX colours,
// 17..183 the cube), so it marks "Timex hi-res pixel here" in lineBuf.
static constexpr uint8_t HIRES_MARK = 16;
static_assert(HIRES_MARK == 0x10, "RenderLine's word test spells it 0x10101010");
#if NEXT_TRACE
uint32_t g_nextvid_frames = 0; // completed frames with composition enabled
#endif

// ===== palette machinery =====

static inline uint8_t g3r3b2_of_rgb333(uint16_t v) {
    // rgb333 = RRRGGGBBB; CLUT cube layout is GGGRRRBB (grb_to_rgb888)
    uint8_t r = (v >> 6) & 7, g = (v >> 3) & 7, b = v & 7;
    return (g << 5) | (r << 2) | (b >> 1);
}

// Nearest usable CLUT index for a G3R3B2 target colour. Usable candidates:
//  - 17..183: the linear G3R3B2 cube. NOT beyond: on HDMI, 184-199 and
//    216-239 hold audio data-island / guard-band TMDS symbols (a pixel with
//    such a value emits CONTROL symbols mid-line — hw: yellow vertical
//    stripes where bright white (cube 219+) should be) and 200-215 are the
//    textmode slots; 240+ is sync. Stay below 184 everywhere.
//  - 0..15: the ZX palette entries — exact black/white/primaries. The cube's
//    dark corner (indices < 17) is cut off by the OSD range, so without
//    these, black lands on index 32 = visibly dark GREEN (hw). Their CLUT
//    values follow the user's ZX palette transform, which for the nominal
//    ZX points is close enough.
// Distances are computed in 8-bit RGB with rough luma weights.
void NEXTVID::buildRemap() {
    static const uint8_t zxlvl[2] = { 205, 255 };
    for (int i = 0; i < 256; i++) {
        int tr = ((i >> 2) & 7) * 255 / 7;
        int tg = ((i >> 5) & 7) * 255 / 7;
        int tb = (i & 3) * 255 / 3;
        int best = 17;
        // max d = 255^2 * 10 < 2^32 — plain uint32 arithmetic (a 64-bit
        // sentinel via `long` was UB on 32-bit ARM: bestd == 0, nothing ever
        // beat it and the whole remap collapsed to index 17 — hw: solid
        // crimson screen whatever the OS drew)
        uint32_t bestd = 0xFFFFFFFFu;
        for (int c = 0; c < 16; c++) {           // ZX colours
            int l = zxlvl[(c >> 3) & 1];
            int cr = (c & 2) ? l : 0, cg = (c & 4) ? l : 0, cb = (c & 1) ? l : 0;
            int dr = tr - cr, dg = tg - cg, db = tb - cb;
            uint32_t d = (uint32_t)(dr * dr * 3 + dg * dg * 6 + db * db);
            if (d < bestd) { bestd = d; best = c; }
        }
        for (int j = 17; j < 184; j++) {          // linear cube
            int cr = ((j >> 2) & 7) * 255 / 7;
            int cg = ((j >> 5) & 7) * 255 / 7;
            int cb = (j & 3) * 255 / 3;
            int dr = tr - cr, dg = tg - cg, db = tb - cb;
            uint32_t d = (uint32_t)(dr * dr * 3 + dg * dg * 6 + db * db);
            if (d < bestd) { bestd = d; best = j; }
        }
        remap[i] = (uint8_t)best;
    }
}

// Timex hi-res runs the driver in packed-pair mode: every framebuffer byte is
// a (left, right) pair of the 16 ZX colours. The ULA's 512 pixels are exact
// (VIDEO's timex_hr_lut); every other layer's palette index — 0..15 the ZX
// colours themselves, 17..183 the G3R3B2 cube (buildRemap) — becomes the solid
// pair of its nearest ZX colour, the same limit every pair mode has.
void NEXTVID::rebuildPairMap() {
    if (!pairMap) return;
    static const uint8_t zxlvl[2] = { 205, 255 };
    for (int i = 0; i < 256; i++) {
        int tr, tg, tb;
        if (i < 16) {
            const int l = zxlvl[(i >> 3) & 1];
            tr = (i & 2) ? l : 0; tg = (i & 4) ? l : 0; tb = (i & 1) ? l : 0;
        } else {
            tr = ((i >> 2) & 7) * 255 / 7; tg = ((i >> 5) & 7) * 255 / 7; tb = (i & 3) * 255 / 3;
        }
        int best = 0; uint32_t bestd = 0xFFFFFFFFu;
        for (int c = 0; c < 16; c++) {
            const int l = zxlvl[(c >> 3) & 1];
            const int dr = tr - ((c & 2) ? l : 0), dg = tg - ((c & 4) ? l : 0), db = tb - ((c & 1) ? l : 0);
            const uint32_t d = (uint32_t)(dr * dr * 3 + dg * dg * 6 + db * db);
            if (d < bestd) { bestd = d; best = c; }
        }
        pairMap[i] = VIDEO::profi_pair_lookup[best][best];
    }
}

void NEXTVID::setLutEntry(uint8_t pal, uint8_t idx, uint16_t rgb333) {
    rawPal[pal][idx] = rgb333;
    lut[pal][idx] = remap[g3r3b2_of_rgb333(rgb333)];
}

void NEXTVID::rebuildLut(uint8_t pal) {
    for (int i = 0; i < 256; i++)
        lut[pal][i] = remap[g3r3b2_of_rgb333(rawPal[pal][i])];
}

static inline uint16_t rgb332_to_333(uint8_t v) {
    uint8_t b2 = v & 3;
    return ((v & 0xE0) << 1) | ((v & 0x1C) << 1) | (b2 << 1) | (b2 ? 1 : 0);
}

void NEXTVID::defaultPalettes() {
    // Classic Spectrum colours in RGB333: level 5 normal, 7 bright
    for (int pal = 0; pal < 8; pal++) {
        palHasPrio[pal] = false;
        for (int i = 0; i < 256; i++)
            rawPal[pal][i] = rgb332_to_333((uint8_t)i);
    }
    for (int pal = 0; pal < 8; pal += 4) { // both ULA palettes
        for (int c = 0; c < 16; c++) {
            uint8_t base = c & 7;
            uint8_t l = (c & 8) ? 7 : 5;
            uint16_t v = ((base & 2) ? (l << 6) : 0)   // R
                       | ((base & 4) ? (l << 3) : 0)   // G
                       | ((base & 1) ? l : 0);         // B
            rawPal[pal][c] = v;        // ink
            rawPal[pal][16 + c] = v;   // paper (same colours)
        }
    }
    for (int pal = 0; pal < 8; pal++) rebuildLut(pal);
}

static inline uint8_t writePalSel_(uint8_t ctrl) { return (ctrl >> 4) & 7; }

uint8_t NEXTVID::palRead8() {
    // nextreg 0x41 read: bits 8:1 of the current entry (RRRGGGBB)
    if (!rawPal) return 0;
    return (uint8_t)(rawPal[writePalSel_(palCtrl)][palIdx] >> 1);
}

uint8_t NEXTVID::palRead9() {
    // nextreg 0x44 read: second-byte view — bit7 = L2 priority, bit0 = B lsb
    if (!rawPal) return 0;
    uint16_t raw = rawPal[writePalSel_(palCtrl)][palIdx];
    return (uint8_t)(((raw & 0x8000) ? 0x80 : 0) | (raw & 1));
}

void NEXTVID::palIndex(uint8_t v) {
    palIdx = v;
    pal9Second = false;
}

static inline uint8_t writePalSel(uint8_t ctrl) { return (ctrl >> 4) & 7; }

void NEXTVID::palValue8(uint8_t v) {
    setLutEntry(writePalSel(palCtrl), palIdx, rgb332_to_333(v));
    pal9Second = false;
    if (!(palCtrl & 0x80)) palIdx++;
}

void NEXTVID::palValue9(uint8_t v) {
    if (!pal9Second) {
        pal9First = v;
        pal9Second = true;
    } else {
        // second byte: bit0 = blue LSB, bit7 = Layer 2 priority
        uint16_t rgb = (rgb332_to_333(pal9First) & ~1) | (v & 1);
        uint8_t pal = writePalSel(palCtrl);
        if (v & 0x80) {
            rgb |= 0x8000;
            palHasPrio[pal] = true; // sticky until reset — cheap and safe
        }
        setLutEntry(pal, palIdx, rgb);
        pal9Second = false;
        if (!(palCtrl & 0x80)) palIdx++;
    }
}

void NEXTVID::palControl(uint8_t v) {
    palCtrl = v;
    pal9Second = false;
}

void NEXTVID::clipWrite(uint8_t win, uint8_t v) {
    win &= 3;
    clip[win][clipIdx[win] & 3] = v;
    clipIdx[win] = (clipIdx[win] + 1) & 3;
}

void NEXTVID::clipIndexReset(uint8_t mask) {
    for (int w = 0; w < 4; w++)
        if (mask & (1 << w)) clipIdx[w] = 0;
}

// ===== sprites =====

// Port 0x303B write: bits 5:0 select the pattern slot (bit 7 adds a 128-byte
// half-offset for 4bpp patterns) and bits 6:0 the attribute slot.
void NEXTVID::spriteSlotSelect(uint8_t v) {
    sprPatWrite = (uint16_t)(v & 0x3F) * 256 + ((v & 0x80) ? 128 : 0);
    sprAttrSlot = v & 0x7F;
    sprAttrByte = 0;
}

uint8_t NEXTVID::spriteFlagsRead() {
    uint8_t f = sprFlags;
    sprFlags = 0; // collision/overflow flags clear on read
    return f;
}

void NEXTVID::spriteAttrWrite(uint8_t v) {
    uint8_t* a = sprAttr[sprAttrSlot];
    a[sprAttrByte] = v;
    // Auto-advance: 4-byte sprites move on after byte 3 unless byte 3
    // requests the 5th attribute byte
    if (sprAttrByte == 3 && !(v & 0x40)) {
        a[4] = 0;
        sprAttrByte = 0;
        sprAttrSlot = (sprAttrSlot + 1) & 0x7F;
    } else if (sprAttrByte == 4) {
        sprAttrByte = 0;
        sprAttrSlot = (sprAttrSlot + 1) & 0x7F;
    } else {
        sprAttrByte++;
    }
}

void NEXTVID::spritePatternWrite(uint8_t v) {
    sprPatterns[sprPatWrite] = v;
    sprPatWrite = (sprPatWrite + 1) & 0x3FFF;
}

void NEXTVID::spriteAttrDirect(uint8_t byteIdx, uint8_t v) {
    // nextreg 0x35-0x39 mirror: write byte N of the selected sprite
    if (byteIdx < 5) sprAttr[sprAttrSlot][byteIdx] = v;
}

// ===== Copper =====

void NEXTVID::copperDataWrite(uint8_t v) {
    copperMem[copIndex] = v;
    copIndex = (copIndex + 1) & 0x7FF;
}

void NEXTVID::copperIndexLo(uint8_t v) {
    copIndex = (copIndex & 0x700) | v;
}

void NEXTVID::copperControl(uint8_t v) {
    copIndex = ((v & 0x07) << 8) | (copIndex & 0xFF);
    uint8_t prevMode = copCtrl >> 6, mode = v >> 6;
    copCtrl = v;
    if (mode != prevMode && mode != 0) {
        copPC = 0;
        copWaiting = false;
    }
}

void NEXTVID::copperData16Write(uint8_t v) {
    // reg 0x63: full instruction per two writes (MSB first), committed on
    // the second byte, index auto-advances by one instruction
    if (!cop16Second) {
        cop16First = v;
        cop16Second = true;
    } else {
        copperMem[copIndex & 0x7FE] = cop16First;
        copperMem[(copIndex & 0x7FE) + 1] = v;
        copIndex = (copIndex + 2) & 0x7FF;
        cop16Second = false;
    }
}

// Execute the copper program for one raster line: run MOVEs until an
// unsatisfied WAIT. Instructions are big-endian:
//   MOVE: 0RRRRRRR VVVVVVVV  — write V to nextreg R
//   WAIT: 1HHHHHHL LLLLLLLL  — wait for line (H = horizontal, line granularity)
IRAM_ATTR void NEXTVID::copperLine(uint16_t rasterLine) {
    uint8_t mode = copCtrl >> 6;
    if (mode == 0) return;
    if (copWaiting) {
        if (copWaitLine != rasterLine) return;
        copWaiting = false;
    }
    int guard = 1024;
    while (guard--) {
        uint8_t hi = copperMem[copPC * 2];
        uint8_t lo = copperMem[copPC * 2 + 1];
        copPC = (copPC + 1) & 0x3FF;
        if (hi & 0x80) {
            uint16_t wl = ((uint16_t)(hi & 0x01) << 8) | lo;
            if (wl != rasterLine) {
                copWaiting = true;
                copWaitLine = wl;
                return;
            }
        } else if (hi | lo) { // 0x0000 = NOOP
            NextReg::write(hi & 0x7F, lo);
        }
    }
}

// ===== geometry / frame control =====

void NEXTVID::Reset() {
    if (!sprPatterns) {
        // One block, Next-boot only, never freed (the Next boundary reboots).
        // Layout: patterns 16K | copper 2K | rawPal 4K | lut 2K | sprAttr 640 |
        // lineBuf 384 | pairMap 256 | hiresRow 256 (all word-aligned: 25216 is a
        // multiple of 4)
        uint8_t* blk = (uint8_t*)malloc(16384 + 2048 + 8*256*2 + 8*256 + 128*5 + 384 + 512);
        sprPatterns = blk;
        copperMem   = blk + 16384;
        rawPal      = (uint16_t(*)[256])(blk + 16384 + 2048);
        lut         = (uint8_t(*)[256])(blk + 16384 + 2048 + 8*256*2);
        sprAttr     = (uint8_t(*)[5])(blk + 16384 + 2048 + 8*256*2 + 8*256);
        lineBuf     = blk + 16384 + 2048 + 8*256*2 + 8*256 + 128*5;
        pairMap     = lineBuf + 384;
        hiresRow    = pairMap + 256;
        memset(blk, 0, 16384 + 2048 + 8*256*2 + 8*256 + 128*5 + 384 + 512);
    }
    buildRemap();
    defaultPalettes();
    palIdx = 0; pal9Second = false; palCtrl = 0;
    static const uint8_t defclip[4][4] = {
        { 0, 255, 0, 191 },  // Layer 2
        { 0, 255, 0, 191 },  // sprites
        { 0, 255, 0, 191 },  // ULA
        { 0, 159, 0, 255 },  // tilemap
    };
    for (int w = 0; w < 4; w++) {
        for (int i = 0; i < 4; i++) clip[w][i] = defclip[w][i];
        clipIdx[w] = 0;
    }

    memset(sprAttr, 0, 128 * 5);
    sprPatWrite = 0;
    sprAttrSlot = 0;
    sprAttrByte = 0;
    sprFlags = 0;

    copIndex = 0;
    copPC = 0;
    copCtrl = 0;
    copWaiting = false;
    cop16Second = false;

    totalLines = CPU::statesInFrame ?
        ((CPU::statesInFrame >> ESPectrum::multiplicator) / VIDEO::tStatesPerLine) : 311;
    firstVisibleLine = (VIDEO::tStatesBorder + VIDEO::tStatesPerLine / 2) / VIDEO::tStatesPerLine;
    paperTopRow = (VIDEO::tStatesScreen + VIDEO::tStatesPerLine / 2) / VIDEO::tStatesPerLine
                  - firstVisibleLine;
    xPaperOff = ((int)VIDEO::vga.xres - 256) / 2;
    lineIdx = 0;
    nextLineTstate = (uint32_t)VIDEO::tStatesPerLine << ESPectrum::multiplicator;
    skipFrame = false;
    framePacing.reset(time_us_32());
}

// nextreg 0x07 can switch the CPU clock MID-FRAME; the pending line
// boundary was computed in the old T-state scale.
void NEXTVID::onClockChange() {
    nextLineTstate = (lineIdx + 1) * ((uint32_t)VIDEO::tStatesPerLine << ESPectrum::multiplicator);
}

// Skip straight to the next line boundary instead of ticking 4T at a time
// (~17.7k Draw_Opcode calls a frame at 3.5 MHz, 4x that at 14) — TS-Conf's
// haltAdvanceTo. Exactly equivalent to the 4T loop it replaced: every step is
// a whole number of 4T HALT cycles, so tstates ends on the same value, R
// advances by the same count (mod 256 is enough, R is 7-bit), and
// ScanlineWork runs at the same crossings.
IRAM_ATTR void NEXTVID::haltAdvanceTo(uint32_t stEnd) {
    while (CPU::tstates < stEnd) {
        uint32_t tgt = nextLineTstate;
        if (tgt > stEnd) tgt = stEnd;
        const uint32_t n = (tgt > CPU::tstates) ? ((tgt - CPU::tstates + 3) & ~3u) : 4u;
        CPU::tstates += n;
        Z80::incRegR((uint8_t)(n >> 2));
        while (CPU::tstates >= nextLineTstate) ScanlineWork();
    }
}

IRAM_ATTR void NEXTVID::EndFrame() {
#if NEXT_TRACE
    if (!skipFrame) ++g_nextvid_frames;
#endif
    // A fixed 1-in-64 skip made the BIOS update at <1 Hz even when the CPU
    // managed 50 emulated frames/s. Bound skipping by wall time instead;
    // when emulation is slow, every frame is drawn.
    skipFrame = framePacing.skip(ESPectrum::maxSpeed, time_us_32());
    lineIdx = 0;
    nextLineTstate = (uint32_t)VIDEO::tStatesPerLine << ESPectrum::multiplicator;
    totalLines = (CPU::statesInFrame >> ESPectrum::multiplicator) / VIDEO::tStatesPerLine;
    if ((copCtrl >> 6) == 3) { // mode %11: restart the copper on vertical blank
        copPC = 0;
        copWaiting = false;
    }
}

IRAM_ATTR void NEXTVID::Tick(unsigned int statestoadd, bool contended) {
    CPU::tstates += statestoadd;
    while (CPU::tstates >= nextLineTstate) ScanlineWork();
}

IRAM_ATTR void NEXTVID::lineTick() {
    while (CPU::tstates >= nextLineTstate) ScanlineWork();
}

IRAM_ATTR void NEXTVID::Tick_Opcode(bool contended) {
    CPU::tstates += 4;
    while (CPU::tstates >= nextLineTstate) ScanlineWork();
}

void NEXTVID::DrawBorderNop() {}

#if NEXT_TRACE
// Compositor cost estimate for the 1 Hz NEXT: line: every 16th visible line
// is timed and extrapolated ×16 (~15 time_us_64 pairs per frame — noise-level
// overhead, but enough to see RenderLine's real share of cpu_frame_us).
volatile uint32_t g_nextvid_us = 0;
#endif

IRAM_ATTR void NEXTVID::ScanlineWork() {
    uint8_t m = ESPectrum::multiplicator;
    if (lineIdx < totalLines) {
        // Copper runs before the line is drawn; raster line 0 = paper start
        uint32_t paperStart = (uint32_t)(firstVisibleLine + paperTopRow);
        copperLine((uint16_t)((lineIdx + totalLines - paperStart) % totalLines));
        int row = (int)lineIdx - firstVisibleLine;
        if (!skipFrame && row >= 0 && row < (int)VIDEO::vga.yres) {
#if NEXT_TRACE
            if ((lineIdx & 15) == 0) {
                uint64_t t0 = time_us_64();
                RenderLine(row);
                g_nextvid_us += (uint32_t)(time_us_64() - t0) * 16;
            } else
#endif
            RenderLine(row);
        }
        lineIdx++;
        nextLineTstate = (lineIdx + 1) * ((uint32_t)VIDEO::tStatesPerLine << m);
    } else {
        // frame over-run (turbo tail) — wait for EndFrame to restart
        nextLineTstate = 0xFFFFFFFF;
    }
}

// ===== scanline compositor =====

// One framebuffer row. The layers are composed into lineBuf in linear pixel
// order, then copied into the framebuffer in ONE pass, already rotated into
// scanout order (the HDMI/VGA ISR reads fb bytes at x^2 — legacy renderers bake
// that into their AluBytes words; hw: NextZXOS text scrambled in 4-pixel groups
// without it). The framebuffer therefore only ever holds a finished row. It used
// to be composed in place and rotated afterwards, and whenever the scanout read
// the row between those two passes the display showed it unrotated for a frame:
// 4-pixel specks on glyph edges of a STATIC picture, one fb row at a time,
// drifting slowly with the frame/display phase (capture 2026-09-25).
// The copy also skips the OSD box: the F8 stats / F9-F10 volume box, 144x16 at
// x 168 (320-wide) / 188 (360-wide), rows 220 (240-row fb) / 268 (288-row fb),
// which ESPectrum::loop repaints only after EndFrame — tsRenderLine's and
// Update_Border_DS80's rectangle. The box is word-aligned (168/188 and 144 are
// multiples of 4), so it is a range of whole words.
IRAM_ATTR void NEXTVID::RenderLine(int row) {
    uint32_t* dst = (uint32_t*)VIDEO::vga.frameBuffer[row];
    RenderLayers(lineBuf, row);
    if (VIDEO::timex_hires_live) {
        // Packed-pair frame (Timex hi-res): the ULA left HIRES_MARK where its
        // 512-pixel line goes (hiresRow, already pair slots); everything else
        // is a palette index and becomes a solid pair.
        // Word at a time: almost every 4-byte group is either all HIRES_MARK
        // (take the hi-res word as is) or one colour (border, ULA-free paper);
        // only groups that mix layers go byte by byte. xPaperOff is a multiple
        // of 4 in both fb widths (32 / 52), so paper words line up with hiresRow.
        const int x0 = xPaperOff, words = ((int)VIDEO::vga.xres + 3) >> 2;
        uint32_t* lw = (uint32_t*)lineBuf;
        const uint32_t* hw = (const uint32_t*)hiresRow;
        for (int k = 0; k < words; k++) {
            const uint32_t w = lw[k];
            const int x = k << 2;
            if (w == 0x10101010u && (unsigned)(x - x0) < 256u) { lw[k] = hw[(x - x0) >> 2]; continue; }
            const uint8_t b0 = (uint8_t)w;
            if (w == b0 * 0x01010101u && b0 != HIRES_MARK) { lw[k] = pairMap[b0] * 0x01010101u; continue; }
            uint8_t* b = (uint8_t*)&lw[k];
            for (int i = 0; i < 4; i++) {
                const uint8_t v = b[i];
                b[i] = (v == HIRES_MARK && (unsigned)(x + i - x0) < 256u) ? hiresRow[x + i - x0] : pairMap[v];
            }
        }
    }
    const uint32_t* src = (const uint32_t*)lineBuf;
    const int words = ((int)VIDEO::vga.xres + 3) >> 2;
    int k0 = words, k1 = words;                    // [k0, k1) = words left alone
    const int cy0 = ((int)VIDEO::vga.yres >= 288) ? 268 : 220;
    if ((VIDEO::OSD & 0x07) != 0 && row >= cy0 && row < cy0 + 16) {
        k0 = ((VIDEO::vga.xres >= 360) ? 188 : 168) >> 2;
        k1 = k0 + 144 / 4;
    }
    for (int k = 0; k < k0; k++) { const uint32_t v = src[k]; dst[k] = (v >> 16) | (v << 16); }
    for (int k = k1; k < words; k++) { const uint32_t v = src[k]; dst[k] = (v >> 16) | (v << 16); }
}

IRAM_ATTR void NEXTVID::RenderLayers(uint8_t* fb, int row) {
    int xres = VIDEO::vga.xres;
    const uint8_t* ulaLut = lut[(palCtrl & 0x08) ? 4 : 0];

    bool ulaNext = palCtrl & 0x01;
    uint8_t borderIdx = ulaNext ? ulaLut[128 + (VIDEO::borderColor & 7)]
                                : ulaLut[16 + (VIDEO::borderColor & 7)];
    int y = row - paperTopRow;
    uint8_t r15 = NextReg::reg[0x15];

    if (y < 0 || y >= 192) {
        memset(fb, borderIdx, xres);
        // tilemap and Layer 2 (320x256 mode) cover the border area of the
        // 320x256 window; sprites reach it with the over-border bit
        if (NextReg::reg[0x6B] & 0x80)
            RenderTilemapLine(fb, row);
        if ((NextReg::port123B & 0x02) || (NextReg::reg[0x69] & 0x80))
            RenderL2Line(fb, row, false);
        if ((r15 & 0x03) == 0x03) // sprites enabled + over border
            RenderSpritesLine(fb, y);
        return;
    }

    // left/right border
    memset(fb, borderIdx, xPaperOff);
    memset(fb + xPaperOff + 256, borderIdx, xres - xPaperOff - 256);

    // ULA layer (classic attribute mode; ULANext later). ULA can be disabled
    // via nextreg 0x68 bit 7 — border colour shows through. LoRes (128x96,
    // reg 0x15 bit 7) replaces the ULA output.
    uint8_t* p = fb + xPaperOff;
    if (NextReg::reg[0x15] & 0x80) {
        uint8_t ly = (uint8_t)((y + NextReg::reg[0x33]) % 192) >> 1;
        const uint8_t* base = VIDEO::grmem +
            (ly < 48 ? ly * 128 : 0x2000 + (uint16_t)(ly - 48) * 128);
        uint8_t sxl = NextReg::reg[0x32];
        for (int x = 0; x < 256; x++)
            p[x] = ulaLut[base[(uint8_t)(x + sxl) >> 1]];
    }
    else if (!(NextReg::reg[0x68] & 0x80) && VIDEO::timex_hires_live) {
        // Timex hi-res 512x192: screen 0 (0x4000) gives the left 8 pixels of
        // each 16-pixel column, screen 1 (0x6000) the right 8 — the TC2048
        // layout VIDEO's renderer uses, and its nibble LUT (ink/paper from port
        // FF bits 5:3, all BRIGHT). hiresRow gets the pair slots in linear
        // order; the paper carries HIRES_MARK so layers drawn over the ULA win.
        const uint16_t* hl = VIDEO::timexHiresLut();
        const uint8_t* bmp = VIDEO::grmem + VIDEO::offBmp[y];
        uint32_t* hr = (uint32_t*)hiresRow;
        for (int col = 0; col < 32; col++) {
            const uint8_t e = bmp[col], o = bmp[col + 0x2000];
            hr[2 * col]     = (uint32_t)hl[e >> 4] | ((uint32_t)hl[e & 0x0F] << 16);
            hr[2 * col + 1] = (uint32_t)hl[o >> 4] | ((uint32_t)hl[o & 0x0F] << 16);
        }
        memset(p, HIRES_MARK, 256);
    }
    else if (!(NextReg::reg[0x68] & 0x80)) {
        const uint8_t* bmp = VIDEO::grmem + VIDEO::offBmp[y];
        const uint8_t* att = VIDEO::grmem + VIDEO::offAtt[y];
        if (ulaNext) {
            // ULANext attribute format: ink = attr & mask, paper = 128 + rest
            uint8_t mask = NextReg::reg[0x42];
            uint8_t shift = 0;
            for (uint8_t m = mask; m; m >>= 1) shift++;
            for (int col = 0; col < 32; col++) {
                uint8_t a = att[col];
                uint8_t b = bmp[col];
                uint8_t ink = ulaLut[a & mask];
                uint8_t pap = (mask == 255)
                    ? remap[g3r3b2_of_rgb333(rgb332_to_333(NextReg::reg[0x4A]))]
                    : ulaLut[128 + (a >> shift)];
                for (int bit = 0; bit < 8; bit++)
                    p[bit] = (b & (0x80 >> bit)) ? ink : pap;
                p += 8;
            }
        } else {
            for (int col = 0; col < 32; col++) {
                uint8_t a = att[col];
                uint8_t b = bmp[col] ^ (uint8_t)(-((a & VIDEO::flashing) >> 7));
                uint8_t bright = (a & 0x40) >> 3;
                uint8_t ink = ulaLut[(a & 7) | bright];
                uint8_t pap = ulaLut[16 + (((a >> 3) & 7) | bright)];
                p[0] = (b & 0x80) ? ink : pap;
                p[1] = (b & 0x40) ? ink : pap;
                p[2] = (b & 0x20) ? ink : pap;
                p[3] = (b & 0x10) ? ink : pap;
                p[4] = (b & 0x08) ? ink : pap;
                p[5] = (b & 0x04) ? ink : pap;
                p[6] = (b & 0x02) ? ink : pap;
                p[7] = (b & 0x01) ? ink : pap;
                p += 8;
            }
        }
    } else {
        memset(fb + xPaperOff, borderIdx, 256);
    }

    // Tilemap (over ULA/LoRes, under Layer 2 and sprites)
    if (NextReg::reg[0x6B] & 0x80)
        RenderTilemapLine(fb, row);

    // Layer 2, over ULA
    bool l2on = (NextReg::port123B & 0x02) || (NextReg::reg[0x69] & 0x80);
    uint8_t slu = (r15 >> 2) & 7;
    bool spritesUnderL2 = (slu == 1 || slu == 4); // L-S-U / L-U-S orders

    if (spritesUnderL2 && (r15 & 0x01))
        RenderSpritesLine(fb, y);

    if (l2on)
        RenderL2Line(fb, row, false);

    if (!spritesUnderL2 && (r15 & 0x01))
        RenderSpritesLine(fb, y);

    // Layer 2 palette-priority pixels sit above everything
    if (l2on && palHasPrio[(palCtrl & 0x04) ? 5 : 1])
        RenderL2Line(fb, row, true);
}

// Layer 2: 256x192x8bpp (row-major) or 320x256x8bpp (column-major, covers
// the border window). prioPass re-draws only palette-priority pixels after
// the sprites so they end up on top.
IRAM_ATTR void NEXTVID::RenderL2Line(uint8_t* fb, int row, bool prioPass) {
    uint8_t pal = (palCtrl & 0x04) ? 5 : 1;
    const uint8_t* l2Lut = lut[pal];
    const uint16_t* raw = rawPal[pal];
    uint8_t transp = NextReg::reg[0x14];
    uint8_t mode = (NextReg::reg[0x70] >> 4) & 3;
    uint8_t palOfs = (NextReg::reg[0x70] & 0x0F) << 4;
    uint16_t bank = (uint16_t)(NextReg::reg[0x12] & 0x7F) * 2;

    if (mode == 0) {
        // 256x192, aligned with the paper area
        int y = row - paperTopRow;
        if (y < 0 || y >= 192) return;
        if (y < clip[0][2] || y > clip[0][3]) return;
        uint16_t sy = y + NextReg::reg[0x17];
        if (sy >= 192) sy -= 192;
        uint16_t page = bank + (sy >> 5);
        if (page >= MemESP::NEXT_PAGES - 1u) return;
        const uint8_t* src = MemESP::nextRamPtr[page] + (sy & 31) * 256;
        uint8_t sx = NextReg::reg[0x16] + clip[0][0];
        uint8_t* q = fb + xPaperOff + clip[0][0];
        int count = (int)clip[0][1] - (int)clip[0][0] + 1;
        if (!prioPass) {
            while (count-- > 0) {
                uint8_t v = src[sx++]; // uint8 wrap = 256px scroll wrap
                if (v != transp) *q = l2Lut[(uint8_t)(v + palOfs)];
                q++;
            }
        } else {
            while (count-- > 0) {
                uint8_t v = src[sx++];
                if (v != transp && (raw[(uint8_t)(v + palOfs)] & 0x8000))
                    *q = l2Lut[(uint8_t)(v + palOfs)];
                q++;
            }
        }
    } else if (mode == 1) {
        // 320x256, column-major (byte offset = x*256 + y), covers the border
        int ty = row - (paperTopRow - 32);
        if (ty < 0 || ty >= 256) return;
        int cy1 = clip[0][2], cy2 = clip[0][3];
        if (ty < cy1 || ty > cy2) return;
        uint16_t scrollX = ((NextReg::reg[0x71] & 1) << 8) | NextReg::reg[0x16];
        uint8_t sy = (uint8_t)(ty + NextReg::reg[0x17]);
        int xOrig = xPaperOff - 32;
        int xres = VIDEO::vga.xres;
        int cx1 = clip[0][0] * 2, cx2 = clip[0][1] * 2 + 1;
        for (int tx = 0; tx < 320; tx++) {
            if (tx < cx1 || tx > cx2) continue;
            int fx = xOrig + tx;
            if (fx < 0 || fx >= xres) continue;
            uint16_t sx = tx + scrollX;
            if (sx >= 320) sx -= 320;
            uint32_t off = (uint32_t)sx * 256 + sy;
            uint16_t page = bank + (off >> 13);
            if (page >= MemESP::NEXT_PAGES) continue;
            uint8_t v = MemESP::nextRamPtr[page][off & 0x1FFF];
            if (v == transp) continue;
            if (prioPass && !(raw[(uint8_t)(v + palOfs)] & 0x8000)) continue;
            fb[fx] = l2Lut[(uint8_t)(v + palOfs)];
        }
    }
    // mode 2 (640x256x4bpp) is a documented non-goal (scanout is 360px max)
}

// Tilemap: 40x32 (or 80x32 rendered horizontally halved) tiles of 8x8x4bpp.
// Map and tile definitions live in bank 5 (offsets from nextreg 0x6E/0x6F).
// The tilemap window is 320x256 and covers the border area around the paper.
IRAM_ATTR void NEXTVID::RenderTilemapLine(uint8_t* fb, int row) {
    uint8_t ctrl = NextReg::reg[0x6B];
    int ty = row - (paperTopRow - 32);
    if (ty < 0 || ty >= 256) return;
    if (ty < clip[3][2] || ty > clip[3][3]) return;

    bool cols80 = ctrl & 0x40;
    bool noAttr = ctrl & 0x20;
    const uint8_t* lutTM = lut[(ctrl & 0x10) ? 7 : 3];
    uint8_t transp = NextReg::reg[0x4C] & 0x0F;
    const uint8_t* bank = MemESP::ram[5].direct();
    const uint8_t* map  = bank + (uint16_t)(NextReg::reg[0x6E] & 0x3F) * 256;
    const uint8_t* defs = bank + (uint16_t)(NextReg::reg[0x6F] & 0x3F) * 256;
    uint16_t scrollX = ((NextReg::reg[0x2F] & 3) << 8) | NextReg::reg[0x30];
    uint8_t sy = (uint8_t)(ty + NextReg::reg[0x31]); // wraps at 256
    int cols = cols80 ? 80 : 40;
    int entrySz = noAttr ? 1 : 2;
    const uint8_t* mrow = map + (sy >> 3) * cols * entrySz;
    uint8_t inRow = sy & 7;
    int xOrig = xPaperOff - 32;
    int xres = VIDEO::vga.xres;
    int cx1 = clip[3][0] * 2, cx2 = clip[3][1] * 2 + 1; // clip x in half-columns

    for (int tx = 0; tx < 320; tx++) {
        if (tx < cx1 || tx > cx2) continue;
        int fx = xOrig + tx;
        if (fx < 0 || fx >= xres) continue;
        // 80-column mode: 640 source pixels rendered halved
        uint16_t sx = cols80 ? (uint16_t)((tx * 2 + scrollX) % 640)
                             : (uint16_t)((tx + scrollX) % 320);
        const uint8_t* e = mrow + (sx >> 3) * entrySz;
        uint8_t tile = e[0];
        uint8_t attr = noAttr ? NextReg::reg[0x6C] : e[1];
        if (attr & 0x01) continue; // ULA over this tile
        uint8_t u = sx & 7, vv = inRow;
        if (attr & 0x08) u = 7 - u;                              // X mirror
        if (attr & 0x04) vv = 7 - vv;                            // Y mirror
        if (attr & 0x02) { uint8_t t = u; u = vv; vv = 7 - t; }  // rotate
        uint8_t b = defs[(uint16_t)tile * 32 + vv * 4 + (u >> 1)];
        uint8_t pix = (u & 1) ? (b & 0x0F) : (b >> 4);
        if (pix == transp) continue;
        fb[fx] = lutTM[pix | (attr & 0xF0)];
    }
}

// Render all visible sprites crossing paper line y (0-191). Sprite
// coordinate space puts (32,32) at the paper top-left corner; rendering is
// clipped to the paper area for now (sprites-over-border comes later).
IRAM_ATTR void NEXTVID::RenderSpritesLine(uint8_t* fb, int y) {
    const uint8_t* sprLut = lut[(palCtrl & 0x02) ? 6 : 2];
    uint8_t transp = NextReg::reg[0x4B];
    int sline = y + 32;
    if (sline < 0 || sline > 255) return;
    bool reverse = NextReg::reg[0x15] & 0x40; // bit6: sprite 0 drawn on top of 127
    bool overBorder = NextReg::reg[0x15] & 0x02;
    int xPixMin = overBorder ? -32 : 0;
    int xPixMax = overBorder ? 287 : 255;
    uint8_t coverage[40]; // 320-bit line coverage for the collision flag
    memset(coverage, 0, sizeof(coverage));
    int rendered = 0;

    // Default priority: lower sprite numbers on top — draw high numbers first
    for (int i = 0; i < 128; i++) {
        int s = reverse ? i : 127 - i;
        const uint8_t* a = sprAttr[s];
        if (!(a[3] & 0x80)) continue; // not visible
        uint8_t a4 = (a[3] & 0x40) ? a[4] : 0;
        // Relative/unified sprites (byte 4 type bits) not implemented yet —
        // rendered as independent anchors
        uint8_t yscale = (a4 >> 1) & 3;
        uint8_t xscale = (a4 >> 3) & 3;
        int sy = a[1] | ((a4 & 0x01) << 8);
        int height = 16 << yscale;
        int rowIn = sline - sy;
        if (rowIn < 0 || rowIn >= height) {
            if (sy + height > 511 && ((sline + 512 - sy) < height))
                rowIn = sline + 512 - sy; // Y wrap
            else
                continue;
        }
        int sx = a[0] | ((a[2] & 0x01) << 8);
        bool xmirror = a[2] & 0x08;
        bool ymirror = a[2] & 0x04;
        bool rotate  = a[2] & 0x02;
        uint8_t palOfs = a[2] & 0xF0;
        bool fourBit = a4 & 0x80;
        uint16_t patBase = (uint16_t)(a[3] & 0x3F) * 256;
        if (fourBit) patBase = (uint16_t)(a[3] & 0x3F) * 256 + ((a4 & 0x40) ? 128 : 0);

        uint8_t v = rowIn >> yscale;
        if (ymirror) v = 15 - v;
        int width = 16 << xscale;
        for (int px = 0; px < width; px++) {
            int xPix = sx + px - 32; // paper coords
            if (xPix < xPixMin || xPix > xPixMax) continue;
            if (!overBorder) {
                if (xPix < clip[1][0] || xPix > clip[1][1]) continue;
                if (y < clip[1][2] || y > clip[1][3]) continue;
            }
            int fx = xPaperOff + xPix;
            if (fx < 0 || fx >= (int)VIDEO::vga.xres) continue;
            uint8_t u = px >> xscale;
            if (xmirror) u = 15 - u;
            uint8_t uu = u, vv = v;
            if (rotate) { uu = v; vv = 15 - u; }
            uint8_t pix;
            if (fourBit) {
                uint8_t b = sprPatterns[(patBase + vv * 8 + (uu >> 1)) & 0x3FFF];
                pix = (uu & 1) ? (b & 0x0F) : (b >> 4);
                if (pix == (transp & 0x0F)) continue;
                pix |= palOfs;
            } else {
                pix = sprPatterns[(patBase + vv * 16 + uu) & 0x3FFF];
                if (pix == transp) continue;
                pix = (uint8_t)(pix + palOfs);
            }
            uint16_t cbit = xPix + 32;
            uint8_t mask = 1 << (cbit & 7);
            if (coverage[cbit >> 3] & mask) sprFlags |= 0x01; // collision
            coverage[cbit >> 3] |= mask;
            fb[fx] = sprLut[pix];
        }
        if (++rendered > 100) { sprFlags |= 0x02; break; } // per-line overflow
    }
}


