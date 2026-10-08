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
#include "hardware/sync.h"
#include "pico/time.h"
#include "app/TryAlloc.h"
#include "app/Buffer.h"
#include "app/Config.h"

// Hot render path in SRAM, like the rest of the video code
#undef IRAM_ATTR
#define IRAM_ATTR __not_in_flash("nextvid")

#pragma GCC optimize("O3")

uint16_t (*NEXTVID::rawPal)[256] = nullptr;
uint8_t  (*NEXTVID::lut)[256] = nullptr;
uint8_t* NEXTVID::remap = nullptr;
bool     NEXTVID::palHasPrio[8];

uint8_t NEXTVID::clip[4][4];
uint8_t NEXTVID::clipIdx[4];

uint8_t NEXTVID::palIdx = 0;
bool    NEXTVID::pal9Second = false;
uint8_t NEXTVID::pal9First = 0;
uint8_t NEXTVID::palCtrl = 0;

uint8_t* NEXTVID::lineBuf = nullptr;
uint8_t* NEXTVID::gigsPrev = nullptr;
uint8_t  NEXTVID::gigsBlend[256];
// 0 = off, 1 = blend with the previous frame, 2 = only record this frame (the
// first frame after the buffer came up, or Auto between flips)
static uint8_t s_gigs = 0;
static bool    s_gigsSeed = true;
uint8_t* NEXTVID::pairMap = nullptr;
uint8_t* NEXTVID::hiresRow = nullptr;
uint8_t* NEXTVID::sprPatterns = nullptr;
uint8_t  (*NEXTVID::sprAttr)[5] = nullptr;
uint16_t NEXTVID::sprPatWrite = 0;
uint8_t  NEXTVID::sprAttrSlot = 0;
uint8_t  NEXTVID::sprAttrByte = 0;
uint8_t  NEXTVID::sprFlags = 0;
uint8_t  NEXTVID::sprNrSlot = 0;
NEXTVID::SprRes* NEXTVID::sprRes = nullptr;
uint8_t* NEXTVID::sprVis = nullptr;
uint8_t  NEXTVID::sprVisN = 0;
bool     NEXTVID::sprDirty = true;

uint8_t* NEXTVID::copperMem = nullptr;
uint16_t NEXTVID::copIndex = 0;
uint16_t NEXTVID::copPC = 0;
uint8_t  NEXTVID::copCtrl = 0;
bool     NEXTVID::copWaiting = false;
uint16_t NEXTVID::copWaitLine = 0;
bool     NEXTVID::cop16Second = false;
uint8_t  NEXTVID::cop16First = 0;

uint32_t NEXTVID::lineIdx = 0;
uint8_t NEXTVID::contend = 0;
int16_t NEXTVID::contLine = 0;
int16_t NEXTVID::contCol = 0;
uint8_t NEXTVID::contWait[8] = {6, 5, 4, 3, 2, 1, 0, 0};
uint32_t NEXTVID::totalLines = 311;
uint32_t& NEXTVID::nextLineTstate = VIDEO::ts_line_t;   // see NextVideo.h
int  NEXTVID::firstVisibleLine = 0;
int  NEXTVID::paperTopRow = 0;
int  NEXTVID::xPaperOff = 32;
bool NEXTVID::skipFrame = false;
static NextFramePacing framePacing;

// Everything the scanline renderer reads that the guest (or the copper) can
// change between two lines, captured by core0 when the line is posted. What is
// NOT here is read live by whichever core renders, and its writers drain the
// core1 queue first: the palettes (lut/rawPal/palHasPrio), sprite attributes
// and patterns, the Timex hi-res LUT and pair map, the framebuffer rows.
// Guest RAM (ULA screen, Layer 2, tilemap) is read live WITHOUT a drain — core1
// lags core0 by a few lines, the same race the beam has against a program
// drawing into the visible page (the TS-Conf rule).
struct NextLineState {
    const uint8_t* grmem;
    uint8_t clip[4][4];
    uint8_t r12, r14, r15, r16, r17, r2F, r30, r31, r32, r33, r42, r4A, r4B, r4C, r68, r69, r6B, r6C, r6E, r6F, r70, r71;
    uint8_t p123B, palCtrl, border, flashing, hires, osd;
    uint8_t gigs, _pad[3];   // Gigascreen this frame (s_gigs), kept word-aligned
    // Border colour changes INSIDE the row (OUT #FE mid-line): segment i ends
    // at fb x 2*brd.t[i] (T-states into the row, cell-aligned) in colour nibble i of brd.c;
    // the rest of the row is `border`. n = 0 for an ordinary row (every byte
    // zero, so it still dedups). 15 runs: a colour-bar test changes #FE eight
    // times a row, and OUT (C),r every 12 T fits ~15 into the 180-T window.
    // 24 bytes: 64 queued states = 1.5 KB of heap.
    struct { uint8_t t[15]; uint8_t n; uint8_t c[8]; } brd;
};
static const NextLineState* LS = nullptr;
static_assert(sizeof(NextLineState) % 4 == 0, "compared a word at a time");
// memcmp is libc in FLASH; 12 words compared inline instead, once per line.
static inline __attribute__((always_inline)) bool nxSameState(const NextLineState& a, const NextLineState& b) {
    const uint32_t* x = (const uint32_t*)&a; const uint32_t* y = (const uint32_t*)&b;
    uint32_t d = 0;
    for (unsigned i = 0; i < sizeof(NextLineState) / 4; i++) d |= x[i] ^ y[i];
    return d == 0;
}   // the line being rendered (one renderer at a time)

IRAM_ATTR void NEXTVID::captureLine(NextLineState& st) {
    const uint8_t* r = NextReg::reg;
    st.grmem = VIDEO::grmem;
    memcpy(st.clip, clip, sizeof st.clip);
    st.r12 = r[0x12];
    st.r14 = r[0x14];
    st.r15 = r[0x15];
    st.r16 = r[0x16];
    st.r17 = r[0x17];
    st.r2F = r[0x2F];
    st.r30 = r[0x30];
    st.r31 = r[0x31];
    st.r32 = r[0x32];
    st.r33 = r[0x33];
    st.r42 = r[0x42];
    st.r4A = r[0x4A];
    st.r4B = r[0x4B];
    st.r4C = r[0x4C];
    st.r68 = r[0x68];
    st.r69 = r[0x69];
    st.r6B = r[0x6B];
    st.r6C = r[0x6C];
    st.r6E = r[0x6E];
    st.r6F = r[0x6F];
    st.r70 = r[0x70];
    st.r71 = r[0x71];
    st.p123B = NextReg::port123B;
    st.palCtrl = palCtrl;
    st.border = VIDEO::borderColor;
    st.flashing = VIDEO::flashing;
    st.hires = VIDEO::timex_hires_live ? 1 : 0;
    st.osd = VIDEO::OSD;
    st.gigs = s_gigs;
    st._pad[0] = st._pad[1] = st._pad[2] = 0;
    uint32_t* w = (uint32_t*)&st.brd;   // no libc memset on this path (flash)
    for (unsigned i = 0; i < sizeof st.brd / 4; i++) w[i] = 0;
}

// ---- per-T border (OUT #FE mid-line) ----
// The ULA paints the border with the colour of the moment, so a program that
// rewrites #FE inside a line splits that line into coloured runs (the 48K
// border tests, rainbow demos). VIDEO::DrawBorder() is called on every colour
// change with the OLD colour still in VIDEO::borderColor: borderMark() keeps
// {T of the change, old colour} in a small ring, and emitRow turns the changes
// inside the row's T window into segments. The window comes from the classic
// beam renderer's border anchor (VIDEO::tStatesBorder, hw-verified incl. its
// pipeline correction): fb x of raster line L shows T = L*tpl + s_bcol + x/2.
#define NX_BRD_N 32
static uint32_t s_brdT[NX_BRD_N];
static uint8_t  s_brdC[NX_BRD_N];
static uint8_t  s_brdHead = 0, s_brdCnt = 0;
static int16_t  s_bcol = 0;
// The border is latched in cells of s_bstep T-states — 4 (8 px) on the 48K /
// 128K / +3 ULA, 1 on Pentagon — the classic renderer's brdcol_step. A cell
// that STARTS at or before the change still shows the old colour
// (VIDEO::TopBorder paints every cell with lastBrdTstate <= tstates first).
static uint8_t  s_bstep = 4;

void NEXTVID::borderMark() {
    if (s_brdCnt == NX_BRD_N) { s_brdHead = (s_brdHead + 1) % NX_BRD_N; s_brdCnt--; }
    const uint8_t i = (s_brdHead + s_brdCnt) % NX_BRD_N;
    s_brdT[i] = CPU::tstates;
    s_brdC[i] = VIDEO::borderColor & 7;
    s_brdCnt++;
}

// Flash: runs only while the ring holds changes (a few per row at most).
__attribute__((noinline)) static void nxBorderSegs(NextLineState& st, int row, int fvLine) {
    const uint8_t m = ESPectrum::multiplicator;
    const int32_t t0 = ((row + fvLine) * (int32_t)VIDEO::tStatesPerLine + s_bcol) * (1 << m);
    const int32_t t1 = t0 + ((int32_t)VIDEO::vga.xres / 2) * (1 << m);
    while (s_brdCnt && (int32_t)s_brdT[s_brdHead] < t0) {   // ended before this row
        s_brdHead = (s_brdHead + 1) % NX_BRD_N; s_brdCnt--;
    }
    uint8_t n = 0, k = 0;
    for (; k < s_brdCnt; k++) {
        const uint8_t i = (s_brdHead + k) % NX_BRD_N;
        const int32_t t = (int32_t)s_brdT[i];
        if (t >= t1) break;
        if (n < 15) {
            const int32_t rel = (t - t0) >> m;   // >= 0: earlier changes were dropped
            st.brd.t[n] = (uint8_t)((rel / s_bstep + 1) * s_bstep);
            st.brd.c[n >> 1] |= (uint8_t)(s_brdC[i] << ((n & 1) * 4));
            n++;
        }
    }
    st.brd.n = n;
    // colour after the last in-row change: the one in force until the next
    // change (already recorded if it happened before this row was posted)
    st.border = (k < s_brdCnt) ? s_brdC[(s_brdHead + k) % NX_BRD_N] : (VIDEO::borderColor & 7);
}

// ---- the core1 line queue (NEXT_C1_*) ----
// Jobs are {row, state slot}; a state slot is written only when the captured
// inputs differ from the last posted line's, and reused only once core1 has
// consumed the last job naming it. Free-running indices, one writer each.
#define NEXT_C1_RING   320
#define NEXT_C1_STATES 64
static uint32_t*      nx_ring = nullptr;     // [NEXT_C1_RING]  row | slot << 16
static NextLineState* nx_st = nullptr;       // [NEXT_C1_STATES]
static uint32_t*      nx_st_job = nullptr;   // [NEXT_C1_STATES] nx_w after the last job naming the slot
static uint32_t       nx_cur = 0;
static bool           nx_have = false;
static volatile uint32_t nx_w = 0, nx_r = 0;
static NextLineState  nx_sync;               // the queue-off path
// Tested by core1's render_core loop on every machine: one byte in .data.
extern "C" volatile bool g_next_c1_live = false;

static void nxQueueDead(const char* why) {
    Debug::log("[NXC1] core1 render queue %s (r=%u w=%u) - rendering on core0", why,
               (unsigned)nx_r, (unsigned)nx_w);
    g_next_c1_live = false;
    nx_r = nx_w;
}

// core0: wait for core1 to finish every posted line. Bounded, like the TS-Conf
// drain: a wedged core1 must not take core0 with it.
IRAM_ATTR void NEXTVID::c1Drain() {
    if (nx_r == nx_w) return;
    const uint64_t t0 = time_us_64();
    while (nx_r != nx_w) {
        if (time_us_64() - t0 > 100000) { nxQueueDead("stuck"); return; }
        tight_loop_contents();
    }
}

static void __not_in_flash_func(nxWaitUntil)(uint32_t w1) {
    if ((int32_t)(nx_r - w1) >= 0) return;
    const uint64_t t0 = time_us_64();
    while ((int32_t)(nx_r - w1) < 0) {
        if (time_us_64() - t0 > 100000) { nxQueueDead("stuck"); return; }
        tight_loop_contents();
    }
}
// Palette index 16 is never produced (buildRemap skips it: 0..15 ZX colours,
// 17..183 the cube), so it marks "Timex hi-res pixel here" in lineBuf.
static constexpr uint8_t HIRES_MARK = 16;
static_assert(HIRES_MARK == 0x10, "RenderLine's word test spells it 0x10101010");
#if NEXT_TRACE
uint32_t g_nextvid_frames = 0; // completed frames with composition enabled
#endif

// ===== palette machinery =====


// Nearest usable CLUT index for an RGB333 colour (remap[512]). Usable candidates:
//  - isSlot(): the colour lattice (NextVideo.h slotRgb), programmed by
//    VIDEO::Reset / applyPalette. NEVER 184-199 / 216-239 on HDMI: those hold
//    audio data-island / guard-band TMDS symbols (a pixel with such a value
//    emits CONTROL symbols mid-line — hw: yellow vertical stripes); 240+ is
//    sync; 152-167 is the UI block the menu repaints.
//  - 0..15: the ZX palette entries — exact black/white/primaries. Black is
//    deliberately not in the lattice: it is ZX colour 0.
// Distances are computed in 8-bit RGB with rough luma weights.
void NEXTVID::buildRemap() {
    c1Drain();
    static const uint8_t zxlvl[2] = { 205, 255 };
    for (int i = 0; i < 512; i++) {
        int tr = ((i >> 6) & 7) * 255 / 7;
        int tg = ((i >> 3) & 7) * 255 / 7;
        int tb = (i & 7) * 255 / 7;
        int best = 0;
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
        for (int j = 17; j < 216; j++) {          // colour lattice
            if (!isSlot(j)) continue;
            const uint32_t rgb = slotRgb(j);
            int cr = (rgb >> 16) & 0xFF, cg = (rgb >> 8) & 0xFF, cb = rgb & 0xFF;
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
// colours themselves, the rest the colour lattice (buildRemap) — becomes the solid
// pair of its nearest ZX colour, the same limit every pair mode has.
void NEXTVID::rebuildPairMap() {
    if (!pairMap) return;
    c1Drain();
    static const uint8_t zxlvl[2] = { 205, 255 };
    for (int i = 0; i < 256; i++) {
        int tr, tg, tb;
        if (i < 16) {
            const int l = zxlvl[(i >> 3) & 1];
            tr = (i & 2) ? l : 0; tg = (i & 4) ? l : 0; tb = (i & 1) ? l : 0;
        } else if (isSlot(i)) {
            const uint32_t rgb = slotRgb(i);
            tr = (rgb >> 16) & 0xFF; tg = (rgb >> 8) & 0xFF; tb = rgb & 0xFF;
        } else {
            tr = tg = tb = 0;                     // never produced
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
    c1Drain();
    rawPal[pal][idx] = rgb333;
    lut[pal][idx] = remap[rgb333 & 0x1FF];
}

void NEXTVID::rebuildLut(uint8_t pal) {
    c1Drain();
    for (int i = 0; i < 256; i++)
        lut[pal][i] = remap[rawPal[pal][i] & 0x1FF];
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
    c1Drain();
    uint8_t* a = sprAttr[sprAttrSlot];
    a[sprAttrByte] = v;
    sprDirty = true;
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
    c1Drain();
    sprPatterns[sprPatWrite] = v;
    sprPatWrite = (sprPatWrite + 1) & 0x3FFF;
}

// Nextreg 0x34 keeps its OWN sprite index; only with lockstep (NR 0x09 bit 4)
// is it the port 0x303B index (and then it also selects the pattern). Sharing
// it unconditionally moved the port's pattern upload pointer whenever a game
// updated a sprite through nextregs (zxnext.vhd / ZEsarUX tbblue.c agree).
void NEXTVID::spriteNrSelect(uint8_t v) {
    if (NextReg::reg[0x09] & 0x10) spriteSlotSelect(v);
    else sprNrSlot = v & 0x7F;
}

void NEXTVID::spriteAttrDirect(uint8_t byteIdx, uint8_t v, bool inc) {
    c1Drain();
    const bool lock = NextReg::reg[0x09] & 0x10;
    uint8_t& slot = lock ? sprAttrSlot : sprNrSlot;
    if (byteIdx < 5) sprAttr[slot][byteIdx] = v;
    sprDirty = true;
    if (inc) {
        slot = (slot + 1) & 0x7F;
        if (lock) sprAttrByte = 0;
    }
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
    // copper.vhd 70-77: a mode change restarts the list only into 01 (run
    // once) or 11 (loop each frame); 10 resumes where the copper stopped.
    if (mode != prevMode && (mode & 1)) {
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

void NEXTVID::softReset() {
    c1Drain();
    palIdx = 0; pal9Second = false; palCtrl = 0;
    static const uint8_t defclip[4][4] = {
        { 0, 255, 0, 191 }, { 0, 255, 0, 191 }, { 0, 255, 0, 191 }, { 0, 159, 0, 255 },
    };
    for (int w = 0; w < 4; w++) {
        for (int i = 0; i < 4; i++) clip[w][i] = defclip[w][i];
        clipIdx[w] = 0;
    }
    copIndex = 0;
    copPC = 0;
    copCtrl = 0;
    copWaiting = false;
    cop16Second = false;
}

void NEXTVID::Reset() {
    c1Drain();
    if (!nx_ring) {
        // The core1 line queue (~4.6 KB). No RAM = render on core0 as before.
        uint8_t* q = (uint8_t*)tryMalloc(4 * NEXT_C1_RING + sizeof(NextLineState) * NEXT_C1_STATES
                                         + 4 * NEXT_C1_STATES);
        if (q) {
            nx_ring = (uint32_t*)q;
            nx_st = (NextLineState*)(q + 4 * NEXT_C1_RING);
            nx_st_job = (uint32_t*)(q + 4 * NEXT_C1_RING + sizeof(NextLineState) * NEXT_C1_STATES);
            memset(nx_st_job, 0, 4 * NEXT_C1_STATES);
        }
        Debug::log("[NXC1] core1 line renderer %s", q ? "ON" : "unavailable (no RAM)");
    }
    nx_have = false;
    g_next_c1_live = nx_ring != nullptr;
    if (!sprPatterns) {
        // One block, Next-boot only, never freed (the Next boundary reboots).
        // Layout: patterns 16K | copper 2K | rawPal 4K | lut 2K | sprAttr 640 |
        // lineBuf 384 | pairMap 256 | hiresRow 256 | sprRes 1280 | sprVis 128 |
        // remap 512 (all word-aligned: 27136 is a multiple of 4)
        static_assert(sizeof(SprRes) == 10, "sprRes is sized 128 x 10 in the block");
        uint8_t* blk = (uint8_t*)malloc(16384 + 2048 + 8*256*2 + 8*256 + 128*5 + 384 + 512 + 1408 + 512);
        sprPatterns = blk;
        copperMem   = blk + 16384;
        rawPal      = (uint16_t(*)[256])(blk + 16384 + 2048);
        lut         = (uint8_t(*)[256])(blk + 16384 + 2048 + 8*256*2);
        sprAttr     = (uint8_t(*)[5])(blk + 16384 + 2048 + 8*256*2 + 8*256);
        lineBuf     = blk + 16384 + 2048 + 8*256*2 + 8*256 + 128*5;
        pairMap     = lineBuf + 384;
        hiresRow    = pairMap + 256;
        sprRes      = (SprRes*)(hiresRow + 256);
        sprVis      = hiresRow + 256 + 1280;
        remap       = sprVis + 128;
        memset(blk, 0, 16384 + 2048 + 8*256*2 + 8*256 + 128*5 + 384 + 512 + 1408 + 512);
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
    sprDirty = true;
    sprNrSlot = 0;
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
    computeGeometry();
    xPaperOff = ((int)VIDEO::vga.xres - 256) / 2;
    lineIdx = 0;
    nextLineTstate = (uint32_t)VIDEO::tStatesPerLine << ESPectrum::multiplicator;
    skipFrame = false;
    framePacing.reset(time_us_32());
}

// Which raster line is fb row 0, and which fb row the paper starts on. The
// anchors are the 128K 50 Hz ones VIDEO::Reset sets. At 60 Hz (nextreg 0x05
// bit 2) the frame is 262 lines and the TOP border is 26 lines shorter: the
// paper starts 38 lines after the INT instead of 64 (z88dk zxn copper table —
// 50 Hz: 192 display + 56 bottom + 8 vsync + 56 top = 312; 60 Hz: 192 + 32 +
// 8 + 30 = 262). Keeping the 50 Hz anchor at 60 Hz put fb row 0 on line 39, so
// only 262 - 39 = 223 rows were ever rendered and rows 223..239 kept whatever
// was drawn there last — the menu footer, under NextZXOS (hw 2026-10-07).
// Returns true when the geometry moved.
bool NEXTVID::computeGeometry() {
    const int tpl = VIDEO::tStatesPerLine;
    const int shift = NextReg::raster().hz60 ? 26 : 0;   // Pentagon: 50 Hz only
    int fv = (VIDEO::tStatesBorder + tpl / 2) / tpl - shift;
    if (fv < 0) fv = 0;
    const int paper = (VIDEO::tStatesScreen + tpl / 2) / tpl - shift;
    const bool moved = fv != firstVisibleLine || paper - fv != paperTopRow;
    firstVisibleLine = fv;
    paperTopRow = paper - fv;
    s_bcol = (int16_t)(VIDEO::tStatesBorder - ((VIDEO::tStatesBorder + tpl / 2) / tpl) * tpl);
    s_bstep = NextReg::raster().kind == 2 ? 1 : 4;
    return moved;
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
    // A HALTed Z80 keeps fetching; in contended memory every fetch waits
    // (halt2int's HALT column). Step it one M1 at a time then, exactly like
    // CPU::FlushOnHaltTo does for the classic 48K/128K.
    if (__builtin_expect(contend, 0) && MemESP::ramContended[Z80::getRegPC() >> 13]) {
        while (CPU::tstates < stEnd) { Tick_Opcode(true); Z80::incRegR(1); }
        return;
    }
    while (CPU::tstates < stEnd) {
        uint32_t tgt = nextLineTstate;
        if (tgt > stEnd) tgt = stEnd;
        const uint32_t n = (tgt > CPU::tstates) ? ((tgt - CPU::tstates + 3) & ~3u) : 4u;
        CPU::tstates += n;
        Z80::incRegR((uint8_t)(n >> 2));
        while (CPU::tstates >= nextLineTstate) ScanlineWork();
    }
}

extern int ts_notice_x0, ts_notice_x1, ts_notice_y0, ts_notice_y1;   // Video.cpp
#if NEXT_TRACE
void nxErrPoll();   // Z80_JLS.cpp: ERR_NR turning #15 -> dump the RST 8 ring
#endif
// Gigascreen for the next frame (Config::gigascreen_onoff: 0 off, 1 on, 2 auto —
// Auto blends while VIDEO::gigascreenAutoFlip() keeps the countdown up, i.e. the
// program flips the displayed page through #7FFD bit 3 / NR 0x69 bit 6).
// The generic Gigascreen (prev-FB + blend slots 17..136) stays suspended on the
// Next: those slots are NEXTVID's colour lattice.
__attribute__((noinline)) void NEXTVID::gigsFrame() {
    const uint8_t mode = Config::gigascreen_onoff;
    if (!mode) {
        if (gigsPrev) { c1Drain(); Buffer::pfree(gigsPrev); gigsPrev = nullptr; }
        s_gigs = 0;
        return;
    }
    if (!gigsPrev) {
        gigsPrev = (uint8_t*)Buffer::palloc(192 * 128, Buffer::NEED_POINTER | Buffer::PREFER_PSRAM);
        s_gigsSeed = true;
        if (!gigsPrev) { s_gigs = 0; return; }
    }
    bool want = (mode == 1);
    if (mode == 2 && VIDEO::gigascreen_auto_countdown > 0) {
        VIDEO::gigascreen_auto_countdown--;
        want = true;
    }
    if (want && !s_gigsSeed) {
        // ULA palette (NR 0x43 bit 1 picks the second): the average of every
        // pair of its 16 ink colours, to the nearest renderable index. Rebuilt
        // per blended frame — 256 entries, and the palette may have moved.
        const uint16_t* pal = rawPal[(palCtrl & 0x02) ? 4 : 0];
        for (int i = 0; i < 16; i++)
            for (int j = 0; j < 16; j++) {
                const uint16_t a = pal[i] & 0x1FF, b = pal[j] & 0x1FF;
                const unsigned r = (((a >> 6) & 7) + ((b >> 6) & 7) + 1) >> 1;
                const unsigned g = (((a >> 3) & 7) + ((b >> 3) & 7) + 1) >> 1;
                const unsigned bl = ((a & 7) + (b & 7) + 1) >> 1;
                gigsBlend[(i << 4) | j] = (i == j) ? lut[(palCtrl & 0x02) ? 4 : 0][i]
                                                   : remap[(r << 6) | (g << 3) | bl];
            }
        s_gigs = 1;
    } else {
        s_gigs = 2;   // record only
    }
    s_gigsSeed = false;
}

IRAM_ATTR void NEXTVID::EndFrame() {
#if NEXT_TRACE
    nxErrPoll();
    if (!skipFrame) ++g_nextvid_frames;
#endif
    // fb rows the raster never reaches this frame (a 288-row fb at 60 Hz:
    // 262 lines for 288 rows) are drawn here from the last line's state, or
    // they would keep whatever was painted there before — UI included.
    if (!skipFrame) {
        int row = (int)totalLines - firstVisibleLine;
        if (row < 0) row = 0;
        for (; row < (int)VIDEO::vga.yres; row++) emitRow(row);
    }
    s_brdHead = s_brdCnt = 0;   // the frame's T-states restart
    // The machine timing (nextreg 0x03) and 50/60 Hz (0x05) change the whole
    // frame: apply at the frame boundary, after core1 has drawn every row of
    // the old geometry (RenderLayers reads paperTopRow live). The BIOS writes
    // the timing just before its soft reset into a personality, NextZXOS's TAP
    // Loader without any reset.
    {
        static uint8_t s_key = 0xFF;
        const uint8_t key = NextReg::rasterKey();
        if (key != s_key) {
            s_key = key;
            c1Drain();
            ESPectrum::nextRasterApply();
            computeGeometry();
        }
    }
    if (ESPectrum::nextRefreshCheckPending) {
        ESPectrum::nextRefreshCheckPending = false;
        ESPectrum::nextRefreshCheck();
    }
    // A fixed 1-in-64 skip made the BIOS update at <1 Hz even when the CPU
    // managed 50 emulated frames/s. Bound skipping by wall time instead;
    // when emulation is slow, every frame is drawn.
    skipFrame = framePacing.skip(ESPectrum::maxSpeed, time_us_32());
    lineIdx = 0;
    nextLineTstate = (uint32_t)VIDEO::tStatesPerLine << ESPectrum::multiplicator;
    totalLines = (CPU::statesInFrame >> ESPectrum::multiplicator) / VIDEO::tStatesPerLine;
    gigsFrame();
    if ((copCtrl >> 6) == 3) { // mode %11: restart the copper on vertical blank
        copPC = 0;
        copWaiting = false;
    }
}

// Wait states of a contended access starting now. Contention exists only at
// 3.5 MHz (multiplicator 0), so the line structure is nextLineTstate's: the
// current line is lineIdx, it began at nextLineTstate - tStatesPerLine. A
// paper line's 128-T window starts contCol T into the line BEFORE it (48K
// #14335 = line 63 col 223) and so crosses the line boundary.
inline __attribute__((always_inline)) unsigned NEXTVID::contendWait() {
    const int tpl = VIDEO::tStatesPerLine;
    int idx = (int)CPU::tstates - ((int)nextLineTstate - tpl) - contCol;
    int k = (int)lineIdx - contLine;
    if (idx < 0) { idx += tpl; k--; }
    return ((unsigned)k < 192u && (unsigned)idx < 128u) ? contWait[idx & 7] : 0u;
}

IRAM_ATTR void NEXTVID::Tick(unsigned int statestoadd, bool contended) {
    if (__builtin_expect(contended, 0) && contend) statestoadd += contendWait();
    CPU::tstates += statestoadd;
    while (CPU::tstates >= nextLineTstate) ScanlineWork();
}

IRAM_ATTR void NEXTVID::lineTick() {
    while (CPU::tstates >= nextLineTstate) ScanlineWork();
}

IRAM_ATTR void NEXTVID::Tick_Opcode(bool contended) {
    CPU::tstates += (__builtin_expect(contended, 0) && contend) ? 4u + contendWait() : 4u;
    while (CPU::tstates >= nextLineTstate) ScanlineWork();
}


#if NEXT_TRACE
// Compositor cost estimate for the 1 Hz NEXT: line: every 16th visible line
// is timed and extrapolated ×16 (~15 time_us_64 pairs per frame — noise-level
// overhead, but enough to see RenderLine's real share of cpu_frame_us).
volatile uint32_t g_nextvid_us = 0;
// Per-layer split of the same sampled lines (x16): ULA/LoRes, tilemap,
// Layer 2, sprites. nx_layer_sample is set by the sampling wrapper.
volatile uint32_t g_nextvid_layer_us[4] = {0, 0, 0, 0};
static bool nx_layer_sample = false;
#define NX_LT_BEGIN() uint64_t _lt0 = nx_layer_sample ? time_us_64() : 0
#define NX_LT_END(i) do { if (nx_layer_sample) { const uint64_t _t = time_us_64(); \
    g_nextvid_layer_us[i] += (uint32_t)(_t - _lt0) * 16; _lt0 = _t; } } while (0)
#endif
#if !NEXT_TRACE
#define NX_LT_BEGIN() do {} while (0)
#define NX_LT_END(i) do {} while (0)
#endif

// noinline: at -O3 it was inlined into Tick, Tick_Opcode, lineTick and
// haltAdvanceTo — four ~0.5 KB SRAM copies of a once-per-scanline function.
__attribute__((noinline)) IRAM_ATTR void NEXTVID::ScanlineWork() {
    uint8_t m = ESPectrum::multiplicator;
    if (lineIdx < totalLines) {
        // Copper runs before the line is drawn; raster line 0 = paper start
        uint32_t paperStart = (uint32_t)(firstVisibleLine + paperTopRow);
        copperLine((uint16_t)((lineIdx + totalLines - paperStart) % totalLines));
        int row = (int)lineIdx - firstVisibleLine;
        if (!skipFrame && row >= 0 && row < (int)VIDEO::vga.yres) emitRow(row);
        lineIdx++;
        nextLineTstate = (lineIdx + 1) * ((uint32_t)VIDEO::tStatesPerLine << m);
    } else {
        // frame over-run (turbo tail) — wait for EndFrame to restart
        nextLineTstate = 0xFFFFFFFF;
    }
}

// One fb row: posted to core1 with a register snapshot, or drawn here.
// noinline: ScanlineWork and EndFrame's tail fill both call it.
__attribute__((noinline)) IRAM_ATTR void NEXTVID::emitRow(int row) {
    if (g_next_c1_live) {
        // Post the line: capture, and open a new state slot only when
        // something the renderer reads has moved since the last line.
        NextLineState st;
        captureLine(st);
        if (s_brdCnt) nxBorderSegs(st, row, firstVisibleLine);
        if (!nx_have || !nxSameState(st, nx_st[nx_cur])) {
            const uint32_t k = (nx_cur + 1) % NEXT_C1_STATES;
            nxWaitUntil(nx_st_job[k]);
            nx_st[k] = st;
            nx_cur = k;
            nx_have = true;
        }
        if (nx_w - nx_r >= NEXT_C1_RING) nxWaitUntil(nx_w - NEXT_C1_RING + 1);
    }
    if (g_next_c1_live) {
        nx_ring[nx_w % NEXT_C1_RING] = (uint32_t)row | (nx_cur << 16);
        __dmb();                    // job + state visible before the index
        nx_w = nx_w + 1;
        nx_st_job[nx_cur] = nx_w;
    } else {
        captureLine(nx_sync);
        if (s_brdCnt) nxBorderSegs(nx_sync, row, firstVisibleLine);
        LS = &nx_sync;
#if NEXT_TRACE
        if ((row & 15) == 0) {
            uint64_t t0 = time_us_64();
            nx_layer_sample = true;
            RenderLine(row);
            nx_layer_sample = false;
            g_nextvid_us += (uint32_t)(time_us_64() - t0) * 16;
        } else
#endif
        RenderLine(row);
    }
}


// core1: render up to 8 posted lines per call (render_core interleaves the
// GS pump and the rest of its loop between batches).
IRAM_ATTR void NEXTVID::c1Pump() {
    for (int n = 0; n < 8 && nx_r != nx_w; n++) {
        __dmb();
        const uint32_t j = nx_ring[nx_r % NEXT_C1_RING];
        LS = &nx_st[j >> 16];
#if NEXT_TRACE
        if ((j & 15) == 0) {
            const uint64_t t0 = time_us_64();
            nx_layer_sample = true;
            RenderLine((int)(j & 0xFFFF));
            nx_layer_sample = false;
            g_nextvid_us += (uint32_t)(time_us_64() - t0) * 16;
        } else
#endif
        RenderLine((int)(j & 0xFFFF));
        __dmb();                            // the row is written before it is released
        nx_r = nx_r + 1;
    }
}
extern "C" IRAM_ATTR void next_render_core1_pump(void) { NEXTVID::c1Pump(); }

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
    if (LS->hires) {
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
    if ((LS->osd & 0x07) != 0 && row >= cy0 && row < cy0 + 16) {
        k0 = ((VIDEO::vga.xres >= 360) ? 188 : 168) >> 2;
        k1 = k0 + 144 / 4;
    }
    // OSD::notify banner (VIDEO::setNoticeCarve, 4-aligned): left to drawNotify,
    // or this row copy erased it on the very next frame (hw 2026-10-07).
    else if (row >= ts_notice_y0 && row < ts_notice_y1) {
        k0 = ts_notice_x0 >> 2;
        k1 = (ts_notice_x1 + 3) >> 2;
    }
    for (int k = 0; k < k0; k++) { const uint32_t v = src[k]; dst[k] = (v >> 16) | (v << 16); }
    for (int k = k1; k < words; k++) { const uint32_t v = src[k]; dst[k] = (v >> 16) | (v << 16); }
}

// Mid-line border changes: coloured runs, then the closing colour. Paper rows
// get the runs under the paper too; the ULA layer overwrites those 256 pixels.
// Flash: only rows that carry a change come here.
__attribute__((noinline)) static void nxPaintBorderRuns(uint8_t* fb, int xres, const uint8_t* ulaLut,
                                                        bool ulaNext, uint8_t borderIdx) {
    int x = 0;
    for (int i = 0; i < LS->brd.n; i++) {
        int xe = 2 * LS->brd.t[i];
        if (xe > xres) xe = xres;
        if (xe > x) {
            const uint8_t c = (LS->brd.c[i >> 1] >> ((i & 1) * 4)) & 7;
            memset(fb + x, ulaNext ? ulaLut[128 + c] : ulaLut[16 + c], xe - x);
            x = xe;
        }
    }
    if (x < xres) memset(fb + x, borderIdx, xres - x);
}

IRAM_ATTR void NEXTVID::RenderLayers(uint8_t* fb, int row) {
    int xres = VIDEO::vga.xres;
    // NR 0x43 active palettes (zxnext.vhd 5391-5393): bit 3 sprites,
    // bit 2 Layer 2, bit 1 ULA, bit 0 ULANext. (Bits 3 and 1 were swapped.)
    const uint8_t* ulaLut = lut[(LS->palCtrl & 0x02) ? 4 : 0];

    bool ulaNext = LS->palCtrl & 0x01;
    uint8_t borderIdx = ulaNext ? ulaLut[128 + (LS->border & 7)]
                                : ulaLut[16 + (LS->border & 7)];
    int y = row - paperTopRow;
    uint8_t r15 = LS->r15;

    if (LS->brd.n) nxPaintBorderRuns(fb, xres, ulaLut, ulaNext, borderIdx);
    // Debug > Paper off (VIDEO::paper_off): the paper rows are drawn as border
    // rows — the border (and its mid-line runs) shows through where the ULA
    // screen would be; tilemap / Layer 2 / sprites still draw over it.
    if (y < 0 || y >= 192 || VIDEO::paper_off) {
        if (!LS->brd.n) memset(fb, borderIdx, xres);
        // tilemap and Layer 2 (320x256 mode) cover the border area of the
        // 320x256 window; sprites reach it with the over-border bit
        if (LS->r6B & 0x80)
            RenderTilemapLine(fb, row);
        if ((LS->p123B & 0x02) || (LS->r69 & 0x80))
            RenderL2Line(fb, row, false);
        if ((r15 & 0x03) == 0x03) // sprites enabled + over border
            RenderSpritesLine(fb, y);
        return;
    }

    NX_LT_BEGIN();
    // left/right border
    if (!LS->brd.n) {
        memset(fb, borderIdx, xPaperOff);
        memset(fb + xPaperOff + 256, borderIdx, xres - xPaperOff - 256);
    }

    // ULA layer (classic attribute mode; ULANext later). ULA can be disabled
    // via nextreg 0x68 bit 7 — border colour shows through. LoRes (128x96,
    // reg 0x15 bit 7) replaces the ULA output.
    uint8_t* p = fb + xPaperOff;
    // NR 0x68 bit 7 disables the ULA layer, LoRes included (LoRes REPLACES the
    // ULA screen). TX-1696 runs NR 0x15 = 0x81 (LoRes on) with NR 0x68 = 0x80
    // and a tilemap: drawing LoRes anyway showed its 2x2 garbage through every
    // "ULA over" tile of the menu.
    if ((LS->r15 & 0x80) && !(LS->r68 & 0x80)) {
        uint8_t ly = (uint8_t)((y + LS->r33) % 192) >> 1;
        const uint8_t* base = LS->grmem +
            (ly < 48 ? ly * 128 : 0x2000 + (uint16_t)(ly - 48) * 128);
        uint8_t sxl = LS->r32;
        for (int x = 0; x < 256; x++)
            p[x] = ulaLut[base[(uint8_t)(x + sxl) >> 1]];
    }
    // Timex screen mode = port FF / NR 0x69 bits 2:0, forced to 000 while the
    // shadow screen (bank 7, NR 0x69 bit 6 = port 7FFD bit 3) is displayed —
    // zxula.vhd: "limit timex modes to bank 5 as bank 7 only has 8k bram".
    else if (!(LS->r68 & 0x80) && LS->hires && !(LS->r69 & 0x40)) {
        // Timex hi-res 512x192: screen 0 (0x4000) gives the left 8 pixels of
        // each 16-pixel column, screen 1 (0x6000) the right 8 — the TC2048
        // layout VIDEO's renderer uses, and its nibble LUT (ink/paper from port
        // FF bits 5:3, all BRIGHT). hiresRow gets the pair slots in linear
        // order; the paper carries HIRES_MARK so layers drawn over the ULA win.
        const uint16_t* hl = VIDEO::timexHiresLut();
        const uint8_t* bmp = LS->grmem + VIDEO::offBmp[y];
        uint32_t* hr = (uint32_t*)hiresRow;
        for (int col = 0; col < 32; col++) {
            const uint8_t e = bmp[col], o = bmp[col + 0x2000];
            hr[2 * col]     = (uint32_t)hl[e >> 4] | ((uint32_t)hl[e & 0x0F] << 16);
            hr[2 * col + 1] = (uint32_t)hl[o >> 4] | ((uint32_t)hl[o & 0x0F] << 16);
        }
        memset(p, HIRES_MARK, 256);
    }
    else if (!(LS->r68 & 0x80)) {
        // zxula.vhd vram_a: pixel = mode(0) & pixel address, attribute =
        // '1' & PIXEL address when mode(1) (hi-colour 8x1 attributes at
        // 0x6000), else mode(0) & attribute address. Mode 1 = the second
        // screen at 0x6000/0x7800, mode 2 = hi-colour, mode 3 = both (pixel
        // and attribute from the same 0x6000 byte, as the hardware does).
        // Hi-res modes (bit 2) other than 6 take this path with bits 1:0.
        const uint8_t tm = (LS->r69 & 0x40) ? 0 : (LS->r69 & 0x03);
        const uint16_t pOff = VIDEO::offBmp[y];
        const uint8_t* bmp = LS->grmem + pOff + ((tm & 1) << 13);
        const uint8_t* att = LS->grmem + ((tm & 2) ? 0x2000 + pOff
                                                   : VIDEO::offAtt[y] + ((tm & 1) << 13));
        if (ulaNext) {
            // ULANext attribute format: ink = attr & mask, paper = 128 + rest
            uint8_t mask = LS->r42;
            uint8_t shift = 0;
            for (uint8_t m = mask; m; m >>= 1) shift++;
            for (int col = 0; col < 32; col++) {
                uint8_t a = att[col];
                uint8_t b = bmp[col];
                uint8_t ink = ulaLut[a & mask];
                uint8_t pap = (mask == 255)
                    ? remap[rgb332_to_333(LS->r4A)]
                    : ulaLut[128 + (a >> shift)];
                for (int bit = 0; bit < 8; bit++)
                    p[bit] = (b & (0x80 >> bit)) ? ink : pap;
                p += 8;
            }
        } else if (LS->gigs && gigsPrev) {
            // Gigascreen: the line is blended only when the classic ULA is all
            // there is on it — Layer 2, sprites or the tilemap switch it off
            // (the row is still recorded, so the next frame has a valid prev).
            const bool blend = LS->gigs == 1 && !(LS->r15 & 0x01) && !(LS->r6B & 0x80) &&
                               !((LS->p123B & 0x02) || (LS->r69 & 0x80));
            uint32_t* gp = (uint32_t*)(gigsPrev + y * 128);
            for (int col = 0; col < 32; col++) {
                uint8_t a = att[col];
                uint8_t b = bmp[col] ^ (uint8_t)(-((a & LS->flashing) >> 7));
                uint8_t bright = (a & 0x40) >> 3;
                const uint8_t ci = (a & 7) | bright, cp = ((a >> 3) & 7) | bright;
                const uint8_t ink = ulaLut[ci], pap = ulaLut[16 + cp];
                const uint32_t prev = gp[col];
                uint32_t cur = 0;
                for (int bit = 0; bit < 8; bit++) {
                    const bool on = b & (0x80 >> bit);
                    const uint8_t c = on ? ci : cp;
                    cur |= (uint32_t)c << (bit * 4);
                    p[bit] = blend ? gigsBlend[((prev >> (bit * 4)) & 15) << 4 | c] : (on ? ink : pap);
                }
                gp[col] = cur;
                p += 8;
            }
        } else {
            for (int col = 0; col < 32; col++) {
                uint8_t a = att[col];
                uint8_t b = bmp[col] ^ (uint8_t)(-((a & LS->flashing) >> 7));
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

    NX_LT_END(0);
    // Tilemap (over ULA/LoRes, under Layer 2 and sprites)
    if (LS->r6B & 0x80)
        RenderTilemapLine(fb, row);
    NX_LT_END(1);

    // Layer 2, over ULA
    bool l2on = (LS->p123B & 0x02) || (LS->r69 & 0x80);
    uint8_t slu = (r15 >> 2) & 7;
    bool spritesUnderL2 = (slu == 1 || slu == 4); // L-S-U / L-U-S orders

    if (spritesUnderL2 && (r15 & 0x01)) {
        RenderSpritesLine(fb, y);
        NX_LT_END(3);
    }

    if (l2on)
        RenderL2Line(fb, row, false);
    NX_LT_END(2);

    if (!spritesUnderL2 && (r15 & 0x01)) {
        RenderSpritesLine(fb, y);
        NX_LT_END(3);
    }

    // Layer 2 palette-priority pixels sit above everything
    if (l2on && palHasPrio[(LS->palCtrl & 0x04) ? 5 : 1])
        RenderL2Line(fb, row, true);
    NX_LT_END(2);
}

// Layer 2: 256x192x8bpp (row-major) or 320x256x8bpp (column-major, covers
// the border window). prioPass re-draws only palette-priority pixels after
// the sprites so they end up on top.
IRAM_ATTR void NEXTVID::RenderL2Line(uint8_t* fb, int row, bool prioPass) {
    uint8_t pal = (LS->palCtrl & 0x04) ? 5 : 1;
    const uint8_t* l2Lut = lut[pal];
    const uint16_t* raw = rawPal[pal];
    uint8_t transp = LS->r14;
    uint8_t mode = (LS->r70 >> 4) & 3;
    uint8_t palOfs = (LS->r70 & 0x0F) << 4;
    uint16_t bank = (uint16_t)(LS->r12 & 0x7F) * 2;

    if (mode == 0) {
        // 256x192, aligned with the paper area
        int y = row - paperTopRow;
        if (y < 0 || y >= 192) return;
        if (y < LS->clip[0][2] || y > LS->clip[0][3]) return;
        uint16_t sy = y + LS->r17;
        if (sy >= 192) sy -= 192;
        uint16_t page = bank + (sy >> 5);
        if (page >= MemESP::NEXT_PAGES - 1u) return;
        const uint8_t* src = MemESP::nextRamPtr[page] + (sy & 31) * 256;
        uint8_t sx = LS->r16 + LS->clip[0][0];
        uint8_t* q = fb + xPaperOff + LS->clip[0][0];
        int count = (int)LS->clip[0][1] - (int)LS->clip[0][0] + 1;
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
        int cy1 = LS->clip[0][2], cy2 = LS->clip[0][3];
        if (ty < cy1 || ty > cy2) return;
        uint16_t scrollX = ((LS->r71 & 1) << 8) | LS->r16;
        uint8_t sy = (uint8_t)(ty + LS->r17);
        int xOrig = xPaperOff - 32;
        int xres = VIDEO::vga.xres;
        int cx1 = LS->clip[0][0] * 2, cx2 = LS->clip[0][1] * 2 + 1;
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

// Tilemap text mode (NR 0x6B bit 3) — one tile's span of a line. 1bpp: pixel
// = bit (7 - x) of the row byte, palette index = attr 7:1 + pixel; never the
// NR 0x4C index test: a text pixel is transparent when its colour equals NR 0x14
// (tilemap.vhd 385-386, zxnext.vhd 7109). 80 columns are drawn at half width,
// so the pixel pair is ORed and 1-pixel strokes of the font survive (deviation).
// Flash on purpose: text mode is rare and this keeps ~1 KB out of every board's
// SRAM (RenderTilemapLine is RAM-resident).
__attribute__((noinline)) static void tmTextSpan(uint8_t* o, int n, uint8_t b, int u0, int step,
                                                 uint8_t palHi, const uint8_t* lutTM,
                                                 const uint16_t* rawTM, uint8_t r14) {
    for (int k = 0; k < n; k++) {
        const int u = u0 + k * step;
        uint8_t bit = (b >> (7 - u)) & 1;
        if (step == 2 && u < 7) bit |= (b >> (6 - u)) & 1;
        const uint8_t idx = (uint8_t)(palHi | bit);
        if (((rawTM[idx] & 0x1FF) >> 1) != r14) o[k] = lutTM[idx];
    }
}

// Tilemap: 40x32 (or 80x32 rendered horizontally halved) tiles of 8x8x4bpp.
// Map and tile definitions live in bank 5 (offsets from nextreg 0x6E/0x6F).
// The tilemap window is 320x256 and covers the border area around the paper.
IRAM_ATTR void NEXTVID::RenderTilemapLine(uint8_t* fb, int row) {
    uint8_t ctrl = LS->r6B;
    int ty = row - (paperTopRow - 32);
    if (ty < 0 || ty >= 256) return;
    if (ty < LS->clip[3][2] || ty > LS->clip[3][3]) return;

    bool cols80 = ctrl & 0x40;
    bool noAttr = ctrl & 0x20;
    // tilemap.vhd 191-195: bit 3 text mode (1bpp, 8 bytes a tile, attr 7:1 =
    // palette, no mirror/rotate), bit 1 512 tiles (attr bit 0 = tile bit 8, and
    // the tilemap goes UNDER the ULA), bit 0 tilemap always on top of the ULA.
    const bool text = ctrl & 0x08, m512 = ctrl & 0x02, onTop = ctrl & 0x01;
    const uint8_t palNo = (ctrl & 0x10) ? 7 : 3;
    const uint8_t* lutTM = lut[palNo];
    const uint16_t* rawTM = rawPal[palNo];
    uint8_t transp = LS->r4C & 0x0F;
    // NR 0x6E/0x6F: bit 7 = bank 7 instead of bank 5, bits 5:0 = 256-byte
    // offset; the address wraps inside the 16K bank (tilemap.vhd 399-401).
    const uint8_t* mapBank  = MemESP::ram[(LS->r6E & 0x80) ? 7 : 5].direct();
    const uint8_t* defsBank = MemESP::ram[(LS->r6F & 0x80) ? 7 : 5].direct();
    const uint16_t defsOff = (uint16_t)(LS->r6F & 0x3F) << 8;
    const uint8_t* map  = mapBank + (uint16_t)(LS->r6E & 0x3F) * 256;
    uint16_t scrollX = ((LS->r2F & 3) << 8) | LS->r30;
    uint8_t sy = (uint8_t)(ty + LS->r31); // wraps at 256
    int cols = cols80 ? 80 : 40;
    int entrySz = noAttr ? 1 : 2;
    const uint8_t* mrow = map + (sy >> 3) * cols * entrySz;
    uint8_t inRow = sy & 7;
    const int xOrig = xPaperOff - 32;
    const int xres = VIDEO::vga.xres;
    // Visible tx range: the clip window (in half-columns) and the framebuffer.
    int t0 = LS->clip[3][0] * 2, t1 = LS->clip[3][1] * 2 + 1;
    if (t0 < -xOrig) t0 = -xOrig;
    if (t1 > 319) t1 = 319;
    if (t1 > xres - 1 - xOrig) t1 = xres - 1 - xOrig;
    uint8_t* const out = fb + xOrig;

    // Tile by tile: one map entry, one attribute decode and the tile's 8
    // source pixels per tile instead of per output pixel. 80-column mode
    // renders 640 source pixels halved: every other source pixel.
    const bool ulaOff = LS->r68 & 0x80;
    const int step = cols80 ? 2 : 1;
    const int wrap = cols80 ? 640 : 320;
    int tx = t0;
    while (tx <= t1) {
        int sx = (tx * step + scrollX) % wrap;
        const int u0 = sx & 7;
        int n = (8 - u0 + step - 1) / step;        // output pixels left in this tile
        if (n > t1 - tx + 1) n = t1 - tx + 1;
        const uint8_t* e = mrow + (sx >> 3) * entrySz;
        const uint8_t tile = e[0];
        const uint8_t attr = noAttr ? LS->r6C : e[1];
        // The ULA is drawn OVER this tile when attr bit 0 is set (512-tile
        // mode: always), unless the tilemap is forced on top (tilemap.vhd 388).
        // With the ULA layer off (NR 0x68 bit 7) there is nothing over it.
        // (Approximation: our ULA has no transparent pixels, so "under" = hidden.)
        const bool below = !onTop && ((attr & 0x01) || m512);
        const uint16_t tnum = (uint16_t)(tile | ((m512 && (attr & 0x01)) ? 0x100 : 0));
        if (!below || ulaOff) {
            if (text) {
                tmTextSpan(out + tx, n, defsBank[(uint16_t)(defsOff + tnum * 8 + inRow) & 0x3FFF],
                           u0, step, (uint8_t)(attr & 0xFE), lutTM, rawTM, LS->r14);
            } else {
                const bool xm = attr & 0x08, ym = attr & 0x04, rot = attr & 0x02;
                const uint16_t defBase = (uint16_t)(defsOff + tnum * 32);
                const uint8_t palHi = attr & 0xF0;
                for (int k = 0; k < n; k++) {
                    uint8_t u = (uint8_t)(u0 + k * step), vv = inRow;
                    if (xm) u = 7 - u;                                       // X mirror
                    if (ym) vv = 7 - vv;                                     // Y mirror
                    if (rot) { const uint8_t t = u; u = vv; vv = 7 - t; }    // rotate
                    const uint8_t b = defsBank[(uint16_t)(defBase + vv * 4 + (u >> 1)) & 0x3FFF];
                    const uint8_t pix = (u & 1) ? (b & 0x0F) : (b >> 4);
                    if (pix != transp) out[tx + k] = lutTM[pix | palHi];
                }
            }
        }
        tx += n;
    }
}

// Resolve all 128 attribute sets against their anchors, in sprite order —
// the one order the anchor chain is defined in (a relative sprite belongs to
// the last anchor BEFORE it), independent of the draw priority. Semantics are
// ZEsarUX tbblue.c tbsprite_do_overlay line for line:
//  - attr 4 bits 7:6 == 01 marks a RELATIVE sprite; anything else (incl. a
//    4-byte sprite, E=0) is an anchor. Rendering those as independent 8-bit
//    anchors put every part of a composite sprite at its small signed offset
//    near the display's top-left corner (hw: TX-1696's broken ship/bosses).
//  - relative: X/Y are signed 8-bit offsets from the anchor, visible only if
//    the anchor is, 4bpp/8bpp from the anchor, N6 in attr 4 bit 5, attr 2 bit
//    0 adds the anchor's palette offset, attr 4 bit 0 its pattern number.
//  - an anchor with attr 4 bit 5 (T) makes its relatives UNIFIED: the anchor's
//    rotate / mirror / scale transform the offsets and the parts as one sprite.
IRAM_ATTR void NEXTVID::resolveSprites() {
    int ax = 0, ay = 0, apat = 0, azx = 0, azy = 0;
    uint8_t apal = 0;
    bool avis = false, a4bpp = false, arot = false, axm = false, aym = false, unified = false;
    uint8_t n = 0;
    for (int s = 0; s < 128; s++) {
        const uint8_t* a = sprAttr[s];
        const uint8_t a4 = (a[3] & 0x40) ? a[4] : 0;
        const bool rel = (a4 & 0xC0) == 0x40;
        bool vis = a[3] & 0x80;
        bool xm = a[2] & 0x08, ym = a[2] & 0x04, rot = a[2] & 0x02;
        int x = a[0] | ((a[2] & 0x01) << 8);
        int y = a[1];
        uint8_t pal = a[2] & 0xF0;
        int idx = a[3] & 0x3F;
        int zx = (a4 >> 3) & 3, zy = (a4 >> 1) & 3;
        bool is4;
        if (!rel) {
            y |= (a4 & 0x01) << 8;
            is4 = a4 & 0x80;
            if (is4) idx = idx * 2 + ((a4 >> 6) & 1);
            avis = vis; a4bpp = is4; unified = a4 & 0x20;
            ax = x; ay = y; apal = pal; apat = idx;
            axm = xm; aym = ym; arot = rot; azx = zx; azy = zy;
        } else {
            vis = vis && avis;
            is4 = a4bpp;
            if (is4) idx = idx * 2 + ((a4 >> 5) & 1);
            x = (int8_t)a[0];
            y = (int8_t)a[1];
            if (a[2] & 0x01) pal = (uint8_t)((pal + apal) & 0xF0);
            if (a4 & 0x01) idx = (idx + apat) & (is4 ? 127 : 63);
            if (unified) {
                zx = azx; zy = azy;
                if (arot) {
                    rot = !rot;
                    const int ox = x; x = -y; y = ox;
                    const bool om = xm;
                    xm = rot ? ym : !ym;
                    ym = rot ? om : !om;
                }
                if (axm) { xm = !xm; x = -x; }
                if (aym) { ym = !ym; y = -y; }
                x <<= zx; y <<= zy;
            }
            x = (x + ax) & 0x1FF;
            y = (y + ay) & 0x1FF;
        }
        SprRes& r = sprRes[s];
        r.x = (int16_t)x;
        r.y = (int16_t)(y > 255 ? y - 512 : y);
        r.pat = (uint16_t)(is4 ? idx * 128 : idx * 256);
        r.flags = (xm ? SR_XM : 0) | (ym ? SR_YM : 0) | (rot ? SR_ROT : 0) | (is4 ? SR_4BPP : 0);
        r.pal = pal;
        r.xs = (uint8_t)zx; r.ys = (uint8_t)zy;
        if (vis) sprVis[n++] = (uint8_t)s;
    }
    sprVisN = n;
    sprDirty = false;
}

// Render all visible sprites crossing paper line y (0-191). Sprite
// coordinate space puts (32,32) at the paper top-left corner; rendering is
// clipped to the paper area for now (sprites-over-border comes later).
IRAM_ATTR void NEXTVID::RenderSpritesLine(uint8_t* fb, int y) {
    if (sprDirty) resolveSprites();   // attribute writers drain core1 first
    const uint8_t* sprLut = lut[(LS->palCtrl & 0x08) ? 6 : 2];   // NR 0x43 bit 3
    const uint8_t transp = LS->r4B;
    const int sline = y + 32;
    if (sline < 0 || sline > 255) return;
    // NR 0x15 bit 6: 1 = sprite 0 on top, 0 = sprite 127 on top (ZEsarUX;
    // was inverted). Painter's order: the top one is drawn last.
    const bool zeroOnTop = LS->r15 & 0x40;
    const bool overBorder = LS->r15 & 0x02;
    // Everything that does not depend on the pixel is decided here, once per
    // line / per sprite: the X window (paper or border, clip window, the
    // framebuffer edges) becomes a px range, and the Y clip a line test.
    int lo = overBorder ? -32 : 0, hi = overBorder ? 287 : 255;
    if (!overBorder) {
        if (y < LS->clip[1][2] || y > LS->clip[1][3]) return;
        if (lo < LS->clip[1][0]) lo = LS->clip[1][0];
        if (hi > LS->clip[1][1]) hi = LS->clip[1][1];
    }
    if (lo < -xPaperOff) lo = -xPaperOff;
    if (hi > (int)VIDEO::vga.xres - 1 - xPaperOff) hi = (int)VIDEO::vga.xres - 1 - xPaperOff;
    if (lo > hi) return;
    uint8_t coverage[40]; // 320-bit line coverage for the collision flag
    memset(coverage, 0, sizeof(coverage));
    uint8_t* const fbp = fb + xPaperOff;      // indexed by paper x
    int rendered = 0;
    const int nv = sprVisN;
    uint8_t collided = 0;

    for (int i = 0; i < nv; i++) {
        const SprRes& r = sprRes[sprVis[zeroOnTop ? nv - 1 - i : i]];
        const int ys = r.ys, xs = r.xs;
        const int rowIn = sline - r.y;
        if (rowIn < 0 || rowIn >= (16 << ys)) continue;
        if (++rendered > 100) { sprFlags |= 0x02; break; } // per-line overflow

        // Paper x of pixel 0. x > 319 is left of the display; a sprite whose
        // pixels run past 319 is off the right edge either way, so the 512 wrap
        // needs no per-pixel test.
        const int x0 = (r.x > 319 ? r.x - 512 : r.x) - 32;
        const int width = 16 << xs;
        int p0 = lo - x0, p1 = hi - x0;            // px range, inclusive
        if (p0 < 0) p0 = 0;
        if (p1 > width - 1) p1 = width - 1;
        if (p0 > p1) continue;

        // The 16 source pixels of this sprite row, mirror/rotate applied;
        // 0xFFFF = transparent.
        uint16_t row[16];
        uint8_t v = rowIn >> ys;
        if (r.flags & SR_YM) v = 15 - v;
        const uint8_t* pat = sprPatterns + r.pat;
        const bool xm = r.flags & SR_XM, rot = r.flags & SR_ROT;
        if (r.flags & SR_4BPP) {
            const uint8_t tr = transp & 0x0F;
            for (int u = 0; u < 16; u++) {
                const int su = xm ? 15 - u : u;
                const int uu = rot ? v : su, vv = rot ? 15 - su : v;
                const uint8_t b = pat[vv * 8 + (uu >> 1)];
                const uint8_t px = (uu & 1) ? (b & 0x0F) : (b >> 4);
                row[u] = (px == tr) ? 0xFFFF : (uint16_t)(px | r.pal);
            }
        } else {
            for (int u = 0; u < 16; u++) {
                const int su = xm ? 15 - u : u;
                const int uu = rot ? v : su, vv = rot ? 15 - su : v;
                const uint8_t px = pat[vv * 16 + uu];
                row[u] = (px == transp) ? 0xFFFF : (uint16_t)(uint8_t)(px + r.pal);
            }
        }

        for (int px = p0; px <= p1; px++) {
            const uint16_t c = row[px >> xs];
            if (c == 0xFFFF) continue;
            const int xp = x0 + px;
            const unsigned cbit = (unsigned)(xp + 32);
            const uint8_t mask = 1 << (cbit & 7);
            collided |= coverage[cbit >> 3] & mask;
            coverage[cbit >> 3] |= mask;
            fbp[xp] = sprLut[c];
        }
    }
    if (collided) sprFlags |= 0x01;
}


