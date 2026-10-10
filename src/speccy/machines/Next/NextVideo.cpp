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
#include "speccy/z80/MemGates.h"
#include "hardware/sync.h"
#include "pico/time.h"
#include "app/TryAlloc.h"
#include "app/Buffer.h"
#include "app/Config.h"
#include "speccy/devices/Z80DMA.h"

// Hot render path in SRAM, like the rest of the video code
#undef IRAM_ATTR
#define IRAM_ATTR __not_in_flash("nextvid")

#pragma GCC optimize("O3")

uint16_t (*NEXTVID::rawPal)[256] = nullptr;
// Palette writes never wait for core1 either (TX-1696: ~70 a frame plus the
// copper's, 10 ms/frame of core0 waiting, hw 2026-10-09). rawPal/lut are the
// RENDERER's copy; core0 keeps cRaw (what the guest reads back) and logs every
// entry {pal, idx, raw} into palLog; a posted line carries the log position and
// the renderer applies the log up to it (palApply) before composing the line.
static uint16_t (*cRaw)[256] = nullptr;      // [8][256] core0's view
static uint32_t* palLog = nullptr;
#if NEXT_TRACE
extern volatile uint32_t g_nx_logapply_spr, g_nx_logapply_pal;   // defined with g_nextvid_layer_us
#endif           // [PAL_LOG] pal << 24 | idx << 16 | raw
static constexpr uint32_t PAL_LOG = 512;
static volatile uint32_t pal_w = 0, pal_app = 0;
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
NEXTVID::SprRender* NEXTVID::sprR = nullptr;
uint32_t* NEXTVID::sprLog = nullptr;
// sprLog positions, free-running: core0 writes spr_w, the renderer spr_app.
static constexpr uint32_t SPR_LOG = 1024;
static volatile uint32_t spr_w = 0, spr_app = 0;

uint8_t* NEXTVID::copperMem = nullptr;
uint16_t NEXTVID::copIndex = 0;
uint16_t NEXTVID::copPC = 0;
uint8_t  NEXTVID::copCtrl = 0;
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
    uint32_t sprSeq;         // sprite attribute log position (NEXTVID::sprLog)
    uint32_t palSeq;         // palette log position (palLog)
    uint8_t clip[4][4];
    uint8_t r12, r14, r15, r16, r17, r2F, r30, r31, r32, r33, r42, r4A, r4B, r4C, r68, r69, r6B, r6C, r6E, r6F, r70, r71;
    uint8_t p123B, palCtrl, border, flashing, hires, osd;
    uint8_t gigs;            // Gigascreen this frame (s_gigs)
    uint8_t attOvr;          // 1 = attr[] replaces the ULA attribute row (DMA multicolour)
    uint8_t r26, r27;        // ULA scroll X / Y (keeps the struct word-aligned)
    uint8_t r6A, _pad6A[3];  // LoRes: Radastan, display-file XOR, palette offset
    // Border colour changes INSIDE the row (OUT #FE mid-line): segment i ends
    // at fb x 2*brd.t[i] (T-states into the row, cell-aligned) in colour nibble i of brd.c;
    // the rest of the row is `border`. n = 0 for an ordinary row (every byte
    // zero, so it still dedups). 15 runs: a colour-bar test changes #FE eight
    // times a row, and OUT (C),r every 12 T fits ~15 into the 180-T window.
    // 24 bytes: 64 queued states = 1.5 KB of heap.
    struct { uint8_t t[15]; uint8_t n; uint8_t c[8]; } brd;
    // The ULA attribute row as the per-scanline DMA left it (Z80DMA's write-
    // order shadow, see captureAttrAfterTransfer): copied when the line is
    // posted, because core1 reads guest RAM later, when the charrow already
    // holds its LAST row (NaPICu: 8x8 blocks instead of 8x1 art, hw 2026-10-08).
    uint8_t attr[32];
};
static const NextLineState* LS = nullptr;
// Per-layer RGB line buffers of the compositor (RenderLayers), [384] each, in the Next heap block.
static uint16_t *nxU, *nxT, *nxL, *nxS;
// The line's Layer 2 / sprite renderers wrote at least one opaque pixel into
// nxL / nxS. A layer that is enabled but empty on this row is dropped from the
// mix (NXModPlayer: L2 on, all transparent, no sprites — mix 8.8 of 24 ms of
// core1 a frame, hw 2026-10-10).
static bool nxRowL2, nxRowSpr;
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
    st.attOvr = 0;
    st.r26 = r[0x26];
    st.r6A = r[0x6A];
    st._pad6A[0] = st._pad6A[1] = st._pad6A[2] = 0;
    st.sprSeq = spr_w;
    st.palSeq = pal_w;
    st.r27 = r[0x27];
    uint32_t* w = (uint32_t*)&st.brd;   // no libc memset on this path (flash)
    for (unsigned i = 0; i < sizeof st.brd / 4; i++) w[i] = 0;
    w = (uint32_t*)st.attr;
    for (unsigned i = 0; i < sizeof st.attr / 4; i++) w[i] = 0;
}

// The DMA attribute shadow for this row, if the per-scanline DMA wrote one
// into the page on show (Pentagon applies the same rule in MainScreen_Blank).
// Only reached while the shadow exists (a DMA into an attribute area).
__attribute__((noinline)) static void nxAttrShadow(NextLineState& st, int row, int paperTop) {
    const int y = row - paperTop;
    if (y < 0 || y >= 192 || !Z80DMA::dma_attr_valid[y]) return;
    if (Z80DMA::dma_attr_page[y >> 3] != st.grmem) return;
    if ((st.r69 & 0x43) && !(st.r69 & 0x40)) return;   // Timex screen 1 / hi-colour: other bytes
    const uint32_t* src = (const uint32_t*)&Z80DMA::dma_attr_shadow[y * 32];
    uint32_t* d = (uint32_t*)st.attr;
    for (int i = 0; i < 8; i++) d[i] = src[i];
    st.attOvr = 1;
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
// Occupancy of the line ring, sampled at every publish. This is the number that
// decides whether NEXT_C1_RING can shrink (it is heap: 320 x 4 B + the state
// slots): a max well below the size means core1 keeps up line by line, a max
// pinned at the size means the poster is waiting on a full ring — which
// haltAdvanceTo always does after a HALT, because it posts a whole frame at
// once. Printed once a second on the NEXT_TRACE `NEXT cpu:` line.
volatile uint32_t g_nx_ring_max = 0, g_nx_ring_now = 0;
volatile uint32_t g_nx_ring_size = NEXT_C1_RING;   // one source of truth for the trace
static NextLineState  nx_sync;               // the queue-off path
// Mid-line palette writes by the copper (copper.vhd WAIT H, 8-pixel steps):
// recorded between the paper's left edge and the line end with the fb x they
// take effect at; the row is then drawn in segments (renderRowSegments).
struct NxPalDelta { uint16_t x; uint8_t pal, idx; uint16_t old, neu; };
static constexpr int NX_PAL_DELTAS = 32;
static NxPalDelta s_pd[NX_PAL_DELTAS];
static uint8_t  s_pdn = 0;
static bool     s_pdRec = false;       // ScanlineWork is inside the line's copper span
static int32_t  s_copT = 0;            // copper clock, 28 MHz = 1/4 hc_ula, in the line being run
static int16_t  s_copXBase = 0;        // fb x of hc_ula 0 for the line being run
static const NextLineState* s_preCap = nullptr;   // emitRow: state already captured
static bool     s_lineReady = false;   // RenderLine: lineBuf already composed
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
#if NEXT_TRACE
// Perf baseline (NEXT perf: line): how often and how long core0 waits for
// core1's line queue, and how long core1 spends rendering.
volatile uint32_t g_nx_drain_n = 0, g_nx_drain_us = 0, g_nx_c1_us = 0;
volatile uint32_t g_nx_drain_src[4] = {};    // waits per source (c1Drain's src)
#endif
IRAM_ATTR void NEXTVID::c1Drain(uint8_t src) {
    (void)src;
    if (nx_r == nx_w) return;
    const uint64_t t0 = time_us_64();
    while (nx_r != nx_w) {
        if (time_us_64() - t0 > 100000) { nxQueueDead("stuck"); return; }
        tight_loop_contents();
    }
#if NEXT_TRACE
    g_nx_drain_n = g_nx_drain_n + 1;
    g_nx_drain_src[src & 3] = g_nx_drain_src[src & 3] + 1;
    g_nx_drain_us = g_nx_drain_us + (uint32_t)(time_us_64() - t0);
#endif
}

volatile uint32_t g_nx_qwait_us = 0, g_nx_qwait_max = 0;   // core0 waiting for queue room / a slot
static void __not_in_flash_func(nxWaitUntil)(uint32_t w1) {
    if ((int32_t)(nx_r - w1) >= 0) return;
    const uint64_t t0 = time_us_64();
    while ((int32_t)(nx_r - w1) < 0) {
        if (time_us_64() - t0 > 100000) { nxQueueDead("stuck"); return; }
        tight_loop_contents();
    }
    const uint32_t d = (uint32_t)(time_us_64() - t0);
    g_nx_qwait_us = g_nx_qwait_us + d;
    if (d > g_nx_qwait_max) g_nx_qwait_max = d;
}

// Guest RAM a posted line reads, as (16K bank, offset range) tests: the
// tilemap row and tile definitions, the ULA / LoRes row, Layer 2 (whole
// banks). Conservative where the exact bytes would cost more than they save.
static bool nxRangeHit(uint8_t b, uint32_t a0, uint32_t a1, uint8_t bank, uint16_t o0, uint16_t o1) {
    return b == bank && a0 <= o1 && a1 >= o0;
}
static bool nxLineReads(const NextLineState& st, int row, int paperTop, uint8_t bank, uint16_t o0, uint16_t o1) {
    // Layer 2 (NR 0x12, 3 banks, 5 in 320/640 modes)
    if ((st.p123B & 0x02) || (st.r69 & 0x80)) {
        const uint8_t l2 = st.r12 & 0x7F, n = (st.r70 & 0x30) ? 5 : 3;
        if (bank >= l2 && bank < l2 + n) return true;
    }
    // tilemap: this line's map row + the definitions
    if (st.r6B & 0x80) {
        const int ty = row - (paperTop - 32);
        if (ty >= 0 && ty < 256) {
            const uint8_t mb = (st.r6E & 0x80) ? 7 : 5, db = (st.r6F & 0x80) ? 7 : 5;
            const int rowBytes = ((st.r6B & 0x40) ? 80 : 40) * ((st.r6B & 0x20) ? 1 : 2);
            const uint8_t sy = (uint8_t)(ty + st.r31);
            const uint32_t m0 = ((uint32_t)(st.r6E & 0x3F) << 8) + (uint32_t)(sy >> 3) * rowBytes;
            if (m0 + rowBytes > 0x4000) { if (mb == bank) return true; }   // wraps: whole bank
            else if (nxRangeHit(mb, m0, m0 + rowBytes - 1, bank, o0, o1)) return true;
            const uint32_t d0 = (uint32_t)(st.r6F & 0x3F) << 8;
            const uint32_t dl = (st.r6B & 0x08) ? 0x800 : (st.r6B & 0x02) ? 0x4000 : 0x2000;   // text / 512 / 256 tiles
            if (d0 + dl > 0x4000) { if (db == bank) return true; }
            else if (nxRangeHit(db, d0, d0 + dl - 1, bank, o0, o1)) return true;
        }
    }
    // ULA / LoRes: paper rows only, unless the ULA is off
    const int y = row - paperTop;
    if (y >= 0 && y < 192 && !(st.r68 & 0x80)) {
        const uint8_t ub = (st.grmem == MemESP::ram[7].direct()) ? 7 : 5;
        if (ub != bank) return false;
        if (st.r15 & 0x80) {
            const uint8_t ly = (uint8_t)((y + st.r33) % 192);
            if (st.r6A & 0x20) {                       // Radastan: 64 bytes a row pair
                const uint32_t a = (((st.r69 ^ (st.r6A >> 4)) & 1) ? 0x2000 : 0) + (uint32_t)(ly >> 1) * 64;
                return nxRangeHit(ub, a, a + 63, bank, o0, o1);
            }
            const uint8_t ry = ly >> 1;
            const uint32_t a = ry < 48 ? (uint32_t)ry * 128 : 0x2000 + (uint32_t)(ry - 48) * 128;
            return nxRangeHit(ub, a, a + 127, bank, o0, o1);
        }
        if (nxRangeHit(ub, 0, 0x1AFF, bank, o0, o1)) return true;
        if ((st.r69 & 0x03) || st.hires) return nxRangeHit(ub, 0x2000, 0x3AFF, bank, o0, o1);
    }
    return false;
}

// core0, before guest RAM [o0, o1] of 16K bank `bank` is overwritten faster than
// core1 can follow (the batched DMA): wait only for the posted lines that read it
// — the newest such job — instead of the whole queue (TX-1696 rewrites ~24
// tilemap rows a frame by DMA; a full drain each cost 2.5-3.4 ms, hw 2026-10-09).
void NEXTVID::c1DrainRange(uint8_t bank, uint16_t o0, uint16_t o1) {
    if (!g_next_c1_live) return;
    const uint32_t r = nx_r;
    for (uint32_t i = nx_w; i != r; ) {
        i--;
        if ((int32_t)(nx_r - i) > 0) return;   // core1 already past it
        const uint32_t j = nx_ring[i % NEXT_C1_RING];
        if (nxLineReads(nx_st[j >> 16], (int)(j & 0xFFFF), paperTopRow, bank, o0, o1)) {
#if NEXT_TRACE
            const uint64_t t0 = time_us_64();
#endif
            nxWaitUntil(i + 1);
#if NEXT_TRACE
            g_nx_drain_n = g_nx_drain_n + 1;
            g_nx_drain_src[0] = g_nx_drain_src[0] + 1;
            g_nx_drain_us = g_nx_drain_us + (uint32_t)(time_us_64() - t0);
#endif
            return;
        }
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

// Renderer (or core0 with the queue drained): apply the palette log up to `upto`.
IRAM_ATTR void NEXTVID::palApply(uint32_t upto) {
    uint32_t i = pal_app;
    // A line captured before core0 applied the log ahead (renderRowSegments,
    // rebuildLut, a full log) names an older position: nothing to do. `!=` here
    // ran ~4e9 iterations (hw 2026-10-09: !Copper.snx froze core0).
    if ((int32_t)(upto - i) <= 0) return;
    __dmb();
#if NEXT_TRACE
    g_nx_logapply_pal = g_nx_logapply_pal + (upto - i);
#endif
    for (; i != upto; i++) {
        const uint32_t e = palLog[i % PAL_LOG];
        const uint8_t p = (e >> 24) & 7, x = (uint8_t)(e >> 16);
        const uint16_t v = (uint16_t)e;
        rawPal[p][x] = v;
        lut[p][x] = remap[v & 0x1FF];
    }
    __dmb();
    pal_app = i;
}

#if NEXT_TRACE
volatile uint32_t g_nx_palw = 0;   // palette writes through nrHot (NEXT: pal/s)
#endif
// The palette NEXTREGs (0x40 / 0x41 / 0x44) without NextReg::write's flash
// switch: from flash each write cost ~160 XIP fetches and ~1 us, and TX-1696
// fills 256-entry palettes dozens of times a frame (12k writes = 12 ms, the
// frames of 26-35 ms; hw 2026-10-09). Same effect as palIndex / palValue8 / 9.
static inline uint16_t rgb332_to_333h(uint8_t v) {
    uint8_t b2 = v & 3;
    return ((v & 0xE0) << 1) | ((v & 0x1C) << 1) | (b2 << 1) | (b2 ? 1 : 0);
}
IRAM_ATTR bool NEXTVID::nrHot(uint8_t r, uint8_t v) {
    if (r == 0x40) {
        palIdx = v;
        pal9Second = false;
    } else if (r == 0x41) {
        setLutEntry((palCtrl >> 4) & 7, palIdx, rgb332_to_333h(v));
        pal9Second = false;
        if (!(palCtrl & 0x80)) palIdx++;
    } else if (r == 0x44) {
        if (!pal9Second) {
            pal9First = v;
            pal9Second = true;
        } else {
            uint16_t rgb = (rgb332_to_333h(pal9First) & ~1) | (v & 1);
            const uint8_t pal = (palCtrl >> 4) & 7;
            if (v & 0x80) { rgb |= 0x8000; palHasPrio[pal] = true; }
            setLutEntry(pal, palIdx, rgb);
            pal9Second = false;
            if (!(palCtrl & 0x80)) palIdx++;
        }
    } else {
        return false;
    }
    NextReg::reg[r] = v;
#if NEXT_TRACE
    g_nx_palw = g_nx_palw + 1;
#endif
    return true;
}

IRAM_ATTR void NEXTVID::setLutEntry(uint8_t pal, uint8_t idx, uint16_t rgb333) {
    if (s_pdRec && s_pdn < NX_PAL_DELTAS) {
        const int x = s_copXBase + (s_copT >> 2);
        if (x < (int)VIDEO::vga.xres)
            s_pd[s_pdn++] = { (uint16_t)(x < 0 ? 0 : x), pal, idx, cRaw[pal][idx], rgb333 };
    }
    cRaw[pal][idx] = rgb333;
    if (pal_w - pal_app >= PAL_LOG) { c1Drain(1); palApply(pal_w); }   // log full
    palLog[pal_w % PAL_LOG] = ((uint32_t)(pal & 7) << 24) | ((uint32_t)idx << 16) | rgb333;
    __dmb();
    pal_w = pal_w + 1;
}

void NEXTVID::rebuildLut(uint8_t pal) {
    c1Drain(2);
    palApply(pal_w);
    for (int i = 0; i < 256; i++)
        lut[pal][i] = remap[rawPal[pal][i] & 0x1FF];
}

static inline uint16_t rgb332_to_333(uint8_t v) {
    uint8_t b2 = v & 3;
    return ((v & 0xE0) << 1) | ((v & 0x1C) << 1) | (b2 << 1) | (b2 ? 1 : 0);
}

void NEXTVID::defaultPalettes() {
    c1Drain(2);
    palApply(pal_w);
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
    memcpy(cRaw, rawPal, 8 * 256 * 2);
}

static inline uint8_t writePalSel_(uint8_t ctrl) { return (ctrl >> 4) & 7; }

int NEXTVID::nrRead(uint8_t r) {
    switch (r) {
        case 0x40: return palIdx;                                   // auto-incremented index
        case 0x61: return copIndex & 0xFF;                          // copper write address 7:0
        case 0x62: return (copCtrl & 0xC0) | ((copIndex >> 8) & 7); // mode & 000 & address 10:8
        default:   return -1;
    }
}

uint8_t NEXTVID::palRead8() {
    // nextreg 0x41 read: bits 8:1 of the current entry (RRRGGGBB)
    if (!cRaw) return 0;
    return (uint8_t)(cRaw[writePalSel_(palCtrl)][palIdx] >> 1);
}

uint8_t NEXTVID::palRead9() {
    // nextreg 0x44 read: second-byte view — bit7 = L2 priority, bit0 = B lsb
    if (!cRaw) return 0;
    uint16_t raw = cRaw[writePalSel_(palCtrl)][palIdx];
    return (uint8_t)(((raw & 0x8000) ? 0x80 : 0) | (raw & 1));
}

void NEXTVID::ulapWrite(uint8_t idx, uint8_t v) {
    // the nextreg 0xFF stream: GGGRRRBB -> RRRGGGBB, blue lsb = B1 | B0
    const uint8_t pal = (palCtrl & 0x40) ? 4 : 0;
    const uint8_t rgb = (uint8_t)(((v & 0x1C) << 3) | ((v & 0xE0) >> 3) | (v & 0x03));
    setLutEntry(pal, (uint8_t)(0xC0 | (idx & 0x3F)), rgb332_to_333(rgb));
}

uint8_t NEXTVID::ulapRead(uint8_t idx) {
    if (!cRaw) return 0;
    const uint16_t e = cRaw[(palCtrl & 0x40) ? 4 : 0][0xC0 | (idx & 0x3F)];
    // port_ff3b_dat = dat(5:3) & dat(8:6) & dat(2:1)  (GGG RRR BB)
    return (uint8_t)((((e >> 3) & 7) << 5) | (((e >> 6) & 7) << 2) | ((e >> 1) & 3));
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
    sprPut(sprAttrSlot, sprAttrByte, v);
    // Auto-advance: 4-byte sprites move on after byte 3 unless byte 3
    // requests the 5th attribute byte
    if (sprAttrByte == 3 && !(v & 0x40)) {
        a[4] = 0;
        sprPut(sprAttrSlot, 4, 0);
        sprAttrByte = 0;
        sprAttrSlot = (sprAttrSlot + 1) & 0x7F;
    } else if (sprAttrByte == 4) {
        sprAttrByte = 0;
        sprAttrSlot = (sprAttrSlot + 1) & 0x7F;
    } else {
        sprAttrByte++;
    }
}

// Logged like the attributes (sprLog, bit 31 set): sprPatterns is the
// renderer's, the guest never reads it back. A live write let lines already
// past show patterns uploaded after them — TX-1696's bottom sprites blinked at
// the level start; a drain per byte instead cost 2-3 ms a frame, the game
// animates its sprites by re-uploading patterns (hw 2026-10-09).
void NEXTVID::spritePatternWrite(uint8_t v) {
    sprLogPut(0x80000000u | ((uint32_t)(sprPatWrite & 0x3FFF) << 8) | v);
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

uint8_t NEXTVID::spriteNrIndex() {
    return (NextReg::reg[0x09] & 0x10) ? sprAttrSlot : sprNrSlot;
}

void NEXTVID::spriteAttrDirect(uint8_t byteIdx, uint8_t v, bool inc) {
    const bool lock = NextReg::reg[0x09] & 0x10;
    uint8_t& slot = lock ? sprAttrSlot : sprNrSlot;
    if (byteIdx < 5) { sprAttr[slot][byteIdx] = v; sprPut(slot, byteIdx, v); }
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
    if (mode != prevMode && (mode & 1)) copPC = 0;
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

// The copper (copper.vhd), one 28 MHz clock per step: a WAIT that is due
// takes a clock, a MOVE two (the write, then the strobe clear), a NOP one.
// Instructions are big-endian:
//   MOVE: 0RRRRRRR VVVVVVVV  — write V to nextreg R; register 0 = NOP
//   WAIT: 1HHHHHHL LLLLLLLL  — wait until cvc == L and hc_ula >= H*8 + 12
// hc_ula (zxula_timing.vhd 422-438, 7 MHz) is 0 twelve pixels before the
// paper's left edge, so WAIT H lands on paper pixel H*8; cvc steps (and NR
// 0x64 offsets it) at hc_ula 0, which is why the left border of a display
// row still belongs to the previous copper line (MrKWatkins !Copper, test 2).
// PC stays ON a pending WAIT, so the next span re-tests it.
IRAM_ATTR void NEXTVID::copperRun(uint16_t cvc, uint16_t from, uint16_t to) {
    if ((copCtrl >> 6) == 0) return;
    const int32_t end = (int32_t)to << 2;
    if (s_copT < ((int32_t)from << 2)) s_copT = (int32_t)from << 2;
    int guard = 2048;
    while (guard-- && s_copT < end) {
        const uint8_t hi = copperMem[copPC * 2];
        const uint8_t lo = copperMem[copPC * 2 + 1];
        if (hi & 0x80) {
            const uint16_t wl = ((uint16_t)(hi & 0x01) << 8) | lo;
            const int32_t tgt = (int32_t)(((hi >> 1) & 0x3F) * 8 + 12) << 2;
            if (wl != cvc || tgt >= end) return;   // not due in this span
            if (tgt > s_copT) s_copT = tgt;
            s_copT += 1;
        } else if (hi & 0x7F) {
            NextReg::write(hi & 0x7F, lo);
            s_copT += 2;
        } else {
            s_copT += 1;
        }
        copPC = (copPC + 1) & 0x3FF;
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
        // Two blocks, Next-boot only, never freed (the Next boundary reboots).
        // It used to be ONE 29 KB malloc, which PANICKED at boot once the heap
        // had no hole that size (hw 2026-10-08, pico_malloc panics on NULL).
        // Hot (read per pixel, ~11 KB): rawPal 4K | lut 2K | lineBuf 384 |
        //   pairMap 256 | hiresRow 256 | remap 512 |
        //   layer lines 4 x 384 x 2  -> HOT_SRAM (SRAM when the heap has it).
        // Cold (~19 KB): sprite patterns 16K | copper 2K | sprAttr 640.
        //   Butter PSRAM since 2026-10-11 (SRAM wave 2): 720x576 + NeoGS did
        //   not fit, and this is the one block of the Next video state that is
        //   indexed rather than per-pixel — the sprite row loop below reads 8
        //   or 16 CONTIGUOUS bytes of one pattern per sprite per rendered line
        //   (one XIP line fill, not one per pixel), the copper is core0 and
        //   reads two bytes per instruction, and core0 writing / core1 reading
        //   through the shared coherent XIP cache is exactly what sprLog and
        //   palLog have always done. Costs +19 072 B of SRAM heap on a boot
        //   that claims them; read the [NXV] state: line's addresses (0x11...
        //   = PSRAM, 0x20... = SRAM) and the PERF `c1=` on a sprite title.
        //   Sprite renderer copy + attribute log: ~2.1 KB + 4 KB, their own block.
        static_assert(sizeof(SprRes) == 10, "SprRender packs 128 x 10");
        const unsigned hotSz  = 8*256*2 + 8*256 + 384 + 512 + 512 + 4 * 768;
        const unsigned coldSz = 16384 + 2048 + 128*5;
        uint8_t* hot  = (uint8_t*)Buffer::palloc(hotSz,  Buffer::NEED_POINTER | Buffer::HOT_SRAM);
        uint8_t* cold = (uint8_t*)Buffer::palloc(coldSz, Buffer::NEED_POINTER | Buffer::PREFER_PSRAM);
        const unsigned sprSz = (sizeof(SprRender) + 3) & ~3u;
        // Two blocks, not one: the renderer state is read per sprite per line
        // (core1), the log only for the bytes written since the last line. As
        // one 6 KB block it missed HOT_SRAM's rule on a thin heap and the whole
        // state landed in butter PSRAM (Santa, hw 2026-10-09: spr 3.9 ms/frame).
#if NEXT_CODE_OVERLAY
        // In the .nxovl window (SRAM on a Next boot, heap on any other): even
        // 2 KB missed HOT_SRAM's rule there (Santa, hw 2026-10-09).
        static SprRender s_sprR __attribute__((section(".time_critical.nxspr"), aligned(4)));
        uint8_t* sb = (uint8_t*)&s_sprR;
#else
        uint8_t* sb = (uint8_t*)Buffer::palloc(sprSz, Buffer::NEED_POINTER | Buffer::HOT_SRAM);
#endif
        // The logs go to butter PSRAM: written by core0, read by core1 a few
        // entries a line, and both cores see one XIP cache (coherent). 6 KB of
        // SRAM heap back for the .nxovl window (2026-10-10).
        uint8_t* sl = (uint8_t*)Buffer::palloc(4 * SPR_LOG, Buffer::NEED_POINTER | Buffer::PREFER_PSRAM);
        Debug::log("[NXV] sprite renderer state %u B @%08X, log %u B @%08X", sprSz,
                   (unsigned)(uintptr_t)sb, 4 * SPR_LOG, (unsigned)(uintptr_t)sl);
        if (!sb || !sl) panic("NEXTVID: no memory for the sprite state");
        sprR = (SprRender*)sb;
        sprLog = (uint32_t*)sl;
        palLog = (uint32_t*)Buffer::palloc(4 * PAL_LOG, Buffer::NEED_POINTER | Buffer::PREFER_PSRAM);
        cRaw = (uint16_t(*)[256])Buffer::palloc(8 * 256 * 2, Buffer::NEED_POINTER | Buffer::PREFER_PSRAM);
        Debug::log("[NXV] palette log @%08X, core0 palette @%08X",
                   (unsigned)(uintptr_t)palLog, (unsigned)(uintptr_t)cRaw);
        if (!palLog || !cRaw) panic("NEXTVID: no memory for the palette log");
        Debug::log("[NXV] state: hot %u B @%08X, cold %u B @%08X",
                   hotSz, (unsigned)(uintptr_t)hot, coldSz, (unsigned)(uintptr_t)cold);
        if (!hot || !cold) panic("NEXTVID: no memory for the Next video state");
        memset(hot, 0, hotSz);
        memset(cold, 0, coldSz);
        sprPatterns = cold;
        copperMem   = cold + 16384;
        sprAttr     = (uint8_t(*)[5])(cold + 16384 + 2048);
        rawPal      = (uint16_t(*)[256])hot;
        lut         = (uint8_t(*)[256])(hot + 8*256*2);
        lineBuf     = hot + 8*256*2 + 8*256;
        pairMap     = lineBuf + 384;
        hiresRow    = pairMap + 256;
        remap       = hiresRow + 256;
        nxU = (uint16_t*)(remap + 512);
        nxT = nxU + 384; nxL = nxT + 384; nxS = nxL + 384;
    }
#if NEXT_TRACE
    extern volatile uint16_t* g_nx_pcs;
    extern volatile uint16_t* g_nx_pcpg;
    extern volatile uint32_t (*g_nx_dmav)[3];
    if (!g_nx_pcs) {
        uint16_t* pb = (uint16_t*)tryCalloc((2048 + 256) * 2 + 8 * 3 * 4);
        if (pb) {
            g_nx_pcs = pb; g_nx_pcpg = pb + 2048;
            g_nx_dmav = (volatile uint32_t (*)[3])(pb + 2048 + 256);
        }
    }
#endif
    pal_w = pal_app = 0;
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
    memset(sprR, 0, sizeof(SprRender));      // queue drained above
    sprR->dirty = 1; sprR->lo = 0; sprR->hi = 127;
    spr_w = spr_app = 0;
    sprNrSlot = 0;
    sprPatWrite = 0;
    sprAttrSlot = 0;
    sprAttrByte = 0;
    sprFlags = 0;

    copIndex = 0;
    copPC = 0;
    copCtrl = 0;
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

// NextIdle's fast-forward of a memory-poll loop: the guest clock to stEnd with
// the lines rendered on the way, like haltAdvanceTo but without its R count
// (the caller adds the loop's own fetches). Not in contended memory, where
// every pass has its own length: false, nothing done. Flash: a few calls a frame.
__attribute__((noinline)) bool NEXTVID::idleAdvanceTo(uint32_t stEnd) {
    if (contend) return false;
    while (CPU::tstates < stEnd) {
        const uint32_t t = nextLineTstate < stEnd ? nextLineTstate : stEnd;
        if (t > CPU::tstates) CPU::tstates = t;
        while (CPU::tstates >= nextLineTstate) ScanlineWork();
    }
    return true;
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
        const uint16_t* pal = cRaw[(palCtrl & 0x02) ? 4 : 0];
        for (int i = 0; i < 16; i++)
            for (int j = 0; j < 16; j++) {
                const uint16_t a = pal[i] & 0x1FF, b = pal[j] & 0x1FF;
                const unsigned r = (((a >> 6) & 7) + ((b >> 6) & 7) + 1) >> 1;
                const unsigned g = (((a >> 3) & 7) + ((b >> 3) & 7) + 1) >> 1;
                const unsigned bl = ((a & 7) + (b & 7) + 1) >> 1;
                gigsBlend[(i << 4) | j] = (i == j) ? remap[a]
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
volatile uint32_t g_nextvid_layer_us[7] = {0, 0, 0, 0, 0, 0, 0};   // [6] = log apply (spr/pal) before the layers
volatile uint32_t g_nx_logapply_spr = 0, g_nx_logapply_pal = 0;     // entries applied (all rows)   // + mix, out (RenderLine copy)
volatile uint32_t g_nx_rows = 0;            // rows composed (all, not only sampled)
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
#if NEXT_TRACE
// Profile (NEXT pc: line): Z80 PC sampled once per raster line, 32-byte buckets,
// plus the MMU page of the PC's slot.
// Heap (taken at the first NEXTVID::Reset): 4.5 KB of .bss here OOM-panicked the
// boot in Config::load (hw 2026-10-09).
volatile uint16_t* g_nx_pcs = nullptr;    // [2048]
volatile uint16_t* g_nx_pcpg = nullptr;   // [256]
#endif
__attribute__((noinline)) IRAM_ATTR void NEXTVID::ScanlineWork() {
#if NEXT_TRACE
    if (g_nx_pcs) { const uint16_t pc = Z80::getRegPC();
      g_nx_pcs[pc >> 5] = g_nx_pcs[pc >> 5] + 1;
      const uint8_t pg = MemESP::mmu[pc >> 13];
      g_nx_pcpg[pg] = g_nx_pcpg[pg] + 1; }
#endif
    uint8_t m = ESPectrum::multiplicator;
    if (lineIdx < totalLines) {
        // Copper, in time order across the display row (copper.vhd /
        // zxula_timing.vhd): the row's left border is the tail of the PREVIOUS
        // copper line (hc_ula >= uL), its paper and right border the head of
        // this one. cvc = raster line - paper top + NR 0x64. Mode 11 restarts
        // at hc_ula 0 of copper line 0. Registers are captured mid-paper (one
        // snapshot per row); palette writes are recorded with their fb x and
        // the row is then drawn in segments (renderRowSegments).
        const uint32_t paperStart = (uint32_t)(firstVisibleLine + paperTopRow);
        const uint32_t off = NextReg::reg[0x64];
        const uint16_t cur = (uint16_t)((lineIdx + totalLines - paperStart + off) % totalLines);
        const uint16_t prev = (uint16_t)((lineIdx + 2 * totalLines - 1 - paperStart + off) % totalLines);
        const uint16_t hcEnd = (uint16_t)(VIDEO::tStatesPerLine * 2);
        const int uL = (int)hcEnd - (xPaperOff - 12);       // hc_ula of fb x 0
        int row = (int)lineIdx - firstVisibleLine;
        const bool draw = !skipFrame && row >= 0 && row < (int)VIDEO::vga.yres;
        if ((copCtrl >> 6) != 0) {
            s_pdn = 0;
            s_pdRec = draw;
            s_copXBase = (int16_t)(xPaperOff - 12 - (int)hcEnd);
            copperRun(prev, (uint16_t)uL, hcEnd);
            s_copT -= (int32_t)hcEnd << 2;                  // hc_ula wraps, cvc steps
            if (s_copT < 0) s_copT = 0;
            s_copXBase = (int16_t)(xPaperOff - 12);
            if ((copCtrl >> 6) == 3 && cur == 0) copPC = 0;
            copperRun(cur, 0, 140);                         // up to paper x 128
            NextLineState st;
            if (draw) {
                captureLine(st);
                if (s_brdCnt) nxBorderSegs(st, row, firstVisibleLine);
                if (Z80DMA::dma_attr_valid) nxAttrShadow(st, row, paperTopRow);
            }
            copperRun(cur, 140, (uint16_t)uL);
            s_pdRec = false;
            if (draw) {
                if (s_pdn) {
                    nx_sync = st;
                    renderRowSegments(row);
                } else {
                    s_preCap = &st;
                    emitRow(row);
                    s_preCap = nullptr;
                }
            }
        } else if (draw) {
            emitRow(row);
        }
        lineIdx++;
        nextLineTstate = (lineIdx + 1) * ((uint32_t)VIDEO::tStatesPerLine << m);
    } else {
        // frame over-run (turbo tail) — wait for EndFrame to restart
        nextLineTstate = 0xFFFFFFFF;
    }
}

// One fb row: posted to core1 with a register snapshot, or drawn here.
// noinline: ScanlineWork and EndFrame's tail fill both call it.
#if NEXT_TRACE
volatile uint32_t g_nx_emit_us = 0;          // core0: emitRow (capture + post) per second
volatile uint32_t g_nx_rowprobe[2][8];
#endif
__attribute__((noinline)) IRAM_ATTR void NEXTVID::emitRow(int row) {
#if NEXT_TRACE
    const uint64_t _em0 = time_us_64();
    emitRowBody(row);
    g_nx_emit_us = g_nx_emit_us + (uint32_t)(time_us_64() - _em0);
}
__attribute__((always_inline)) inline void NEXTVID::emitRowBody(int row) {
#endif
    if (g_next_c1_live) {
        // Post the line: capture, and open a new state slot only when
        // something the renderer reads has moved since the last line.
        NextLineState st;
        if (s_preCap) st = *s_preCap;
        else {
            captureLine(st);
            if (s_brdCnt) nxBorderSegs(st, row, firstVisibleLine);
            if (Z80DMA::dma_attr_valid) nxAttrShadow(st, row, paperTopRow);
        }
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
        const uint32_t occ = nx_w - nx_r;      // ring occupancy (see g_nx_ring_max)
        if (occ > g_nx_ring_max) g_nx_ring_max = occ;
        g_nx_ring_now = occ;
    } else {
        if (s_preCap) nx_sync = *s_preCap;
        else {
            captureLine(nx_sync);
            if (s_brdCnt) nxBorderSegs(nx_sync, row, firstVisibleLine);
            if (Z80DMA::dma_attr_valid) nxAttrShadow(nx_sync, row, paperTopRow);
        }
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
extern "C" volatile uint8_t g_sd_as_state;
extern "C" void sd_async_pump(void);
IRAM_ATTR void NEXTVID::c1Pump() {
    // a background SD read posted by DivMMC (sdcard.c): ~1 ms on the card, run
    // here so core0 keeps emulating meanwhile
    if (__builtin_expect(g_sd_as_state == 1, 0)) sd_async_pump();
#if NEXT_TRACE
    if (nx_r == nx_w) return;
    const uint64_t tb = time_us_64();
#endif
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
#if NEXT_TRACE
    g_nx_c1_us = g_nx_c1_us + (uint32_t)(time_us_64() - tb);
#endif
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
// A row the copper re-coloured mid-line (MrKWatkins !Copper: Swedish flags
// drawn by rewriting PAPER/BORDER 7 every 8 pixels). The palette now holds the
// line-END values; each recorded write knows its fb x and the value it
// replaced. Roll the palette back to the line start, then compose the row once
// per palette state and keep each pass's span. core0, after a drain — core1
// must not read the palette while it is rolled back. Flash: rare rows.
__attribute__((noinline)) void NEXTVID::renderRowSegments(int row) {
    c1Drain(3);
    palApply(pal_w);              // line-end values, as the deltas below expect
    LS = &nx_sync;
    uint8_t acc[400];
    const int xres = VIDEO::vga.xres;
    for (int i = s_pdn - 1; i >= 0; i--) {
        rawPal[s_pd[i].pal][s_pd[i].idx] = s_pd[i].old;
        lut[s_pd[i].pal][s_pd[i].idx] = remap[s_pd[i].old & 0x1FF];
    }
    int x0 = 0, i = 0;
    while (x0 < xres) {
        // apply every write that takes effect at or before x0
        while (i < s_pdn && s_pd[i].x <= x0) {
            rawPal[s_pd[i].pal][s_pd[i].idx] = s_pd[i].neu;
            lut[s_pd[i].pal][s_pd[i].idx] = remap[s_pd[i].neu & 0x1FF];
            i++;
        }
        const int x1 = (i < s_pdn && s_pd[i].x < xres) ? s_pd[i].x : xres;
        RenderLayers(lineBuf, row);
        for (int x = x0; x < x1; x++) acc[x] = lineBuf[x];
        x0 = x1;
    }
    for (; i < s_pdn; i++) {   // writes past the visible line: still the final palette
        rawPal[s_pd[i].pal][s_pd[i].idx] = s_pd[i].neu;
        lut[s_pd[i].pal][s_pd[i].idx] = remap[s_pd[i].neu & 0x1FF];
    }
    for (int x = 0; x < xres; x++) lineBuf[x] = acc[x];
    s_lineReady = true;
    RenderLine(row);
    s_lineReady = false;
}

IRAM_ATTR void NEXTVID::RenderLine(int row) {
    uint32_t* dst = (uint32_t*)VIDEO::vga.frameBuffer[row];
    if (!s_lineReady) RenderLayers(lineBuf, row);
#if NEXT_TRACE
    g_nx_rows = g_nx_rows + 1;
    const uint64_t _out0 = nx_layer_sample ? time_us_64() : 0;
#endif
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
#if NEXT_TRACE
    if (nx_layer_sample) g_nextvid_layer_us[5] += (uint32_t)(time_us_64() - _out0) * 16;
#endif
}

// ---- Layer compositor (zxnext.vhd 6940-7350, zxula.vhd 470-565) ----
// Every layer renders its line into its own buffer of 9-bit RGB (palette
// applied), in framebuffer x; the mixer then picks per pixel by NR 0x15 bits
// 4:2 exactly as the board does, with transparency decided on the COLOUR after
// the palette (NR 0x14) for the ULA / LoRes, Layer 2 and text-mode tiles, on
// the index for sprites (NR 0x4B) and standard tiles (NR 0x4C), and the
// fallback colour (NR 0x4A) where nothing is opaque. Flags above the RGB:
static constexpr uint16_t NX_TR    = 0x8000;   // transparent
static constexpr uint16_t NX_SLOT  = 0x4000;   // low byte already a fb slot (Timex hi-res mark, Gigascreen blend)
static constexpr uint16_t NX_BRD   = 0x2000;   // ULA border pixel (layer orders 3-5)
static constexpr uint16_t NX_BELOW = 0x1000;   // tilemap pixel under the ULA (attr bit 0 / 512-tile mode)
static constexpr uint16_t NX_PRIO  = 0x0800;   // Layer 2 palette priority (bit 15 of the entry)

static inline __attribute__((always_inline)) void nxFillTr(uint16_t* b, int n) {
    uint32_t* w = (uint32_t*)b;
    for (int i = 0, e = (n + 1) >> 1; i < e; i++) w[i] = 0x80008000u;
}
// palette entry -> 9-bit RGB, or transparent when it equals NR 0x14
static inline __attribute__((always_inline)) uint16_t nxRgb(uint16_t raw, uint8_t tr) {
    const uint16_t v = raw & 0x1FF;
    return ((v >> 1) == tr) ? NX_TR : v;
}
static void nxLoResStd(uint16_t* p, const uint8_t* base, uint8_t sxl, uint8_t ofs,
                       const uint16_t* raw, uint8_t tr);

// Mid-line border changes: coloured runs, then the closing colour. Paper rows
// get the runs under the paper too; the ULA layer overwrites those 256 pixels.
// Flash: only rows that carry a change come here.
__attribute__((noinline)) static void nxPaintBorderRuns(uint16_t* u, int xres, const uint16_t* bc,
                                                        uint16_t borderV) {
    int x = 0;
    for (int i = 0; i < LS->brd.n; i++) {
        int xe = 2 * LS->brd.t[i];
        if (xe > xres) xe = xres;
        if (xe > x) {
            const uint16_t c = bc[(LS->brd.c[i >> 1] >> ((i & 1) * 4)) & 7];
            for (; x < xe; x++) u[x] = c;
        }
    }
    for (; x < xres; x++) u[x] = borderV;
}

// The ULA layer (zxula.vhd): border, then the paper — classic / ULANext /
// LoRes / Timex hi-res / Gigascreen. NR 0x68 bit 7 (ULA off) is applied by the
// mixer, not here: blend mode 00 still sees the ULA colour.
// Rotate the 256-pixel paper left by n (three reversals, no buffer: core1's
// stack is 2 KB). Only on a scrolled line.
static IRAM_ATTR void nxRev(uint16_t* a, int i, int j) {
    for (j--; i < j; i++, j--) { const uint16_t t = a[i]; a[i] = a[j]; a[j] = t; }
}
static IRAM_ATTR void nxRotL(uint16_t* a, int n) {
    nxRev(a, 0, n); nxRev(a, n, 256); nxRev(a, 0, 256);
}

// Nearest of the 16 ZX colours (the only ones a packed-pair hi-res frame can
// show) to a 9-bit RRRGGGBBB palette colour. Twice a hi-res line.
__attribute__((noinline)) static uint8_t nxZxNearest(uint16_t v) {
    static const uint8_t zxlvl[2] = { 205, 255 };
    const int tr = ((v >> 6) & 7) * 255 / 7, tg = ((v >> 3) & 7) * 255 / 7, tb = (v & 7) * 255 / 7;
    uint32_t bestd = 0xFFFFFFFFu; uint8_t best = 0;
    for (int c = 0; c < 16; c++) {
        const int l = zxlvl[(c >> 3) & 1];
        const int dr = tr - ((c & 2) ? l : 0), dg = tg - ((c & 4) ? l : 0), db = tb - ((c & 1) ? l : 0);
        const uint32_t d = (uint32_t)(dr * dr * 3 + dg * dg * 6 + db * db);
        if (d < bestd) { bestd = d; best = (uint8_t)c; }
    }
    return best;
}

// Returns false when the whole row (border included) is transparent; nxU is
// then just filled transparent and the mixer may leave the ULA out.
IRAM_ATTR bool NEXTVID::RenderUlaLine(int row) {
    const int xres = VIDEO::vga.xres;
    uint16_t* const u = nxU;
    const uint16_t* raw = rawPal[(LS->palCtrl & 0x02) ? 4 : 0];
    const uint8_t tr = LS->r14;
    const bool ulaNext = LS->palCtrl & 0x01;
    const uint8_t mask = LS->r42;
    const uint16_t fallback = rgb332_to_333(LS->r4A);
    // border colours 0..7: ULANext paper base 128 (format 0xFF selects the
    // fallback colour, ula_select_bgnd), classic ULA the paper entries 16..23
    // ULA+ (NR 0x68 bit 3 = port FF3B mode 01, zxula.vhd 531-540): index
    // 11 & attr 7:6 & paper & colour — the border is attr 00, paper: 192+8+c.
    const bool ulap = !ulaNext && (LS->r68 & 0x08);
    uint16_t bc[8];
    for (int c = 0; c < 8; c++)
        bc[c] = (uint16_t)(ulaNext ? (mask == 0xFF ? fallback : nxRgb(raw[128 + c], tr))
                                   : nxRgb(raw[(ulap ? 200 : 16) + c], tr));
    for (int c = 0; c < 8; c++) if (!(bc[c] & NX_TR)) bc[c] |= NX_BRD;
    // Timex hi-res (port FF mode 1xx, zxula.vhd border_clr_tmx): the ULA's
    // attribute is fixed at 01 & ~ink & ink — BRIGHT, paper = the inverse of
    // the ink in port FF 5:3 — and the border takes that paper colour too.
    // Ink / paper go through whatever palette the ULA uses (classic, ULANext,
    // ULA+) like any attribute.
    const bool hr = LS->hires && !(LS->r69 & 0x40);
    uint16_t hrInk = 0, hrPap = 0;
    if (hr) {
        const uint8_t ink = (LS->r69 >> 3) & 7;
        const uint8_t a = (uint8_t)(0x40 | ((~ink & 7) << 3) | ink);
        if (ulaNext) {
            uint8_t shift = 0;
            for (uint8_t m = mask; m; m >>= 1) shift++;
            const bool bgnd = (mask & (uint8_t)(mask + 1)) != 0 || mask == 0xFF || mask == 0;
            hrInk = raw[a & mask] & 0x1FF;
            hrPap = bgnd ? (fallback & 0x1FF) : (raw[128 + (a >> shift)] & 0x1FF);
        } else if (ulap) {
            hrInk = raw[0xD0 | ink] & 0x1FF;
            hrPap = raw[0xD8 | ((a >> 3) & 7)] & 0x1FF;
        } else {
            hrInk = raw[8 | ink] & 0x1FF;
            hrPap = raw[16 + (8 | ((a >> 3) & 7))] & 0x1FF;
        }
    }
    const uint16_t borderV = hr ? (uint16_t)(hrPap | NX_BRD) : bc[LS->border & 7];
    const int y = row - paperTopRow;

    // A row that is transparent end to end (NXModPlayer: black screen, black
    // border, NR 0x14 = 0 under its text tilemap and Layer 2 — 3.3 ms of core1
    // a frame for nothing, hw 2026-10-11): classic ULA only, the check is the
    // attribute pass without the 256 stores.
    bool brdTr = (borderV & NX_TR) != 0;
    if (LS->brd.n && !hr) for (int c = 0; c < 8; c++) brdTr = brdTr && (bc[c] & NX_TR);
    if (brdTr && !hr && !ulaNext && !ulap && !(LS->r15 & 0x80) && !(LS->gigs && gigsPrev)) {
        bool op = false;
        if (y >= 0 && y < 192 && !VIDEO::paper_off) {
            const int sy = LS->r27 ? (y + LS->r27) % 192 : y;
            const uint8_t tm = (LS->r69 & 0x40) ? 0 : (LS->r69 & 0x03);
            const uint16_t pOff = VIDEO::offBmp[sy];
            const uint8_t* bmp = LS->grmem + pOff + ((tm & 1) << 13);
            const uint8_t* att = (LS->attOvr && sy == y) ? LS->attr
                               : LS->grmem + ((tm & 2) ? 0x2000 + pOff
                                                       : VIDEO::offAtt[sy] + ((tm & 1) << 13));
            for (int col = 0; col < 32 && !op; col++) {
                const uint8_t a = att[col];
                const uint8_t b = bmp[col] ^ (uint8_t)(-((a & LS->flashing) >> 7));
                const uint8_t bright = (a & 0x40) >> 3;
                if (b && !(nxRgb(raw[(a & 7) | bright], tr) & NX_TR)) op = true;
                else if (b != 0xFF && !(nxRgb(raw[16 + (((a >> 3) & 7) | bright)], tr) & NX_TR)) op = true;
            }
        }
        if (!op) { nxFillTr(u, xres); return false; }
    }

    if (LS->brd.n && !hr) nxPaintBorderRuns(u, xres, bc, borderV);
    else for (int x = 0; x < xres; x++) u[x] = borderV;

    if (y < 0 || y >= 192 || VIDEO::paper_off) return true;
    uint16_t* p = u + xPaperOff;
    // ULA scroll (NR 0x26/0x27, zxula.vhd 192-206): the memory row is
    // (y + scroll Y) mod 192, the columns start scroll X pixels in and wrap
    // at 256 (rotated below, after the row is drawn). NR 0x68 bit 2 (fine
    // scroll, one more 14 MHz half-pixel) has no lores equivalent: ignored.
    // LoRes keeps its own scroll registers (NR 0x32/0x33).
    const int sy = LS->r27 ? (y + LS->r27) % 192 : y;

    if (LS->r15 & 0x80) {          // LoRes 128x96 replaces the paper (not ULA-clipped)
        // lores.vhd: rows wrap at 192, columns at 256 (scroll NR 0x32/0x33).
        // Standard: one byte a pixel, rows of 128 in two halves (0x0000 /
        // 0x2000), index = (high nibble + NR 0x6A offset) & low nibble.
        // Radastan (NR 0x6A bit 5): 4bpp, 64 bytes a row, display file =
        // port FF bit 0 XOR NR 0x6A bit 4 (+0x2000), index = offset (or 11 &
        // offset 1:0 with ULA+, NR 0x68 bit 3) & the pixel nibble.
        const uint8_t ly = (uint8_t)((y + LS->r33) % 192);
        const uint8_t sxl = LS->r32, ofs = LS->r6A & 0x0F;
        if (LS->r6A & 0x20) {
            const uint8_t hiN = (uint8_t)(((LS->r68 & 0x08) ? (0x0C | (ofs & 3)) : ofs) << 4);
            const bool df = ((LS->r69 ^ (LS->r6A >> 4)) & 1) != 0;
            const uint8_t* base = LS->grmem + (df ? 0x2000 : 0) + (ly >> 1) * 64;
            for (int x = 0; x < 256; x++) {
                const uint8_t xx = (uint8_t)(x + sxl);
                const uint8_t b = base[xx >> 2];
                p[x] = nxRgb(raw[hiN | ((xx & 2) ? (b & 0x0F) : (b >> 4))], tr);
            }
        } else {
            const uint8_t ry = ly >> 1;
            const uint8_t* base = LS->grmem +
                (ry < 48 ? ry * 128 : 0x2000 + (uint16_t)(ry - 48) * 128);
            nxLoResStd(p, base, sxl, ofs, raw, tr);
        }
        return true;
    }
    // Timex screen mode = port FF / NR 0x69 bits 2:0, forced to 000 while the
    // shadow screen (bank 7, NR 0x69 bit 6 = port 7FFD bit 3) is displayed —
    // zxula.vhd: "limit timex modes to bank 5 as bank 7 only has 8k bram".
    if (hr) {
        // Timex hi-res 512x192 (zxula.vhd vram_a, shift_reg_32): each 16-pixel
        // column is the "pixel" byte (left 8) then the "attribute" byte (right
        // 8), fetched per mode bits 1:0 — mode 6: 0x4000 / 0x6000 pixels (the
        // usual hi-res), 7: 0x6000 twice, 4: 0x4000 pixels / 0x5800 attributes,
        // 5: 0x6000 pixels / 0x7800 attributes. hiresRow gets pair slots of
        // the nearest ZX colours of ink / paper; the paper carries HIRES_MARK
        // as a ready fb slot so layers drawn over the ULA win.
        const uint8_t m = LS->r69 & 7;
        const uint8_t ci = nxZxNearest(hrInk), cp = nxZxNearest(hrPap);
        uint16_t hl[16];
        for (int n = 0; n < 16; n++) {
            const uint8_t c3 = (n & 8) ? ci : cp, c2 = (n & 4) ? ci : cp;
            const uint8_t c1 = (n & 2) ? ci : cp, c0 = (n & 1) ? ci : cp;
            hl[n] = (uint16_t)(VIDEO::profi_pair_lookup[c3][c2]
                  | ((uint16_t)VIDEO::profi_pair_lookup[c1][c0] << 8));
        }
        const uint8_t* lb = LS->grmem + ((m & 1) << 13) + VIDEO::offBmp[sy];
        const uint8_t* rb = (m & 2) ? LS->grmem + 0x2000 + VIDEO::offBmp[sy]
                                    : LS->grmem + ((m & 1) << 13) + VIDEO::offAtt[sy];
        uint32_t* hrw = (uint32_t*)hiresRow;
        for (int col = 0; col < 32; col++) {
            const uint8_t e = lb[col], o = rb[col];
            hrw[2 * col]     = (uint32_t)hl[e >> 4] | ((uint32_t)hl[e & 0x0F] << 16);
            hrw[2 * col + 1] = (uint32_t)hl[o >> 4] | ((uint32_t)hl[o & 0x0F] << 16);
        }
        for (int x = 0; x < 256; x++) p[x] = NX_SLOT | HIRES_MARK;
    } else {
        // zxula.vhd vram_a: pixel = mode(0) & pixel address, attribute =
        // '1' & PIXEL address when mode(1) (hi-colour 8x1 attributes at
        // 0x6000), else mode(0) & attribute address.
        const uint8_t tm = (LS->r69 & 0x40) ? 0 : (LS->r69 & 0x03);
        const uint16_t pOff = VIDEO::offBmp[sy];
        const uint8_t* bmp = LS->grmem + pOff + ((tm & 1) << 13);
        const uint8_t* att = (LS->attOvr && sy == y) ? LS->attr
                           : LS->grmem + ((tm & 2) ? 0x2000 + pOff
                                                   : VIDEO::offAtt[sy] + ((tm & 1) << 13));
        if (ulaNext) {
            // ink = attr & format; paper = 128 + the bits above the format;
            // a format that is not 2^n-1 (or 0xFF) gives the fallback paper
            uint8_t shift = 0;
            for (uint8_t m = mask; m; m >>= 1) shift++;
            const bool bgnd = (mask & (uint8_t)(mask + 1)) != 0 || mask == 0xFF || mask == 0;
            for (int col = 0; col < 32; col++) {
                const uint8_t a = att[col], b = bmp[col];
                const uint16_t ink = nxRgb(raw[a & mask], tr);
                const uint16_t pap = bgnd ? fallback : nxRgb(raw[128 + (a >> shift)], tr);
                for (int bit = 0; bit < 8; bit++) p[bit] = (b & (0x80 >> bit)) ? ink : pap;
                p += 8;
            }
        } else if (ulap) {
            // no FLASH, no BRIGHT: attr 7:6 picks one of four 16-entry groups
            for (int col = 0; col < 32; col++) {
                const uint8_t a = att[col], b = bmp[col];
                const uint8_t grp = (uint8_t)(0xC0 | ((a >> 6) << 4));
                const uint16_t ink = nxRgb(raw[grp | (a & 7)], tr);
                const uint16_t pap = nxRgb(raw[grp | 8 | ((a >> 3) & 7)], tr);
                for (int bit = 0; bit < 8; bit++) p[bit] = (b & (0x80 >> bit)) ? ink : pap;
                p += 8;
            }
        } else if (LS->gigs && gigsPrev) {
            // Gigascreen: blended only when the classic ULA is all there is on
            // the line (the row is still recorded so the next frame has a prev)
            const bool blend = LS->gigs == 1 && !(LS->r15 & 0x01) && !(LS->r6B & 0x80) &&
                               !((LS->p123B & 0x02) || (LS->r69 & 0x80)) && !(LS->r68 & 0x80);
            uint32_t* gp = (uint32_t*)(gigsPrev + sy * 128);
            for (int col = 0; col < 32; col++) {
                const uint8_t a = att[col];
                const uint8_t b = bmp[col] ^ (uint8_t)(-((a & LS->flashing) >> 7));
                const uint8_t bright = (a & 0x40) >> 3;
                const uint8_t ci = (a & 7) | bright, cp = ((a >> 3) & 7) | bright;
                const uint16_t ink = nxRgb(raw[ci], tr), pap = nxRgb(raw[16 + cp], tr);
                const uint32_t prev = gp[col];
                uint32_t cur = 0;
                for (int bit = 0; bit < 8; bit++) {
                    const bool on = b & (0x80 >> bit);
                    const uint8_t c = on ? ci : cp;
                    cur |= (uint32_t)c << (bit * 4);
                    p[bit] = blend ? (uint16_t)(NX_SLOT | gigsBlend[((prev >> (bit * 4)) & 15) << 4 | c])
                                   : (on ? ink : pap);
                }
                gp[col] = cur;
                p += 8;
            }
        } else {
            for (int col = 0; col < 32; col++) {
                const uint8_t a = att[col];
                const uint8_t b = bmp[col] ^ (uint8_t)(-((a & LS->flashing) >> 7));
                const uint8_t bright = (a & 0x40) >> 3;
                const uint16_t ink = nxRgb(raw[(a & 7) | bright], tr);
                const uint16_t pap = nxRgb(raw[16 + (((a >> 3) & 7) | bright)], tr);
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
    }
    if (LS->r26 && !LS->hires) nxRotL(u + xPaperOff, LS->r26);
    // ULA clip window (NR 0x1A, zxula.vhd 562): paper only, never the border
    const uint8_t* cw = LS->clip[2];
    if (y < cw[2] || y > cw[3]) { nxFillTr(u + xPaperOff, 256); return true; }
    for (int x = 0; x < cw[0]; x++) u[xPaperOff + x] = NX_TR;
    for (int x = cw[1] + 1; x < 256; x++) u[xPaperOff + x] = NX_TR;
    return true;
}

// LoRes standard mode, one paper row: output pixel x shows byte
// base[(uint8_t)(x + sxl) >> 1]. Each byte covers two output pixels — its
// colour is computed once (TX-1696: 3 ms of core1 a frame at -Os, hw 2026-10-09).
static IRAM_ATTR __attribute__((noinline, optimize("O2", "no-unroll-loops", "no-tree-loop-distribute-patterns")))
void nxLoResStd(uint16_t* p, const uint8_t* base, uint8_t sxl, uint8_t ofs,
                const uint16_t* raw, uint8_t tr) {
    const uint8_t o4 = (uint8_t)(ofs << 4);
    int x = 0;
    if (sxl & 1) {                       // odd scroll: the first byte shows one pixel
        const uint8_t b = base[sxl >> 1];
        p[0] = nxRgb(raw[(uint8_t)(((b + o4) & 0xF0) | (b & 0x0F))], tr);
        x = 1;
    }
    for (; x + 1 < 256; x += 2) {
        const uint8_t b = base[(uint8_t)(x + sxl) >> 1];
        const uint16_t v = nxRgb(raw[(uint8_t)(((b + o4) & 0xF0) | (b & 0x0F))], tr);
        p[x] = v; p[x + 1] = v;
    }
    if (x < 256) {                       // odd scroll: the last pixel
        const uint8_t b = base[(uint8_t)(x + sxl) >> 1];
        p[x] = nxRgb(raw[(uint8_t)(((b + o4) & 0xF0) | (b & 0x0F))], tr);
    }
}

// Mixer state for one line (constant across the row)
struct NxMix { uint8_t* fb; const uint8_t* remap; uint16_t fb9; bool ulaEn, stencil, tm, l2, spr; uint8_t blend; };

static inline __attribute__((always_inline)) uint8_t nxOut(const NxMix& m, uint16_t v) {
    return (v & NX_SLOT) ? (uint8_t)v : m.remap[v & 0x1FF];
}
// ula_final (zxnext.vhd 7112-7136): ULA and tilemap merged, or the stencil
static inline __attribute__((always_inline)) uint16_t nxUlaFinal(const NxMix& m, uint16_t u, uint16_t t) {
    const bool uTr = (u & NX_TR) || !m.ulaEn, tTr = t & NX_TR;
    if (m.stencil) return (uTr || tTr) ? NX_TR : (uint16_t)(u & t & 0x1FF);
    if (!tTr && (!(t & NX_BELOW) || uTr)) return (uint16_t)(t & 0x1FF);
    return uTr ? NX_TR : u;
}

// ula_final in place, for a line with BOTH the ULA and the tilemap on: nxU[x]
// becomes the merged ULA/tilemap pixel. NX_BRD survives only where orders 3-5
// look at it (a ULA border pixel with the tilemap transparent there), so the
// mixer below reads ONE buffer for the pair. With only one of the two on, the
// mixer reads that layer's buffer directly and this pass does not run.
static IRAM_ATTR __attribute__((noinline, optimize("O2", "no-unroll-loops", "no-tree-loop-distribute-patterns")))
void nxUlaMerge(bool stencil, int xres) {
    uint16_t* __restrict u = nxU;
    const uint16_t* __restrict t = nxT;
    for (int x = 0; x < xres; x++) {
        const uint16_t uv = u[x], tv = t[x];
        const bool uTr = uv & NX_TR, tTr = tv & NX_TR;
        uint16_t r;
        if (stencil) r = (uTr || tTr) ? NX_TR : (uint16_t)(uv & tv & 0x1FF);
        else if (!tTr && (!(tv & NX_BELOW) || uTr)) r = (uint16_t)(tv & 0x1FF);
        else r = uTr ? NX_TR : (tTr ? uv : (uint16_t)(uv & ~NX_BRD));
        u[x] = r;
    }
}

// Layer orders 0..5 (NR 0x15 bits 4:2): S L U / L S U / S U L / L U S / U S L / U L S.
// One loop per (order, Layer 2 on/off); the order is decided once per line,
// not per pixel (the per-pixel switch at -Os was 15 of Warhawk's 24 ms of core1
// per frame, hw 2026-10-08). pu = ula_final for the line (nxUlaMerge's result,
// the ULA alone, or the tilemap alone). __restrict + locals: with the old
// NxMix& argument every fb byte store forced a reload of the per-line flags
// (Santa: mix = 8.9 of 24 ms of core1, hw 2026-10-09).
// MERGE: ULA and tilemap both on (no stencil) — ula_final computed inline from
// nxU / nxT instead of a separate nxUlaMerge pass over the line (NXModPlayer:
// mix 9.5 ms of core1 a frame, hw 2026-10-10). pu is ignored then.
template <int MODE, bool L2, bool MERGE = false>
static IRAM_ATTR __attribute__((noinline, optimize("O2", "no-unroll-loops", "no-tree-loop-distribute-patterns")))
void nxMixT(uint8_t* __restrict fb, const uint16_t* __restrict pu, const uint8_t* __restrict remap,
            uint16_t fb9, int xres) {
    const uint16_t* __restrict pl = nxL;
    const uint16_t* __restrict ps = nxS;
    const uint16_t* __restrict pt = nxT;
    if (MERGE) pu = nxU;
    for (int x = 0; x < xres; x++) {
        const uint16_t l = L2 ? pl[x] : NX_TR, s = ps[x];
        uint16_t uf = pu[x];
        if (MERGE) {
            const uint16_t tv = pt[x];
            const bool uTr0 = uf & NX_TR, tTr0 = tv & NX_TR;
            if (!tTr0 && (!(tv & NX_BELOW) || uTr0)) uf = (uint16_t)(tv & 0x1FF);
            else if (!uTr0 && !tTr0) uf = (uint16_t)(uf & ~NX_BRD);
        }
        const bool lTr = l & NX_TR, sTr = s & NX_TR, uTr = uf & NX_TR;
        const bool lPrio = (l & (NX_TR | NX_PRIO)) == NX_PRIO;
        uint16_t v = fb9;
        if (MODE == 0)      { if (lPrio) v = l; else if (!sTr) v = s; else if (!lTr) v = l; else if (!uTr) v = uf; }
        else if (MODE == 1) { if (!lTr) v = l; else if (!sTr) v = s; else if (!uTr) v = uf; }
        else if (MODE == 2) { if (lPrio) v = l; else if (!sTr) v = s; else if (!uTr) v = uf; else if (!lTr) v = l; }
        else {
            // orders 3-5: a ULA BORDER pixel gives way to a sprite where the tilemap is transparent
            const bool uOk = !uTr && !((uf & NX_BRD) && !sTr);
            if (MODE == 3)      { if (!lTr) v = l; else if (uOk) v = uf; else if (!sTr) v = s; }
            else if (MODE == 4) { if (lPrio) v = l; else if (uOk) v = uf; else if (!sTr) v = s; else if (!lTr) v = l; }
            else                { if (lPrio) v = l; else if (uOk) v = uf; else if (!lTr) v = l; else if (!sTr) v = s; }
        }
        fb[x] = (v & NX_SLOT) ? (uint8_t)v : remap[v & 0x1FF];
    }
}

// Neither Layer 2 nor a sprite has an opaque pixel on this row: every order 0-5
// reduces to "ula_final, else the fallback colour" (the border give-way of
// orders 3-5 needs an opaque sprite).
template <bool MERGE>
static IRAM_ATTR __attribute__((noinline, optimize("O2", "no-unroll-loops", "no-tree-loop-distribute-patterns")))
void nxMixU(uint8_t* __restrict fb, const uint16_t* __restrict pu, const uint8_t* __restrict remap,
            uint16_t fb9, int xres) {
    const uint16_t* __restrict pt = nxT;
    if (MERGE) pu = nxU;
    for (int x = 0; x < xres; x++) {
        uint16_t uf = pu[x];
        if (MERGE) {
            const uint16_t tv = pt[x];
            if (!(tv & NX_TR) && (!(tv & NX_BELOW) || (uf & NX_TR))) uf = (uint16_t)(tv & 0x1FF);
        }
        const uint16_t v = (uf & NX_TR) ? fb9 : uf;
        fb[x] = (v & NX_SLOT) ? (uint8_t)v : remap[v & 0x1FF];
    }
}

static IRAM_ATTR void nxMixOrder(const NxMix& m, int xres, int mode) {
    const uint16_t* pu;
    if (!m.l2 && !m.spr) {
        if (m.ulaEn && m.tm && !m.stencil) { nxMixU<true>(m.fb, nxU, m.remap, m.fb9, xres); return; }
        if (m.ulaEn && m.tm) { nxUlaMerge(m.stencil, xres); pu = nxU; }
        else pu = m.ulaEn ? nxU : nxT;
        nxMixU<false>(m.fb, pu, m.remap, m.fb9, xres);
        return;
    }
    if (m.ulaEn && m.tm && !m.stencil && m.l2) {
        switch (mode) {
            case 0: nxMixT<0, true, true>(m.fb, nxU, m.remap, m.fb9, xres); break;
            case 1: nxMixT<1, true, true>(m.fb, nxU, m.remap, m.fb9, xres); break;
            case 2: nxMixT<2, true, true>(m.fb, nxU, m.remap, m.fb9, xres); break;
            case 3: nxMixT<3, true, true>(m.fb, nxU, m.remap, m.fb9, xres); break;
            case 4: nxMixT<4, true, true>(m.fb, nxU, m.remap, m.fb9, xres); break;
            default: nxMixT<5, true, true>(m.fb, nxU, m.remap, m.fb9, xres); break;
        }
        return;
    }
    if (m.ulaEn && m.tm) { nxUlaMerge(m.stencil, xres); pu = nxU; }
    else pu = m.ulaEn ? nxU : nxT;           // ULA off: tilemap (filled transparent when off)
    if (m.l2) {
        switch (mode) {
            case 0: nxMixT<0, true>(m.fb, pu, m.remap, m.fb9, xres); break;
            case 1: nxMixT<1, true>(m.fb, pu, m.remap, m.fb9, xres); break;
            case 2: nxMixT<2, true>(m.fb, pu, m.remap, m.fb9, xres); break;
            case 3: nxMixT<3, true>(m.fb, pu, m.remap, m.fb9, xres); break;
            case 4: nxMixT<4, true>(m.fb, pu, m.remap, m.fb9, xres); break;
            default: nxMixT<5, true>(m.fb, pu, m.remap, m.fb9, xres); break;
        }
    } else {
        switch (mode) {
            case 0: nxMixT<0, false>(m.fb, pu, m.remap, m.fb9, xres); break;
            case 1: nxMixT<1, false>(m.fb, pu, m.remap, m.fb9, xres); break;
            case 2: nxMixT<2, false>(m.fb, pu, m.remap, m.fb9, xres); break;
            case 3: nxMixT<3, false>(m.fb, pu, m.remap, m.fb9, xres); break;
            case 4: nxMixT<4, false>(m.fb, pu, m.remap, m.fb9, xres); break;
            default: nxMixT<5, false>(m.fb, pu, m.remap, m.fb9, xres); break;
        }
    }
}

// Orders 6/7: Layer 2 + the blend source (NR 0x68 bits 6:5), saturated (6) or
// minus 5 (7). One SRAM loop per blend source, the NxMix fields in locals: from
// flash with the per-pixel switch it was 17 of NextSID's 26 ms of core1 a frame
// (6.4 M XIP accesses, hw 2026-10-10).
template <int BLEND>
static IRAM_ATTR __attribute__((noinline, optimize("O2", "no-unroll-loops", "no-tree-loop-distribute-patterns")))
void nxMixBlendT(uint8_t* __restrict fb, const uint8_t* __restrict remap, uint16_t fb9,
                 bool ulaEn, bool stencil, int xres, bool minus5) {
    const uint16_t* __restrict pu = nxU;
    const uint16_t* __restrict pt = nxT;
    const uint16_t* __restrict pl = nxL;
    const uint16_t* __restrict ps = nxS;
    for (int x = 0; x < xres; x++) {
        const uint16_t u = pu[x], t = pt[x], l = pl[x], s = ps[x];
        const bool uMixTr = u & NX_TR, uTr = uMixTr || !ulaEn, tTr = t & NX_TR, below = t & NX_BELOW;
        uint16_t mix, top = NX_TR, bot = NX_TR;
        if (BLEND == 0) {
            mix = uMixTr ? NX_TR : u;
            if (!tTr) { if (below) bot = t; else top = t; }
        } else if (BLEND == 2) {
            // ula_final (nxUlaFinal)
            if (stencil) mix = (uTr || tTr) ? NX_TR : (uint16_t)(u & t & 0x1FF);
            else if (!tTr && (!below || uTr)) mix = (uint16_t)(t & 0x1FF);
            else mix = uTr ? NX_TR : u;
        } else if (BLEND == 3) {
            mix = tTr ? NX_TR : t;
            if (!uTr) { if (below) top = u; else bot = u; }
        } else {
            mix = NX_TR;
            if (below) { top = uTr ? NX_TR : u; bot = tTr ? NX_TR : t; }
            else       { top = tTr ? NX_TR : t; bot = uTr ? NX_TR : u; }
        }
        const bool lTr = l & NX_TR, sTr = s & NX_TR;
        // The blended colour is needed only where Layer 2 is opaque, and with
        // the blend source transparent it is Layer 2's own colour (saturate:
        // l + 0; minus 5 applies only with both opaque). Computed lazily: the
        // per-pixel channel split + clamp was most of the loop (NextSID: mix
        // 13 of core1's 23 ms a frame, hw 2026-10-10).
        uint16_t v;
        const bool lFirst = !lTr && (l & NX_PRIO);
        if (lFirst || (!lTr && (top & NX_TR) && sTr && (bot & NX_TR))) {
            if (mix & NX_TR) {
                v = (uint16_t)(l & 0x1FF);
            } else {
                int r = ((l >> 6) & 7) + ((mix >> 6) & 7), g = ((l >> 3) & 7) + ((mix >> 3) & 7), b = (l & 7) + (mix & 7);
                if (!minus5) { if (r > 7) r = 7; if (g > 7) g = 7; if (b > 7) b = 7; }
                else {
                    r = r <= 4 ? 0 : r >= 12 ? 7 : r - 5;
                    g = g <= 4 ? 0 : g >= 12 ? 7 : g - 5;
                    b = b <= 4 ? 0 : b >= 12 ? 7 : b - 5;
                }
                v = (uint16_t)((r << 6) | (g << 3) | b);
            }
        }
        else if (!(top & NX_TR)) v = top;
        else if (!sTr) v = s;
        else if (!(bot & NX_TR)) v = bot;
        else v = fb9;
        fb[x] = (v & NX_SLOT) ? (uint8_t)v : remap[v & 0x1FF];
    }
}

static IRAM_ATTR void nxMixBlend(const NxMix& m, int xres, bool minus5) {
    switch (m.blend) {
        case 0:  nxMixBlendT<0>(m.fb, m.remap, m.fb9, m.ulaEn, m.stencil, xres, minus5); break;
        case 2:  nxMixBlendT<2>(m.fb, m.remap, m.fb9, m.ulaEn, m.stencil, xres, minus5); break;
        case 3:  nxMixBlendT<3>(m.fb, m.remap, m.fb9, m.ulaEn, m.stencil, xres, minus5); break;
        default: nxMixBlendT<1>(m.fb, m.remap, m.fb9, m.ulaEn, m.stencil, xres, minus5); break;
    }
}

IRAM_ATTR void NEXTVID::RenderLayers(uint8_t* fb, int row) {
    const int xres = VIDEO::vga.xres;
    const int y = row - paperTopRow;
    const uint8_t r15 = LS->r15;
    NX_LT_BEGIN();
    sprApply(LS->sprSeq);               // keep the attribute log moving on every line
    palApply(LS->palSeq);
    NX_LT_END(6);
    // NR 0x68 bit 7: ULA (and LoRes) output off — the mixer then never looks at
    // the ULA layer (ula_final takes the tilemap or transparent), so do not
    // compose it (TX-1696: 3 ms of core1 a frame, hw 2026-10-09). Timex hi-res
    // still needs RenderUlaLine for its pair row.
    bool ulaRow = false;
    if ((LS->r68 & 0x80) && !LS->hires) nxFillTr(nxU, xres);
    else ulaRow = RenderUlaLine(row);
    NX_LT_END(0);
    const bool tmEn = LS->r6B & 0x80;
    const bool l2En = (LS->p123B & 0x02) || (LS->r69 & 0x80);
    if (!tmEn && !l2En && !(r15 & 0x01) && !(LS->r68 & 0x80)) {
        // ULA alone (NextZXOS, every 48K/128K title): the order is moot, only
        // transparency -> fallback is left of the mixer
        const uint16_t fb9 = rgb332_to_333(LS->r4A);
        for (int x = 0; x < xres; x++) {
            const uint16_t u = nxU[x];
            fb[x] = (u & NX_SLOT) ? (uint8_t)u : remap[((u & NX_TR) ? fb9 : u) & 0x1FF];
        }
        NX_LT_END(4);
        return;
    }
    nxFillTr(nxT, xres);
    if (tmEn) RenderTilemapLine(row);
    NX_LT_END(1);
    nxFillTr(nxL, xres);
    nxRowL2 = false;
    if (l2En) RenderL2Line(row);
    NX_LT_END(2);
    nxFillTr(nxS, xres);
    nxRowSpr = false;
    if (r15 & 0x01) RenderSpritesLine(y);
    NX_LT_END(3);

#if NEXT_TRACE
    // Warhawk bottom-stripe probe: for paper rows 0 and 191, which layers are
    // opaque, the Layer 2 source row and a hash of each layer's paper pixels.
    if (y == 0 || y == 191) {
        extern volatile uint32_t g_nx_rowprobe[2][8];
        volatile uint32_t* pr = g_nx_rowprobe[y ? 1 : 0];
        uint32_t nl = 0, ns = 0, nu = 0, hl = 2166136261u, hs = 2166136261u, hu = 2166136261u;
        for (int x = xPaperOff; x < xPaperOff + 256 && x < xres; x++) {
            const uint16_t l = nxL[x], sp = nxS[x], u = nxU[x];
            if (!(l & NX_TR)) nl++;
            if (!(sp & NX_TR)) ns++;
            if (!(u & NX_TR)) nu++;
            hl = (hl ^ l) * 16777619u; hs = (hs ^ sp) * 16777619u; hu = (hu ^ u) * 16777619u;
        }
        uint16_t sy = (uint16_t)(y + LS->r17); if (sy >= 192) sy -= 192;
        pr[0] = LS->r17; pr[1] = sy; pr[2] = nl; pr[3] = ns; pr[4] = nu;
        pr[5] = hl; pr[6] = hs; pr[7] = hu;
    }
#endif
    NxMix m;
    m.fb = fb;
    m.remap = remap;
    m.fb9 = rgb332_to_333(LS->r4A);
    m.ulaEn = !(LS->r68 & 0x80);
    m.stencil = (LS->r68 & 0x01) && m.ulaEn && tmEn;
    // an all-transparent ULA row mixes exactly like the ULA switched off —
    // except under the stencil, where it blanks the tilemap
    if (!ulaRow && !m.stencil) m.ulaEn = false;
    m.tm = tmEn;
    m.l2 = l2En && nxRowL2;             // empty rows drop out of the mix
    m.spr = (r15 & 0x01) && nxRowSpr;
    m.blend = (LS->r68 >> 5) & 3;
    const int order = (r15 >> 2) & 7;
    if (order < 6) nxMixOrder(m, xres, order);
    else nxMixBlend(m, xres, order == 7);
    NX_LT_END(4);
}

// Layer 2: 256x192x8bpp (row-major) or 320x256x8bpp (column-major, covers
// the border window). the palette priority bit
// rides along as NX_PRIO for the mixer.
__attribute__((optimize("O2", "no-unroll-loops", "no-tree-loop-distribute-patterns")))
IRAM_ATTR void NEXTVID::RenderL2Line(int row) {
    uint8_t pal = (LS->palCtrl & 0x04) ? 5 : 1;
    const uint16_t* raw = rawPal[pal];
    const uint8_t transp = LS->r14;     // compared with the COLOUR (zxnext.vhd 7123)
    uint8_t mode = (LS->r70 >> 4) & 3;
    uint8_t palOfs = (LS->r70 & 0x0F) << 4;
    uint16_t bank = (uint16_t)(LS->r12 & 0x7F) * 2;
    uint16_t* const out = nxL;

    if (mode == 0) {
        // 256x192, aligned with the paper area
        int y = row - paperTopRow;
        if (y < 0 || y >= 192) return;
        if (y < LS->clip[0][2] || y > LS->clip[0][3]) return;
        uint16_t sy = y + LS->r17;
        if (sy >= 192) sy -= 192;
        uint16_t page = bank + (sy >> 5);
        if (page >= MemESP::NEXT_PAGES - 1u) return;
        const uint8_t* src = MemESP::nextPgPtr(page) + (sy & 31) * 256;
        uint8_t sx = LS->r16 + LS->clip[0][0];
        uint16_t* q = out + xPaperOff + LS->clip[0][0];
        int count = (int)LS->clip[0][1] - (int)LS->clip[0][0] + 1;
        if (!(sx & 3) && !(count & 3)) {
            // Word-aligned run (no odd X scroll / clip — Santa, most titles):
            // 4 source pixels per load (the row is in butter PSRAM, one XIP
            // access instead of four), branchless store — the buffer was
            // filled transparent, writing NX_TR again is the same value.
            const uint32_t* w = (const uint32_t*)src;
            bool any = false;
            // index 0 transparent (NXModPlayer: NR 0x14 = 0, black background):
            // four zero pixels are skipped — the buffer is already NX_TR
            const bool z0 = ((raw[palOfs] & 0x1FF) >> 1) == transp;
            for (int k = 0; k < count; k += 4, sx = (uint8_t)(sx + 4)) {
                uint32_t b = w[sx >> 2];
                if (z0 && b == 0) { q += 4; continue; }
                for (int i = 0; i < 4; i++, b >>= 8) {
                    const uint16_t e = raw[(uint8_t)(b + palOfs)];
                    const uint16_t v = e & 0x1FF;
                    const bool op = (v >> 1) != transp;
                    any |= op;
                    *q++ = op ? (uint16_t)(v | ((e >> 4) & NX_PRIO)) : NX_TR;
                }
            }
            nxRowL2 = any;
            return;
        }
        while (count-- > 0) {
            const uint16_t e = raw[(uint8_t)(src[sx++] + palOfs)];   // uint8 wrap = 256px scroll wrap
            const uint16_t v = e & 0x1FF;
            if ((v >> 1) != transp) { *q = (uint16_t)(v | ((e & 0x8000) ? NX_PRIO : 0)); nxRowL2 = true; }
            q++;
        }
    } else {
        // 320x256 (mode 1) and 640x256x4bpp (modes 2/3), both column-major
        // (byte offset = x*256 + y, layer2.vhd 160) and covering the border.
        // 640: the byte is two pixels, high nibble left, index = (offset + 0) & nibble
        // (layer2.vhd 202-203). Our fb pixel is a 320-wide one, so the pair is
        // shown as its left pixel, or the right one where the left is
        // transparent (deviation, like the 80-column tilemap).
        const bool hires = mode & 2;
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
            const uint8_t b = MemESP::nextPgPtr(page)[off & 0x1FFF];
            uint16_t e, v;
            if (!hires) {
                e = raw[(uint8_t)(b + palOfs)];
                v = e & 0x1FF;
            } else {
                e = raw[(uint8_t)(palOfs | (b >> 4))];
                v = e & 0x1FF;
                if ((v >> 1) == transp) {
                    e = raw[(uint8_t)(palOfs | (b & 0x0F))];
                    v = e & 0x1FF;
                }
            }
            if ((v >> 1) != transp) { out[fx] = (uint16_t)(v | ((e & 0x8000) ? NX_PRIO : 0)); nxRowL2 = true; }
        }
    }
}


// Tilemap text mode (NR 0x6B bit 3) — one tile's span of a line. 1bpp: pixel
// = bit (7 - x) of the row byte, palette index = attr 7:1 + pixel; never the
// NR 0x4C index test: a text pixel is transparent when its colour equals NR 0x14
// (tilemap.vhd 385-386, zxnext.vhd 7109). 80 columns are drawn at half width,
// so the pixel pair is ORed and 1-pixel strokes of the font survive (deviation).
// Flash on purpose: text mode is rare and this keeps ~1 KB out of every board's
// SRAM (RenderTilemapLine is RAM-resident).
__attribute__((noinline)) static void tmTextSpan(uint16_t* o, int n, uint8_t b, int u0, int step,
                                                 uint8_t palHi, const uint16_t* rawTM, uint8_t r14,
                                                 uint16_t below) {
    for (int k = 0; k < n; k++) {
        const int u = u0 + k * step;
        uint8_t bit = (b >> (7 - u)) & 1;
        if (step == 2 && u < 7) bit |= (b >> (6 - u)) & 1;
        const uint16_t v = rawTM[(uint8_t)(palHi | bit)] & 0x1FF;
        if ((v >> 1) != r14) o[k] = (uint16_t)(v | below);
    }
}

// Tilemap: 40x32 (or 80x32 rendered horizontally halved) tiles of 8x8x4bpp.
// Map and tile definitions live in bank 5 (offsets from nextreg 0x6E/0x6F).
// The tilemap window is 320x256 and covers the border area around the paper.
// -O2 (the file is -Os): tile setup and the span copy are per-pixel hot code
// (TX-1696: 5 ms of core1 a frame, hw 2026-10-09).
__attribute__((optimize("O2", "no-unroll-loops", "no-tree-loop-distribute-patterns")))
IRAM_ATTR void NEXTVID::RenderTilemapLine(int row) {
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
    uint16_t* const out = nxT + xOrig;

    // Tile by tile: one map entry, one attribute decode and the tile's 8
    // source pixels per tile instead of per output pixel. 80-column mode
    // renders 640 source pixels halved: every other source pixel.
    const int step = cols80 ? 2 : 1;
    const int wrap = cols80 ? 640 : 320;
    int tx = t0;
    int sx = (tx * step + scrollX) % wrap;     // advanced per tile below, no per-tile %
    while (tx <= t1) {
        const int u0 = sx & 7;
        int n = (8 - u0 + step - 1) / step;        // output pixels left in this tile
        if (n > t1 - tx + 1) n = t1 - tx + 1;
        const uint8_t* e = mrow + (sx >> 3) * entrySz;
        const uint8_t tile = e[0];
        const uint8_t attr = noAttr ? LS->r6C : e[1];
        // The ULA is drawn OVER this tile when attr bit 0 is set (512-tile
        // mode: always), unless the tilemap is forced on top (tilemap.vhd 388);
        // the mixer decides with the ULA's own transparency.
        const uint16_t below = (!onTop && ((attr & 0x01) || m512)) ? NX_BELOW : 0;
        const uint16_t tnum = (uint16_t)(tile | ((m512 && (attr & 0x01)) ? 0x100 : 0));
        if (text) {
            const uint8_t b = defsBank[(uint16_t)(defsOff + tnum * 8 + inRow) & 0x3FFF];
            if (n == 8 && step == 1) {
                // Whole 40-column text tile: two colours, 8 pixels by bit. The
                // line buffer was filled transparent and tiles never overlap,
                // so writing NX_TR for a transparent colour is the same as
                // skipping it (NXModPlayer: tilemap 6.3 ms of core1, hw 2026-10-10).
                const uint8_t ph = (uint8_t)(attr & 0xFE), r14 = LS->r14;
                const uint16_t v0 = rawTM[ph] & 0x1FF, v1 = rawTM[(uint8_t)(ph | 1)] & 0x1FF;
                const uint16_t c0 = ((v0 >> 1) != r14) ? (uint16_t)(v0 | below) : NX_TR;
                const uint16_t c1 = ((v1 >> 1) != r14) ? (uint16_t)(v1 | below) : NX_TR;
                uint16_t* __restrict o = out + tx;
                for (int k = 0; k < 8; k++) o[k] = ((b << k) & 0x80) ? c1 : c0;
                tx += 8; sx += 8; if (sx >= wrap) sx -= wrap;
                continue;
            }
            tmTextSpan(out + tx, n, b, u0, step, (uint8_t)(attr & 0xFE), rawTM, LS->r14, below);
        } else {
            const bool xm = attr & 0x08, ym = attr & 0x04, rot = attr & 0x02;
            const uint16_t defBase = (uint16_t)(defsOff + tnum * 32);
            const uint8_t palHi = attr & 0xF0;
            // The tile's 8 pixels of this row, mirror/rotate applied, indexed by
            // the source column u (0..7) the span below steps through.
            uint8_t px[8];
            if (!rot) {
                const uint8_t vv = ym ? (uint8_t)(7 - inRow) : inRow;
                // 4 bytes = 8 pixels, high nibble first; 4-aligned in the bank
                const uint32_t w = *(const uint32_t*)(defsBank + ((defBase + vv * 4) & 0x3FFC));
                if (n == 8 && step == 1) {
                    // Whole tile (every tile of a row but the first under a fine
                    // X scroll): output pixel k = nibble k of wn, no px[] and no
                    // per-pixel index arithmetic (Santa: tilemap 5.6 ms of core1
                    // a frame, hw 2026-10-10). X mirror = byte-reversed word: its
                    // low nibbles are the source's odd pixels from the right.
                    uint32_t wn = xm ? __builtin_bswap32(w)
                                     : (((w >> 4) & 0x0F0F0F0Fu) | ((w << 4) & 0xF0F0F0F0u));
                    const uint16_t* __restrict pr = rawTM + palHi;
                    uint16_t* __restrict o = out + tx;
                    for (int k = 0; k < 8; k++, wn >>= 4) {
                        const uint8_t pix = wn & 0x0F;
                        if (pix != transp) o[k] = (uint16_t)((pr[pix] & 0x1FF) | below);
                    }
                    tx += 8; sx += 8; if (sx >= wrap) sx -= wrap;
                    continue;
                }
                for (int i = 0; i < 4; i++) {
                    const uint8_t b = (uint8_t)(w >> (8 * i));
                    px[2 * i] = b >> 4; px[2 * i + 1] = b & 0x0F;
                }
                if (xm)
                    for (int i = 0; i < 4; i++) { const uint8_t t = px[i]; px[i] = px[7 - i]; px[7 - i] = t; }
            } else {
                for (int u = 0; u < 8; u++) {
                    uint8_t uu = (uint8_t)u, vv = inRow;
                    if (xm) uu = 7 - uu;                                    // X mirror
                    if (ym) vv = 7 - vv;                                    // Y mirror
                    const uint8_t t = uu; uu = vv; vv = 7 - t;              // rotate
                    const uint8_t b = defsBank[(uint16_t)(defBase + vv * 4 + (uu >> 1)) & 0x3FFF];
                    px[u] = (uu & 1) ? (b & 0x0F) : (b >> 4);
                }
            }
            uint16_t* o = out + tx;
            for (int k = 0; k < n; k++) {
                const uint8_t pix = px[u0 + k * step];
                if (pix != transp) o[k] = (uint16_t)((rawTM[pix | palHi] & 0x1FF) | below);
            }
        }
        tx += n;
        sx += n * step;
        if (sx >= wrap) sx -= wrap;
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
static inline bool sprIsRel(const uint8_t* a) {
    return (a[3] & 0x40) && (a[4] & 0xC0) == 0x40;
}

// core0: log one attribute byte for the renderer. A full log (the renderer far
// behind, or no lines rendered — skipped frames) is applied here after the
// queue drains: nothing renders then.
IRAM_ATTR void NEXTVID::sprLogPut(uint32_t e) {
    if (spr_w - spr_app >= SPR_LOG) {
        c1Drain();
        sprApply(spr_w);
    }
    sprLog[spr_w % SPR_LOG] = e;
    __dmb();
    spr_w = spr_w + 1;
}

IRAM_ATTR void NEXTVID::sprPut(uint8_t s, uint8_t b, uint8_t v) {
    sprLogPut(((uint32_t)(s & 0x7F) << 11) | ((uint32_t)b << 8) | v);
}

// Renderer: apply the log up to a line's capture position, widen the dirty range.
IRAM_ATTR void NEXTVID::sprApply(uint32_t upto) {
    SprRender& R = *sprR;
    uint32_t i = spr_app;
    if ((int32_t)(upto - i) <= 0) return;   // older position: see palApply
    __dmb();
    int lo = R.dirty ? R.lo : 127, hi = R.dirty ? R.hi : 0;
#if NEXT_TRACE
    g_nx_logapply_spr = g_nx_logapply_spr + (upto - i);
#endif
    for (; i != upto; i++) {
        const uint32_t e = sprLog[i % SPR_LOG];
        if (e & 0x80000000u) { sprPatterns[(e >> 8) & 0x3FFF] = (uint8_t)e; continue; }
        const int sn = (e >> 11) & 0x7F;
        R.attr[sn][(e >> 8) & 7] = (uint8_t)e;
        if (sn < lo) lo = sn;
        if (sn > hi) hi = sn;
    }
    if (lo <= hi) { R.lo = (uint8_t)lo; R.hi = (uint8_t)hi; R.dirty = 1; }   // patterns only: nothing to resolve
    __dmb();
    spr_app = i;
}

// Renderer: re-resolve the dirty range — from the anchor at or before lo to the
// last relative after hi (a relative is resolved against the anchor before it).
IRAM_ATTR void NEXTVID::resolveSprites() {
    SprRender& d = *sprR;
    int s0 = d.lo, s1 = d.hi + 1;
    while (s0 > 0 && sprIsRel(d.attr[s0])) s0--;
    while (s1 < 128 && sprIsRel(d.attr[s1])) s1++;
    bool visChanged = false;
    int ax = 0, ay = 0, apat = 0, azx = 0, azy = 0;
    uint8_t apal = 0;
    bool avis = false, a4bpp = false, arot = false, axm = false, aym = false, unified = false;
    uint8_t n = 0;
    for (int s = s0; s < s1; s++) {
        const uint8_t* a = d.attr[s];
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
        SprRes& r = d.res[s];
        r.x = (int16_t)x;
        r.y = (int16_t)(y > 255 ? y - 512 : y);
        r.pat = (uint16_t)(is4 ? idx * 128 : idx * 256);
        const uint8_t of = r.flags;
        r.flags = (xm ? SR_XM : 0) | (ym ? SR_YM : 0) | (rot ? SR_ROT : 0) | (is4 ? SR_4BPP : 0) | (vis ? SR_VIS : 0);
        if ((of ^ r.flags) & SR_VIS) visChanged = true;
        r.pal = pal;
        r.xs = (uint8_t)zx; r.ys = (uint8_t)zy;
    }
    if (visChanged) {
        for (int s = 0; s < 128; s++)
            if (d.res[s].flags & SR_VIS) d.vis[n++] = (uint8_t)s;
        d.n = n;
    }
    d.dirty = 0;
}


// Render all visible sprites crossing paper line y (0-191). Sprite
// coordinate space puts (32,32) at the paper top-left corner; rendering is
// clipped to the paper area for now (sprites-over-border comes later).
IRAM_ATTR void NEXTVID::RenderSpritesLine(int y) {
    if (sprR->dirty) resolveSprites();  // RenderLayers applied this line's log
    const SprRender& sn = *sprR;
    const uint16_t* sprRaw = rawPal[(LS->palCtrl & 0x08) ? 6 : 2];   // NR 0x43 bit 3
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
    uint16_t* const fbp = nxS + xPaperOff;    // indexed by paper x
    int rendered = 0;
    const int nv = sn.n;
    uint8_t collided = 0;

    for (int i = 0; i < nv; i++) {
        const SprRes& r = sn.res[sn.vis[zeroOnTop ? nv - 1 - i : i]];
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
            fbp[xp] = (uint16_t)(sprRaw[c] & 0x1FF);
            nxRowSpr = true;
        }
    }
    if (collided) sprFlags |= 0x01;
}


