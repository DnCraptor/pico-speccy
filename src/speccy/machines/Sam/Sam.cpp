// pico-speccy — MGT SAM Coupe core. See Sam.h for the model and its sources.
//
// Everything here is FLASH code on purpose: SAM runs only on boards with butter
// PSRAM, and the shared RAM-resident hot paths (Ports::input/output, the accessors)
// keep one predicted-not-taken test that sends a SAM access here.

#include "Sam.h"
#include "SamFdc.h"

#include <string.h>

#include "app/Config.h"
#include "app/ESPectrum.h"
#include "app/Buffer.h"
#include "app/Debug.h"
#include "speccy/core/MemESP.h"
#include "speccy/core/Ports.h"
#include "speccy/z80/CPU.h"
#include "speccy/z80/z80.h"
#include "pico/time.h"
#include "speccy/z80/MemGates.h"   // g_atm_ro: bit n = drop writes to CPU window n
#include "speccy/video/Video.h"
#include "speccy/devices/tape/Tape.h"
#include "speccy/devices/sound/SAASound.h"
#include "ui/LEDIndicators.h"
#include "speccy/roms/sam/sam_roms.h"
#include "speccy/devices/disk/DiskSlots.h"
#include "app/MachineSwitch.h"
#include "hardware/structs/watchdog.h"
#include "hardware/watchdog.h"

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
static uint8_t        s_cont = 0;         // windows showing internal RAM (contended)
static uint8_t        s_romw = 0;         // windows showing ROM (the F-key AUTO test)
static int8_t         s_keysOvr = -1;     // F-keys: -1 AUTO, 0 OFF, 1 ON (session)
static uint32_t       s_fkPollMs = 0;     // last F-key row read from RAM code
static uint8_t        s_inj[9] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};  // auto-typed keys (active low)
static uint8_t        s_hpen = 192, s_lpen = 0;
static bool           s_lineSeen = false;  // a LINE write since reset (bootTick waits for it)
static int16_t        s_boot = -1;         // auto-BOOT: -1 off, else frames since the ROM came up  // last HPEN/LPEN (held while the screen is off)

static constexpr uint8_t LMPR_PAGE = 0x1F, LMPR_ROM0_OFF = 0x20, LMPR_ROM1 = 0x40, LMPR_WPROT = 0x80;
static constexpr uint8_t HMPR_PAGE = 0x1F, HMPR_MCNTRL = 0x80;
static constexpr uint8_t VMPR_PAGE = 0x1F, VMPR_MODE = 0x60, VMPR_MDE1 = 0x40;
static constexpr uint8_t BORDER_SOFF = 0x80;

// The tape trap's three ROM1 addresses (SimCoupe Memory.cpp rom_hooks): LDSTRT
// `RET Z / LD B,8` at #E678, LOADFAIL `XOR H / RET NZ / LD A,C` at #E6F5,
// LOADEXIT `LD A,L / CP 1 / RET` at #E739. Checked against the shipped image at
// bind time — a ROM that does not carry them keeps the trap off.
static constexpr uint16_t TAPE_LOADEXIT = 0xE739, TAPE_LOADFAIL = 0xE6F5;
static bool s_tapeTrapOk = false;

static bool romHas(const uint8_t* rom1, uint16_t addr, const uint8_t* pat, int n) {
    return memcmp(rom1 + (addr - 0xC000), pat, n) == 0;
}

void bindRoms() {
    s_rom0 = gb_rom_sam_30;
    s_rom1 = gb_rom_sam_30 + 0x4000;
    static const uint8_t kStrt[] = {0xC8, 0x06, 0x08}, kFail[] = {0xAC, 0xC0, 0x79},
                         kExit[] = {0x7D, 0xFE, 0x01, 0xC9};
    s_tapeTrapOk = romHas(s_rom1, Sam::TAPE_LDSTRT, kStrt, 3) &&
                   romHas(s_rom1, TAPE_LOADFAIL, kFail, 3) &&
                   romHas(s_rom1, TAPE_LOADEXIT, kExit, 4);
    if (!s_tapeTrapOk) Debug::log("[SAM] ROM loader signatures not found: fast tape load off");
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
    s_cont |= (uint8_t)(1u << w);
}

// External page (LEPR/HEPR, 0..255): ram[32 + page], or absent past the fitted megabytes.
static void mapExt(int w, uint8_t page) {
    if (page >= s_extPages) { mapAbsent(w); return; }
    MemESP::plugPtr16(w, MemESP::ram[32 + page].sync(w));
}

void remap() {
    s_ro = 0;
    s_cont = 0;
    s_romw = 0;
    // A
    if (!(lmpr & LMPR_ROM0_OFF)) { mapRo(0, s_rom0); s_romw |= 1; }
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
    else if (lmpr & LMPR_ROM1) { mapRo(3, s_rom1); s_romw |= 8; }
    else                       mapInt(3, (hmpr + 1) & HMPR_PAGE);

    // Contention (SimCoupe PageIn): internal RAM only — not ROM, external or absent memory.
    for (int w = 0; w < 4; w++)
        MemESP::ramContended[w * 2] = MemESP::ramContended[w * 2 + 1] = (s_cont >> w) & 1;
    g_atm_ro = s_ro;
    MemESP::romInUse = 0;
    MemESP::p3special = 2;        // recoverPage0() keeps out of window 0
    MemESP::pagingLock = 0;
}

static void periphReset();

void reset() {
    periphReset();
    const RomsetIdx r = Config::romSetSam;
    s_intPages = samIntPages(r);
    s_extPages = samExtPages(r);
    if (32 + s_extPages > MEM_PG_CNT) s_extPages = MEM_PG_CNT > 32 ? MEM_PG_CNT - 32 : 0;
    if (!s_rom0) bindRoms();
    lmpr = hmpr = vmpr = lepr = hepr = border = attr = 0;
    line = 0xFF;
    status = 0xFF;
    s_hpen = 192; s_lpen = 0;
    s_lineSeen = false;
    memset(s_inj, 0xFF, sizeof(s_inj));
    memset(clut, 0, sizeof(clut));
    clutDirty = true;
    SamFdc::reset();
    remap();
    Debug::log("[SAM] reset: %uK internal + %u MB external", (unsigned)(s_intPages * 16),
               (unsigned)(s_extPages / 64));
}

// ---------------------------------------------------------------------------
// Disk launch and auto-BOOT
//
// ROM 3.0 boots a disk on BOOT + Enter (SimCoupe's own autoboot types exactly
// that; F9 at the copyright screen did nothing in tools/sam_sim). The keys go
// straight into the matrix the ROM scans (s_inj), held 5 frames, 10 apart,
// starting 20 frames after the ROM's frame-INT handler first writes LINE — the
// RAM test before it takes longer with external memory, so no fixed delay.

static constexpr uint32_t BOOT_SCRATCH = 1;             // shares ESPectrum's P3BT register
static constexpr uint32_t BOOT_TAG = 0x53414D42u;       // 'SAMB'

struct BootKey { uint8_t row, bit; };
// The first key only leaves the copyright screen and is swallowed (hw 2026-10-22:
// "OOT" got typed), so an Enter goes first — harmless if the screen is already gone.
static const BootKey kBootKeys[] = { {6, 0}, {7, 4}, {5, 1}, {5, 1}, {2, 4}, {6, 0} };   // Enter B O O T Enter
// Tape launch: LOAD "" — the SAM has its own " key (row 5 bit 6, unshifted).
static const BootKey kLoadKeys[] = { {6, 0}, {6, 1}, {5, 1}, {1, 0}, {1, 2}, {5, 6}, {5, 6}, {6, 0} };   // Enter L O A D " " Enter
static constexpr int BOOT_START = 20, BOOT_STEP = 15, BOOT_HOLD = 5;
static const BootKey* s_seq = kBootKeys;
static int            s_seqN = (int)(sizeof(kBootKeys) / sizeof(kBootKeys[0]));
static bool           s_seqDisk = true;    // the sequence needs drive 1 mounted (else a tape)

static void seqArm(const BootKey* seq, int n, bool disk) {
    s_seq = seq; s_seqN = n; s_seqDisk = disk;
    s_boot = 0;
    memset(s_inj, 0xFF, sizeof(s_inj));
}

void bootArm() {
    watchdog_hw->scratch[BOOT_SCRATCH] = 0;
    seqArm(kBootKeys, (int)(sizeof(kBootKeys) / sizeof(kBootKeys[0])), true);
}

void bootArmAcrossReboot() { watchdog_hw->scratch[BOOT_SCRATCH] = BOOT_TAG; }

void bootResume() {
    Config::samKeepTmp = false;
    if (watchdog_hw->scratch[BOOT_SCRATCH] != BOOT_TAG) return;
    watchdog_hw->scratch[BOOT_SCRATCH] = 0;
    if (watchdog_caused_reboot() && Config::arch == A_SAM && SamFdc::mounted(0)) {
        Debug::log("[SAM] disk auto-boot armed after reboot");
        bootArm();
    }
}

void frameTick() {
    if (s_boot < 0) return;
    const bool have = s_seqDisk ? SamFdc::mounted(0) : (Tape::tapeFileName != "none");
    if (!have) { s_boot = -1; memset(s_inj, 0xFF, sizeof(s_inj)); return; }
    if (!s_lineSeen) return;                       // the ROM is not up yet
    const int t = s_boot++ - BOOT_START;
    memset(s_inj, 0xFF, sizeof(s_inj));
    if (t < 0) return;
    const int i = t / BOOT_STEP, ph = t % BOOT_STEP;
    if (i >= s_seqN) { s_boot = -1; return; }
    if (ph < BOOT_HOLD) s_inj[s_seq[i].row] &= (uint8_t)~(1u << s_seq[i].bit);
}

void launchTape() {
    if (Config::arch != A_SAM) return;
    ESPectrum::resetForLoad();
    seqArm(kLoadKeys, (int)(sizeof(kLoadKeys) / sizeof(kLoadKeys[0])), false);
}

// ---------------------------------------------------------------------------
// Fast tape load: the ROM 3.0 loader trap (SimCoupe Tape.cpp LoadTrap)
//
// ROM1 #E65D is the loader: A' = the wanted block type, HL = destination, DE =
// length bits 0-15, (#5AC8) = bits 16-23 — a SAM load may exceed 16K and the ROM
// steps HMPR whenever HL reaches #C000 (#E707). After the BREAK test (CALL #0E69 /
// RET Z at #E678) the pilot search starts at #E679; check_trdos sees the RET into
// #E678 and calls here. Type 0 = a Spectrum header (17 bytes whatever DE says).
// The block's bytes go through the ROM's own parity (L = type ^ every byte incl.
// the checksum, so 0 on a good block) and the trap leaves through the ROM's exits:
// LOADEXIT (LD A,L / CP 1 / RET) or LOADFAIL (XOR H / RET NZ) with A = the wanted
// type and H = the tape's, i.e. what the ROM has there on a mismatch. A block too
// short for the request takes LOADFAIL too, with A = ~H so RET NZ is taken.
bool tapeTrap() {
    if (!s_tapeTrapOk || !(s_romw & 8)) return false;      // ROM1 not in section D
    if (Z80::getFlags() & 0x40) return false;    // BREAK: the RET Z returns
    if (!Tape::fastLoadEnabled() || Tape::tapeFileName == "none") return false;
    Tape::FlashBlk b;
    if (!Tape::flashBlockOpen(b)) return false;
    const uint8_t want = Z80::getRegAx();
    const uint8_t type = b.flag;
    uint32_t left = b.dataLen;
    uint16_t dest = Z80::getRegHL();
    uint32_t n = ((uint32_t)MemESP::readbyte(0x5AC8) << 16) | Z80::getRegDE();
    if (type == 0) n = (n & ~0xFFu) | 17;
    else if (type != want) {
        Tape::flashBlockDone();
        Z80::setRegA(want); Z80::setRegH(type);
        Z80::setFlags(Z80::getFlags() & ~(0x40 | 0x01));
        Z80::setRegPC(TAPE_LOADFAIL);
        return true;
    }
    uint8_t par = type, last = type;
    for (;;) {
        if (!left) {
            Tape::flashBlockDone();
            Z80::setRegA((uint8_t)~last); Z80::setRegH(last); Z80::setRegL(par);
            Z80::setFlags(Z80::getFlags() & ~(0x40 | 0x01));
            Z80::setRegPC(TAPE_LOADFAIL);
            return true;
        }
        const uint8_t v = Tape::flashBlockRead();
        par ^= v; last = v; left--;
        if (!n) break;                              // that was the checksum
        if (!(s_ro & (1u << (dest >> 14)))) MemESP::writebyte(dest, v);
        dest++; n--;
        if (dest >= 0xC000) { hmpr = (uint8_t)(hmpr + 1); remap(); dest -= 0x4000; }
    }
    Z80::setRegH(last); Z80::setRegL(par);
    Tape::flashBlockDone();
    if (Tape::tapeStatus == TAPE_LOADING) Tape::Stop();
    LED::touchR(LED::TAPE);
    Z80::setRegPC(TAPE_LOADEXIT);
    return true;
}

bool launchDisk(const std::string& path) {
    DiskSlots::slotMount(IFACE_SAM, 0, path);
    if (!SamFdc::mounted(0)) return false;
    Config::ram_file = NO_RAM_FILE;
    Config::last_ram_file = NO_RAM_FILE;
    // Switching machine may reboot: every save until then keeps a /tmp path.
    Config::samKeepTmp = (Config::arch != A_SAM);
    Config::save();
    if (Config::arch != A_SAM) {
        bootArmAcrossReboot();
        MachineSwitch::commit(A_SAM, Config::romSetSam);   // resets, or reboots and never returns
    } else {
        ESPectrum::resetForLoad();
    }
    Config::samKeepTmp = false;              // reached only without a reboot
    bootArm();
    return true;
}

// ---------------------------------------------------------------------------
// Function keys

bool keysToGuest() {
    if (s_keysOvr >= 0) return s_keysOvr != 0;
    return s_fkPollMs && (to_ms_since_boot(get_absolute_time()) - s_fkPollMs) < 2000;
}

KeysMode cycleKeysMode() {
    switch (s_keysOvr) {
        case KEYS_AUTO: s_keysOvr = keysToGuest() ? KEYS_OFF : KEYS_ON; break;   // always a change
        case KEYS_ON:   s_keysOvr = KEYS_OFF; break;
        default:        s_keysOvr = KEYS_AUTO; break;
    }
    return (KeysMode)s_keysOvr;
}

void clearKeysOverride() { s_keysOvr = -1; s_fkPollMs = 0; }

// ---------------------------------------------------------------------------
// Interrupts

__attribute__((noinline)) bool intLine() {
    uint32_t t = CPU::tstates;
    if (t >= CPU::statesInFrame) t -= CPU::statesInFrame;   // overshoot = next frame
    if (t >= (uint32_t)CPU::IntStart && t < (uint32_t)CPU::IntEnd) return true;
    return lineIntUp(t);
}

// ---------------------------------------------------------------------------
// Contention (SimCoupe Memory.cpp Init / UpdateContention, SAMIO.h WaitStates)

static constexpr uint32_t CONT_OFFSET = 4;   // CPU_CYCLES_SCREEN_CONTENTION_OFFSET

static inline uint32_t frameT() {
    uint32_t t = CPU::tstates;
    if (t >= CPU::statesInFrame) t -= CPU::statesInFrame;
    return t;
}

// Wait before a memory access starting at frame time t.
static inline uint32_t memWait(uint32_t t) {
    uint32_t mask = 3;
    const bool off = (border & BORDER_SOFF) && (vmpr & VMPR_MDE1);
    if (!off) {
        const uint32_t ln = t / T_PER_LINE, lc = (t + CONT_OFFSET) % T_PER_LINE;
        const bool screen = ln >= TOP_LINES && ln < TOP_LINES + 192 && lc >= 128;
        const bool mode1 = (vmpr & VMPR_MODE) == 0;
        if (screen || (mode1 && !(lc & 0x40))) mask = 7;
    }
    return mask - ((t + 2) & mask);
}

// ASIC ports (>= #F8): one 8 T slot.
static inline uint32_t ioWait(uint32_t t) { return 7 - ((t + 2) & 7); }

void draw(unsigned int n, bool contended) {
    // n < 3 is addressOnBus (internal cycles): no MREQ, so no wait on the SAM.
    if (contended && n >= 3) CPU::tstates += memWait(frameT());
    CPU::tstates += n;
}

void drawOpcode(bool contended) {
    if (contended) CPU::tstates += memWait(frameT());
    CPU::tstates += 4;
}

// ---------------------------------------------------------------------------
// Ports

static uint8_t keyRows(uint8_t hi, uint8_t mask) {
    uint8_t k = mask;
    for (int r = 0; r < 8; r++)
        if (!(hi & (1u << r))) k &= keys[r] & s_inj[r];
    return k;
}

// SAMVox (SimCoupe SAMVox.cpp, reverse-engineered from Modules Tracker): four
// 8-bit DACs on #7C-#7F — 0 right, 1 left, 2 right 2, 3 left 2. Played through
// the Covox buffers (Audio > Covox on): each side is the average of the DACs a
// program has used on it, the SounDrive rule, so one DAC a side keeps full scale.
static uint8_t s_vox[4] = {0x80, 0x80, 0x80, 0x80};
static uint8_t s_voxUsed;
static void samVox(uint8_t ch, uint8_t v) {
    if (!ESPectrum::audioBufferCovoxL) return;
    s_vox[ch] = v; s_voxUsed |= (uint8_t)(1u << ch);
    const int rn = ((s_voxUsed & 1) ? 1 : 0) + ((s_voxUsed & 4) ? 1 : 0);
    const int ln = ((s_voxUsed & 2) ? 1 : 0) + ((s_voxUsed & 8) ? 1 : 0);
    const int r = rn ? (((s_voxUsed & 1) ? s_vox[0] : 0) + ((s_voxUsed & 4) ? s_vox[2] : 0)) / rn : 0x80;
    const int l = ln ? (((s_voxUsed & 2) ? s_vox[1] : 0) + ((s_voxUsed & 8) ? s_vox[3] : 0)) / ln : 0x80;
    ESPectrum::CovoxGetSample();
    ESPectrum::lastCovoxVal = l;
    ESPectrum::lastCovoxValR = r;
    LED::touchW(LED::COVOX);
}

// SAM mouse (SimCoupe Mouse.cpp): reads of #FE with A15..A8 = #FF step through a
// 9-nibble buffer — strobe, dummy, buttons (active low: bit 0 left, 1 middle,
// 2 right), Y 256/16/1, X 256/16/1 — the movement since the last full read, Y up
// positive. 38 us without a read resets it to the start. Only with a real USB mouse.
static uint8_t  s_mPos;
static uint64_t s_mLastT;
static int32_t  s_mX, s_mY;           // ESPectrum::mouseX/Y already reported
static uint8_t  s_mBuf[9];
static uint8_t mouseIn() {
    const uint64_t now = CPU::global_tstates + CPU::tstates;
    if (now - s_mLastT > 228) s_mPos = 0;  // 38 us at 6 MHz
    s_mLastT = now;
    if (s_mPos == 0) { s_mBuf[0] = s_mBuf[1] = 0xFF; }
    if (s_mPos == 2) {
        int dx = ESPectrum::mouseX - s_mX, dy = ESPectrum::mouseY - s_mY;
        if (dx > 127) dx = 127; else if (dx < -127) dx = -127;
        if (dy > 127) dy = 127; else if (dy < -127) dy = -127;
        s_mX += dx; s_mY += dy;
        s_mBuf[2] = (uint8_t)~((ESPectrum::mouseButtonL ? 1 : 0) | (ESPectrum::mouseButtonM ? 2 : 0) |
                               (ESPectrum::mouseButtonR ? 4 : 0));
        s_mBuf[3] = (dy >> 8) & 0xF; s_mBuf[4] = (dy >> 4) & 0xF; s_mBuf[5] = dy & 0xF;
        s_mBuf[6] = (dx >> 8) & 0xF; s_mBuf[7] = (dx >> 4) & 0xF; s_mBuf[8] = dx & 0xF;
        LED::touchR(LED::KEMPMOUSE);
    }
    const uint8_t r = s_mBuf[s_mPos];
    if (++s_mPos >= 9) s_mPos = 1;   // back to the data, still strobed
    return (uint8_t)(0xF0 | r);
}

static void periphReset() {
    s_mPos = 0; s_mX = ESPectrum::mouseX; s_mY = ESPectrum::mouseY;
    s_voxUsed = 0; s_vox[0] = s_vox[1] = s_vox[2] = s_vox[3] = 0x80;
}

uint8_t portIn(uint16_t address) {
    if ((uint8_t)address >= 0xF8) CPU::tstates += ioWait(frameT());
    CPU::tstates += 4;
    const uint8_t lo = (uint8_t)address, hi = (uint8_t)(address >> 8);
    switch (lo) {
    case 0xFE: {
        // The tape advances on EAR reads (Tape::Read scales the 6 MHz clock to the
        // 3.5 MHz pulse lengths). No ROM traps: the SAM loader is not the
        // Spectrum's. Auto-start: a stopped tape starts when one PC reads #FE 200
        // times in a row (an edge-polling loader; the keyboard scan reads 9 rows a
        // frame from different places) — the TapePortRead rule, so LOAD "" from
        // a mounted tape needs no Play.
        if (Tape::tapeStatus == TAPE_LOADING) { Tape::Read(); LED::touchR(LED::TAPE); }
        else if (Config::tape_autostart && Tape::tapeFileType != TAPE_FTYPE_EMPTY &&
                 Tape::tapeFileName != "none") {
            static uint16_t s_pc; static uint16_t s_n;
            const uint16_t pc = Z80::getRegPC();
            if (pc != s_pc) { s_pc = pc; s_n = 0; }
            else if (++s_n >= 200) { s_n = 0; Tape::Play(); }
        }
        uint8_t k = (hi == 0xFF) ? (uint8_t)(keys[8] & 0x1F) : keyRows(hi, 0x1F);
        if (hi == 0xFF && ESPectrum::mouseSeen) k &= mouseIn();
        return (uint8_t)(k | (border & BORDER_SOFF) | (Tape::tapeEarBit ? 0x40 : 0));
    }
    case 0xF9: {
        // Frame INT: bit 3 low inside the 128 T window (the same one isActiveINT raises).
        // F-keys live in rows 0-2, bits 5-7: a RAM program scanning them wants them.
        if ((hi & 0x07) != 0x07 && !(s_romw & (1u << (Z80::getRegPC() >> 14))))
            s_fkPollMs = to_ms_since_boot(get_absolute_time());
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
    default:
        if ((lo & 0xF8) == 0xE0) return SamFdc::in(0, address);   // drive 1
        if ((lo & 0xF8) == 0xF0) return SamFdc::in(1, address);   // drive 2
        return 0xFF;
    }
}

void portOut(uint16_t address, uint8_t data) {
    if ((uint8_t)address >= 0xF8) CPU::tstates += ioWait(frameT());
    CPU::tstates += 4;
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
        s_lineSeen = true;            // the ROM's interrupt handler is running
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
    default:
        if ((lo & 0xF8) == 0xE0)      SamFdc::out(0, address, data);
        else if ((lo & 0xF8) == 0xF0) SamFdc::out(1, address, data);
        else if ((lo & 0xFC) == 0x7C) samVox(lo & 3, data);
        break;
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
