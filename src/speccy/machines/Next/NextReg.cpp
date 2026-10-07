/*

pico-speccy: ZX Spectrum Next (TBBlue) NextReg register file

*/

#include "NextReg.h"
#include "speccy/core/MemESP.h"
#include "speccy/z80/CPU.h"
#include "app/ESPectrum.h"
#include "speccy/video/Video.h"
#include "app/Debug.h"
#include "app/Config.h"
#include "hardware/regs/xip.h"
#include "hardware/regs/addressmap.h"
#include "NextVideo.h"
#include "app/Debug.h"
#include "speccy/z80/z80.h"
#include "speccy/devices/storage/DivMMC.h"
#include <cstdio>
#include <cstring>

static void applyTimexMode(uint8_t v);   // port FF / NR 0x69 screen-mode bits (below)

extern "C" const unsigned char gb_rom_tbblue_loader[];  // src/speccy/roms/next/

#pragma GCC optimize("O3")

uint8_t NextReg::reg[256];
uint8_t NextReg::selected = 0;

uint8_t NextReg::port7FFD = 0;
uint8_t NextReg::portDFFD = 0;
uint8_t NextReg::port1FFD = 0;
uint8_t NextReg::port123B = 0;

uint8_t  NextReg::resetPending = 0;
bool     NextReg::lineIrqEnabled = false;
bool     NextReg::ulaIrqDisabled = false;
uint16_t NextReg::lineIrqLine = 0;
uint8_t  NextReg::portFF = 0;
bool     NextReg::portFFIntDisable = false;

// NR 0x03 sub-fields (zxnext.vhd:1099-1103 defaults; write rules 5121-5151,
// read mux 5894 — semantics pinned via zx_go WireMachineType). The flat
// reg[0x03] byte is not enough: writes update the fields independently and
// reads RECONSTRUCT the byte — NextZXOS's post-boot `NEXTREG 0x03,0xB0`
// (bits 2:0 = 000) must leave machine_type intact.
static uint8_t s_machineType   = 0x03;
static uint8_t s_machineTiming = 0x03;
static uint8_t s_userDTLock    = 0;
static uint8_t s_reg03TopBit   = 1;
static uint8_t reg03Compose() {
    return (uint8_t)(((s_reg03TopBit & 1) << 7) | ((s_machineTiming & 7) << 4) |
                     ((s_userDTLock & 1) << 3) | (s_machineType & 7));
}

#if NEXT_TRACE
// --- NEXT_TRACE ring (see NextReg.h for the design rules) ---
struct NextTraceEnt { char kind; uint8_t reg, val; uint16_t pc; uint16_t rep; };
static NextTraceEnt s_nt_ring[64];
static uint32_t s_nt_w = 0;                  // total writes; ring index = w & 63
static uint32_t s_nt_pal = 0, s_nt_cop = 0, s_nt_spr = 0;   // flood counters
void next_trace_ev(char kind, uint8_t r, uint8_t v) {
    // Consecutive duplicates collapse into one entry with a repeat count —
    // a file loader hammering NR 0x04 with the same page (2600+/s, hw
    // 2026-08-09) purged the whole ring and hid the one interesting write.
    if (s_nt_w) {
        NextTraceEnt& last = s_nt_ring[(s_nt_w - 1) & 63];
        uint16_t pc = Z80::getRegPC();
        if (last.kind == kind && last.reg == r && last.val == v &&
            last.pc == pc && last.rep < 0xFFFF) {
            last.rep++;
            return;
        }
    }
    NextTraceEnt& e = s_nt_ring[s_nt_w & 63];
    e.kind = kind; e.reg = r; e.val = v;
    e.pc = Z80::getRegPC();
    e.rep = 1;
    s_nt_w++;
}
void next_trace_dump(uint32_t frames) {
    extern bool g_next_automap_armed;
    extern volatile uint32_t g_sd_stall_us, g_sd_ops;   // DivMMC.cpp
    extern uint32_t g_nextvid_frames;
    extern volatile uint32_t g_nextvid_us;              // NextVideo.cpp
    extern volatile uint32_t g_nextvid_layer_us[4];     // ula / tilemap / L2 / sprites
    // Who REALLY owns slot 0 right now — the flags can say "bootrom" while a
    // missed replug hands the CPU something else entirely (2026-08-09: flags
    // said boot=1 while the machine visibly ran a machine ROM).
    const uint8_t* s0 = MemESP::ramCurrent[0];
    char own = '?';
    if (s0 == (const uint8_t*)gb_rom_tbblue_loader) own = 'B';       // bootrom
    else if (MemESP::nextRomBase && s0 >= MemESP::nextRomBase &&
             s0 < MemESP::nextRomBase + 0x10000) own = 'N';          // enNextZX
    else if (s0 == MemESP::rom[MemESP::romInUse & 1].direct()) own = 'F'; // 128K fallback
    else if (s0 >= (const uint8_t*)0x20000000 &&
             s0 <  (const uint8_t*)0x20082000) own = 'S';            // SRAM page
    else if (s0 >= (const uint8_t*)0x11000000 &&
             s0 <  (const uint8_t*)0x12000000) own = 'P';            // butter page
    Debug::log("NEXT: pc=%04X s0=%08X%c s1=%08X spd=%u/%u"
               " mmu=%02X.%02X.%02X.%02X.%02X.%02X.%02X.%02X"
               " rom=%u boot=%c cfg=%c pg=%02X am=%u 7FFD=%02X 1FFD=%02X DFFD=%02X"
               " FF=%02X fps=%u pal/s=%u cop/s=%u spr/s=%u sd=%ums/%u vid=%ums",
        (unsigned)Z80::getRegPC(), (unsigned)(uintptr_t)s0, own,
        (unsigned)(uintptr_t)MemESP::ramCurrent[1],
        (unsigned)ESPectrum::multiplicator, (unsigned)(NextReg::reg[0x07] & 3),
        MemESP::mmu[0], MemESP::mmu[1], MemESP::mmu[2], MemESP::mmu[3],
        MemESP::mmu[4], MemESP::mmu[5], MemESP::mmu[6], MemESP::mmu[7],
        (unsigned)MemESP::romInUse,
        MemESP::nextBootRomActive ? '1' : '0', MemESP::nextConfigMode ? '1' : '0',
        MemESP::nextConfigPage, (unsigned)DivMMC::automap,
        NextReg::port7FFD, NextReg::port1FFD, NextReg::portDFFD, NextReg::portFF,
        (unsigned)frames, (unsigned)s_nt_pal, (unsigned)s_nt_cop, (unsigned)s_nt_spr,
        (unsigned)(g_sd_stall_us / 1000u), (unsigned)g_sd_ops,
        (unsigned)(g_nextvid_us / 1000u));
    // Frame cost on its own line (the one above is near Debug::log's 256-byte
    // cap). VIDEO::EndFrame returns before its PERF block on the Next, so this
    // is the only place the CPU::loop time shows: cpu = ms per emulated frame
    // (Z80 + rendering + DMA, the whole CPU::loop), vid = the renderer's share,
    // li = line interrupt armed (the frame then runs execute() per instruction).
    {
        extern volatile uint32_t cpu_frame_us;          // CPU.cpp, never reset on Next
        static uint32_t s_prev_cpu = 0;
        const uint32_t c = cpu_frame_us, d = c - s_prev_cpu;
        s_prev_cpu = c;
        const uint32_t f = frames ? frames : 1;
        Debug::log("NEXT cpu: %u.%ums/fr vid=%u.%ums/fr li=%u line=%u clk=%u MHz fps=%u draw=%u max=%u",
            (unsigned)(d / f / 1000u), (unsigned)(d / f % 1000u / 100u),
            (unsigned)(g_nextvid_us / f / 1000u), (unsigned)(g_nextvid_us / f % 1000u / 100u),
            (unsigned)NextReg::lineIrqEnabled, (unsigned)NextReg::lineIrqLine,
            (unsigned)(ESPectrum::multiplicator == 0 ? 3 : 7u << (ESPectrum::multiplicator - 1)),
            (unsigned)frames, (unsigned)g_nextvid_frames,
            (unsigned)ESPectrum::maxSpeed);
        // Which layer the renderer's time goes to (same sampled lines, the
        // paper rows only — border rows are in vid but not split here).
        {
            extern volatile uint32_t g_dma_n, g_dma_bytes, g_dma_endframes, g_dma_maxlen,
                                     g_dma_max_pc, g_dma_max_src, g_dma_max_dst, g_dma_max_fl;
            // fl: 100 A->B, 80 auto-restart, 40 #0B (Zilog) mode, 2 src IO,
            // 1 dst IO, bits 12+ = prescaler
            Debug::log("NEXT dma: n=%u bytes=%u endfr=%u max=%u pc=%04X %04X->%04X fl=%X",
                (unsigned)g_dma_n, (unsigned)g_dma_bytes, (unsigned)g_dma_endframes,
                (unsigned)g_dma_maxlen, (unsigned)g_dma_max_pc, (unsigned)g_dma_max_src,
                (unsigned)g_dma_max_dst, (unsigned)g_dma_max_fl);
            g_dma_n = g_dma_bytes = g_dma_endframes = g_dma_maxlen = 0;
        }
        Debug::log("NEXT vid: ula=%u.%u tm=%u.%u l2=%u.%u spr=%u.%u ms/fr",
            (unsigned)(g_nextvid_layer_us[0] / f / 1000u), (unsigned)(g_nextvid_layer_us[0] / f % 1000u / 100u),
            (unsigned)(g_nextvid_layer_us[1] / f / 1000u), (unsigned)(g_nextvid_layer_us[1] / f % 1000u / 100u),
            (unsigned)(g_nextvid_layer_us[2] / f / 1000u), (unsigned)(g_nextvid_layer_us[2] / f % 1000u / 100u),
            (unsigned)(g_nextvid_layer_us[3] / f / 1000u), (unsigned)(g_nextvid_layer_us[3] / f % 1000u / 100u));
#if PERF_TRACE && PERF_HIST
        // The TS-Conf opcode histogram (Z80_JLS.cpp, exec_nocheck), printed here
        // because VIDEO::EndFrame's PERF block never runs on the Next. ns/instr is
        // against this second's CPU::loop time — pessimistic by the histogram's own
        // cost, as on TS-Conf. Prefixes count as themselves (ED = IN/OUT/block ops).
        {
            extern uint32_t z80_op_hist[256];
            uint64_t total = 0;
            for (int i = 0; i < 256; i++) total += z80_op_hist[i];
            char hl[200]; int hp = 0; uint8_t used[256] = {0};
            for (int k = 0; k < 16 && hp < (int)sizeof(hl) - 12; k++) {
                int best = -1; uint32_t bv = 0;
                for (int i = 0; i < 256; i++)
                    if (!used[i] && z80_op_hist[i] > bv) { bv = z80_op_hist[i]; best = i; }
                if (best < 0 || bv == 0) break;
                used[best] = 1;
                hp += snprintf(hl + hp, sizeof(hl) - hp, " %02X:%u", best,
                               (unsigned)(total ? (uint64_t)bv * 100 / total : 0));
            }
            hl[hp] = 0;
            Debug::log("NEXT z80: %u instr/fr %u ns/instr top%%:%s",
                (unsigned)(total / f), (unsigned)(total ? (uint64_t)d * 1000 / total : 0), hl);
            for (int i = 0; i < 256; i++) z80_op_hist[i] = 0;
            extern void pcprof_dump();
            pcprof_dump();
        }
#endif
    }
    s_nt_pal = s_nt_cop = s_nt_spr = 0;
    g_sd_stall_us = 0;
    g_sd_ops = 0;
    g_nextvid_us = 0;
    g_nextvid_frames = 0;
    for (int i = 0; i < 4; i++) g_nextvid_layer_us[i] = 0;
    // Ring line: newest LAST, budget spent from the newest end backwards, a
    // leading ".." marks dropped older entries (never drop the newest — the
    // NeoGS ring printed oldest-first once and hid every wedge it saw).
    char line[200];
    uint32_t n = s_nt_w < 64 ? s_nt_w : 64;
    int pos = sizeof(line);
    line[--pos] = 0;
    uint32_t shown = 0;
    for (uint32_t i = 0; i < n; i++) {
        const NextTraceEnt& e = s_nt_ring[(s_nt_w - 1 - i) & 63];
        char ent[24];
        int el;
        if (e.rep > 1)
            el = snprintf(ent, sizeof(ent), "%c%02X:%02X@%04X*%u ",
                          e.kind, e.reg, e.val, e.pc, (unsigned)e.rep);
        else
            el = snprintf(ent, sizeof(ent), "%c%02X:%02X@%04X ", e.kind, e.reg, e.val, e.pc);
        if (pos - el < 2) break;
        pos -= el;
        memcpy(line + pos, ent, el);
        shown++;
    }
    Debug::log("NEXT ev:%s%s", shown < s_nt_w ? " .." : "", line + pos);
}
#define NT_EV(k, r, v) next_trace_ev((k), (r), (v))
#else
#define NT_EV(k, r, v) ((void)0)
#endif

static void applyRomSelect();

void NextReg::reset(bool hard) {
    // The zxnext.vhd reset block (~4900-5111) runs on BOTH reset types — a
    // soft reset restores the nextreg defaults too (zx_go pins this with
    // TestResetRestoresDefaults). hw 2026-08-10: Alt-F11 after a 48K .tap
    // game used to leave the game's Layer2/sprite/alt-ROM latches standing —
    // NextZXOS rebooted into garbage and only the full MCU reboot recovered.
    //
    // Alt-ROM first: reset PROMOTES the staged nibble (bits 3:0) into the
    // active one (bits 7:4) — zxnext.vhd:2255, both reset types.
    uint8_t altStaged = reg[0x8C] & 0x0F;
    // Peripheral setup the BIOS writes before its soft reset into a
    // personality: zxnext.vhd's reset block leaves NR 0x05/0x06/0x08/0x09/0x0A
    // and the port-decode enables 0x82-0x85 alone apart from a few bits
    // (hotkeys, contention-disable, sprite tie; 0x82-0x85 only when 0x85 bit 7
    // asks for it). Zeroing them on a soft reset threw the BIOS setup away —
    // the divMMC automap enable among it.
    static const uint8_t kKeep[] = { 0x05, 0x06, 0x08, 0x09, 0x0A, 0x82, 0x83, 0x84, 0x85 };
    uint8_t kept[sizeof(kKeep)];
    for (unsigned i = 0; i < sizeof(kKeep); i++) kept[i] = reg[kKeep[i]];
    for (int i = 0; i < 256; i++) reg[i] = 0;
    selected = 0;
    reg[0x8C] = (uint8_t)((altStaged << 4) | altStaged);
    MemESP::altrom_rd  = (reg[0x8C] & 0x80) && !(reg[0x8C] & 0x40);
    MemESP::altrom_wr  = (reg[0x8C] & 0x80) && (reg[0x8C] & 0x40);
    if (hard) {
        // TBBLUE power-on: bootrom readable + config_mode = 1 — but only when
        // the boot chain is armed (TBBLUE.FW found on SD, see ESPectrum::setup).
        // The direct enNextZX boot never enters config mode: nothing would
        // exit it (boot.bin's first NR 0x03 write is the exit on real hw).
        MemESP::nextBootRomActive = (MemESP::nextBootRom != nullptr);
        MemESP::nextConfigMode = MemESP::nextBootRomActive;
        MemESP::nextConfigPage = 0;
        s_machineType = 0x03; s_machineTiming = 0x03;
        s_userDTLock = 0; s_reg03TopBit = 1;
        reg[0x8C] = 0;                 // power-on: alt-ROM fully off
        MemESP::altrom_rd = MemESP::altrom_wr = false;
        MemESP::altrom_sel = 0;
    }
    // Soft reset: bootrom/config flags survive (NextZXOS soft-resets between
    // boot stages with config mode already off; re-arming would loop the boot).
    portFF = 0;
    portFFIntDisable = false;
    applyTimexMode(0);   // leave Timex hi-res with the latch
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
    // Reset defaults pinned to the FPGA VHDL (zx_go VHDL_CONFORMANCE.md)
    if (hard) {
        reg[0x05] = 0x01; // periph1: 50Hz
        reg[0x06] = 0xA0; // periph2: hotkeys enabled
        reg[0x08] = 0x10; // periph3: AY enabled
        // DivMMC automap enable: power-on 0, set by the BIOS (TBBLUE.FW
        // PERIPH5 bit 4). The direct enNextZX boot has no BIOS to do it.
        reg[0x0A] = MemESP::nextBootRom ? 0x00 : 0x10;
        reg[0x82] = reg[0x83] = reg[0x84] = 0xFF;   // every internal port on
        reg[0x85] = 0x8F;
    } else {
        for (unsigned i = 0; i < sizeof(kKeep); i++) reg[kKeep[i]] = kept[i];
        reg[0x06] |= 0xA0;                // hotkey CPU speed / 50-60 Hz re-enabled
        reg[0x08] &= (uint8_t)~0x40;      // contention disable cleared
        reg[0x09] &= (uint8_t)~0x10;      // sprite tie cleared
        if (reg[0x85] & 0x80) {           // "reset the enables on soft reset"
            reg[0x82] = reg[0x83] = reg[0x84] = 0xFF;
            reg[0x85] = 0x8F;
        }
    }
    reg[0x12] = 0x08; // Layer 2 bank
    reg[0x13] = 0x0B; // Layer 2 shadow bank
    reg[0x42] = 0x07; // ULANext attribute format
    reg[0x6E] = 0x2C; // tilemap map base
    reg[0x6F] = 0x0C; // tilemap tiles base
    // divMMC programmable entry points (esxdos-compatible defaults):
    // 0xB8 = RST-trap mask (0x00/0x08/0x38), 0xB9 = "always" mask (0x0000
    // stays always-on), 0xBB = 0066/04C6/0562/3Dxx/off-region set
    reg[0x02] = 0x01; // read-back: bit0 = "last reset was soft" (VHDL default)
    reg[0xB8] = 0x83;
    reg[0xB9] = 0x01;
    reg[0xBA] = 0x00;
    reg[0xBB] = 0xCD;
    Debug::log("NextReg: programmable divMMC traps armed (B8=83 B9=01 BB=CD)");
    reg[0x14] = 0xE3; // global transparency colour
    reg[0x4A] = 0xE3; // fallback colour (VHDL reset value)
    reg[0x4B] = 0xE3; // sprite transparency index
    reg[0x4C] = 0x0F; // tilemap transparency index
    reg[0x03] = reg03Compose();
    reg[0x04] = MemESP::nextConfigPage;
    // ROM index / ROM3 / Alt-ROM half from the reset latches (7FFD/1FFD = 0)
    MemESP::romLatch = 0;
    applyRomSelect();
    MemESP::applyNextConfigWindow();
}

// Raster line for reg 0x1E/0x1F and the line interrupt. Line 0 is the first
// line of the pixel area, matching the Next convention. tstates run 2^m
// faster in turbo, so scale back to base-clock units first.
uint16_t NextReg::activeLine() {
    uint32_t ts = CPU::tstates >> ESPectrum::multiplicator;
    uint32_t line = ts / VIDEO::tStatesPerLine;
    // ULA paper starts 64 lines into the frame on 128K timing (63 border
    // lines + sync); the Next counts video lines from paper start. At 60 Hz
    // the top border is 26 lines shorter (NEXTVID::computeGeometry).
    const uint32_t paperStart = paperLine();
    uint32_t total = CPU::statesInFrame >> ESPectrum::multiplicator;
    uint32_t lines = total / VIDEO::tStatesPerLine;
    return (uint16_t)((line + lines - paperStart) % lines);
}

static void applyRomSelect();
uint8_t NextReg::machineType() { return s_machineType; }
bool NextReg::osTapeTrapArmed() {
    return (reg[0x0A] & 0x10) && (reg[0x83] & 0x01) && (reg[0xBB] & 0x20) &&
           (MemESP::next_rom_mask & 1) && MemESP::next_rom3;
}
static void applyBankC000();

// 28 MHz is beyond what the emulation core keeps up with today — the frame
// takes longer than its 20 ms budget and the machine crawls — so the EFFECTIVE
// speed is capped by Config::next_clk_cap (menu: CPU cap, default 14 MHz).
// reg[0x07] keeps the programmed value and the read-back reports the actual
// speed in bits 5:4, which is how real boards expose "asked for more than I
// can do". Same idea as TS-Conf's tsconf_clk_cap.
void NextReg::applySpeed() {
    uint8_t m = reg[0x07] & 0x03;
    if (m > Config::next_clk_cap) m = Config::next_clk_cap;
    uint8_t old = ESPectrum::multiplicator;
    if (old == m) return;
    // The switch happens MID-FRAME: everything measured in T-states (tstates,
    // statesInFrame, the NEXTVID line boundary) must move to the new clock
    // domain together, or the end-of-frame `tstates -= statesInFrame`
    // underflows and the INT window is never hit again (hw: NextZXOS set
    // 28MHz and froze on the next HALT with tstates = 0xFFFFFFFF).
    ESPectrum::multiplicator = m;
    if (m > old) CPU::tstates <<= (m - old);
    else         CPU::tstates >>= (old - m);
    CPU::updateStatesInFrame();
    NEXTVID::onClockChange();
}

void NextReg::write(uint8_t r, uint8_t v) {
    reg[r] = v;
#if NEXT_TRACE
    // Flood sources are counted, not recorded (palette / copper / sprite
    // streams run thousands of writes a second and would purge the ring).
    if (r == 0x40 || r == 0x41 || r == 0x44) s_nt_pal++;
    else if (r >= 0x60 && r <= 0x63) s_nt_cop++;
    else if ((r >= 0x34 && r <= 0x39) || (r >= 0x75 && r <= 0x79)) s_nt_spr++;
    else NT_EV('N', r, v);
#endif
    switch (r) {
        case 0x03: { // Machine type / display timing / config mode
            bool bootromWasActive = MemESP::nextBootRomActive;
            // machine_timing: only with bit7 set, user lock clear, bit3 clear
            if ((v & 0x80) && !s_userDTLock && !(v & 0x08)) {
                uint8_t t = (v >> 4) & 7;
                s_machineTiming = (t <= 1) ? 1 : (t <= 4 ? t : 3);
            }
            s_userDTLock ^= (v >> 3) & 1;   // XOR-toggles on every bit3=1 write
            // machine_type: updates only in config mode and only to 1..4
            if (MemESP::nextConfigMode) {
                uint8_t t = v & 7;
                if (t >= 1 && t <= 4) s_machineType = t;
            }
            applyRomSelect();   // the machine type decides the ROM / ROM3 rule
            // config_mode: 111 re-enters, other non-zero exits, 000 no change
            switch (v & 7) {
                case 0: break;
                case 7: MemESP::nextConfigMode = true;  break;
                default: MemESP::nextConfigMode = false; break;
            }
            s_reg03TopBit = (v >> 7) & 1;
            // The bootrom read overlay dies on the very first NR 0x03 write
            // of any value (zxnext.vhd:5122)
            if (bootromWasActive) MemESP::nextBootRomActive = false;
            MemESP::applyNextConfigWindow();
            reg[0x03] = reg03Compose();
            break;
        }
        case 0x04: // Config-mode RAMPAGE: 16K page at 0x0000-0x3FFF
            MemESP::nextConfigPage = v;
            MemESP::applyNextConfigWindow();
            break;
        case 0x02: // Reset request: bit1 = hard, bit0 = soft. NextZXOS uses a
                   // soft reset to restart into the freshly configured state
                   // at the end of each boot stage — latching only left the
                   // boot in an endless crash-restart loop (hw: white flash
                   // blinking). Deferred to the frame boundary.
            if (v & 0x02) resetPending = 2;
            else if (v & 0x01) resetPending = 1;
            if (v & 0x03) {
                // On hardware the OUT asserts /RESET at once: not one more
                // instruction runs. We reset at the frame boundary, so park the
                // CPU the way HALT does (halted + end the slice) with interrupts
                // off. Letting it run on was a real bug: TBBLUE.FW's video test
                // writes NR 02 = 2 after the user's Y and then, in the very next
                // loop iteration, re-writes its "VideoTest" resume signature for
                // the following mode (0x618F -> 0xD5C1) — the reboot found it and
                // started the test again, for ever (hw 2026-09-25).
                Z80::setHalted(true);
                Z80::setIFF1(false);
                Z80::setIFF2(false);
                CPU::stFrame = 0;
            }
            break;
        case 0x05: { // Peripheral 1: bit 2 = 50/60 Hz
            ESPectrum::target = (v & 0x04) ? 17067 : MICROS_PER_FRAME_128;
            CPU::updateStatesInFrame();
            break;
        }
        case 0x07: // CPU speed: 0=3.5, 1=7, 2=14, 3=28 MHz
            applySpeed();
            break;
        case 0x22: // Line interrupt control
            ulaIrqDisabled = v & 0x04;
            lineIrqEnabled = v & 0x02;
            lineIrqLine = (lineIrqLine & 0xFF) | ((v & 0x01) << 8);
            lineIrqReslice();
            break;
        case 0x23: // Line interrupt value LSB
            lineIrqLine = (lineIrqLine & 0x100) | v;
            lineIrqReslice();
            break;
        case 0x12: case 0x13: // Layer 2 active/shadow bank
            updateLayer2Window();
            break;
        case 0x69: { // Display Control 1: bit7 = Layer 2 enable (read by the
                     // renderer), bit6 = ULA shadow display — the same latch
                     // as port 7FFD bit 3 (hw: NextZXOS drew its boot screen
                     // into bank 7 via this and we kept showing blank bank 5)
            applyTimexMode(v & 0x3F);
            uint8_t vl = (v >> 6) & 1;
            if (MemESP::videoLatch != vl) {
                MemESP::videoLatch = vl;
                VIDEO::grmem = vl ? MemESP::ram[7].direct() : MemESP::ram[5].direct();
            }
            break;
        }
        case 0x18: case 0x19: case 0x1A: case 0x1B: // clip windows
            NEXTVID::clipWrite(r - 0x18, v);
            break;
        case 0x1C: // clip window index reset
            NEXTVID::clipIndexReset(v);
            break;
        case 0x34: // sprite number (own index unless lockstep, NR 0x09 bit 4)
            NEXTVID::spriteNrSelect(v);
            break;
        case 0x35: case 0x36: case 0x37: case 0x38: case 0x39:
            NEXTVID::spriteAttrDirect(r - 0x35, v);
            break;
        case 0x75: case 0x76: case 0x77: case 0x78: case 0x79:
            NEXTVID::spriteAttrDirect(r - 0x75, v, true);   // + sprite number
            break;
        case 0x40: NEXTVID::palIndex(v);   break;
        case 0x41: NEXTVID::palValue8(v);  break;
        case 0x43: NEXTVID::palControl(v); break;
        case 0x44: NEXTVID::palValue9(v);  break;
        case 0x50: case 0x51: case 0x52: case 0x53:
        case 0x54: case 0x55: case 0x56: case 0x57:
            MemESP::applyMMU(r - 0x50, v);
            break;
        case 0x8C: { // Alt-ROM: bit7 enable, bit6 write-only, bit5 lock ROM1,
                     // bit4 lock ROM0; bits 3:0 staged (promoted on soft reset)
            MemESP::altrom_rd = (v & 0x80) && !(v & 0x40);
            MemESP::altrom_wr = (v & 0x80) && (v & 0x40);
            applyRomSelect();   // lock bits move ROM / ROM3 / the Alt-ROM half
            break;
        }
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
        case 0x03: return reg03Compose();
        case 0x0E: return 0x03;  // core sub-minor: report 3.02.03 — the
                                  // current NextZXOS distro gates boot on
                                  // >= 3.02.03 (hw: 3.02.00 failed the check
                                  // at ROM PC=0E13 → 1.4s restart loop);
                                  // same stable version zx_go settled on
        case 0x07: { // bits 5:4 = actual speed, 1:0 = programmed
            uint8_t actual = reg[0x07] & 3;
            if (actual > Config::next_clk_cap) actual = Config::next_clk_cap;
            return (actual << 4) | (reg[0x07] & 3);
        }
        case 0x18: case 0x19: case 0x1A: case 0x1B: {
            // Read returns the CURRENT coordinate (index not advanced) —
            // zxnext.vhd read mux 5947-5975 via zx_go's conformance notes
            uint8_t w = r - 0x18;
            return NEXTVID::clip[w][NEXTVID::clipIdx[w] & 3];
        }
        case 0x1C: // packed clip indices read-back
            return (uint8_t)((NEXTVID::clipIdx[3] & 3) << 6) |
                   (uint8_t)((NEXTVID::clipIdx[2] & 3) << 4) |
                   (uint8_t)((NEXTVID::clipIdx[1] & 3) << 2) |
                   (uint8_t)(NEXTVID::clipIdx[0] & 3);
        case 0x41: return NEXTVID::palRead8();
        case 0x44: return NEXTVID::palRead9();
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

// Timex/SCLD port 0xFF. bit6 = ULA frame INT disable (consumed by
// Z80Ops::isActiveINT alongside nextreg 0x22 bit 2); bits 5:0 mirror into
// nextreg 0x69, which is what NEXTVID reads for the screen-mode bits.
// Screen-mode bits 5:0 (port FF and NR 0x69 are one latch): hand the Timex
// mode to VIDEO's hi-res machinery, which switches the driver in vblank.
static void applyTimexMode(uint8_t v) {
    NEXTVID::c1Drain();   // core1 reads the hi-res LUT live
    const uint8_t pm = VIDEO::timex_mode, pi = VIDEO::timex_hires_ink;
    VIDEO::timex_mode = v & 0x07;
    VIDEO::timex_hires_ink = (v >> 3) & 0x07;
    if ((VIDEO::timex_mode == 6) != (pm == 6))
        VIDEO::timexHiresRequest(VIDEO::timex_mode == 6);
    else if (VIDEO::timex_hires_ink != pi && VIDEO::timex_hires_live)
        VIDEO::timexHiresColour();
}

void NextReg::writePortFF(uint8_t v) {
    portFF = v;
    portFFIntDisable = (v & 0x40) != 0;
    reg[0x69] = (uint8_t)((reg[0x69] & 0xC0) | (v & 0x3F));
    applyTimexMode(v);
    NT_EV('F', 0xFF, v);
}

void NextReg::updateLayer2Window() {
    // The TBBLUE config window owns the write overlay while it is up;
    // applyNextConfigWindow() calls back here the moment it closes.
    if (MemESP::nextBootRomActive || MemESP::nextConfigMode) return;
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
    // zxnext.vhd "rom address computation": which 16K ROM, whether it is
    // "ROM3" (the 48K BASIC — gates the divMMC tape/RST entry points) and which
    // Alt-ROM half (alt_128_n: 1 = the 48K half) all follow the MACHINE TYPE
    // (nextreg 0x03, set in config mode), with the 0x8C lock bits overriding:
    //   48K       : ROM 0 always, ROM3 = 1
    //   +3 / Next : ROM = 1FFD.2:7FFD.4, ROM3 = both set
    //   128K/Pent : ROM = 0:7FFD.4,      ROM3 = 7FFD.4
    // NextZXOS's TAP Loader (48K/128K/Pentagon modes) runs 48 BASIC as ROM1 of
    // a 128K-type machine; testing "ROM index == 3" never armed its #056A tape
    // trap there and every .tap loaded in real time (hw 2026-10-07).
    const uint8_t a   = NextReg::reg[0x8C];
    const bool    l1  = a & 0x20, l0 = a & 0x10;       // lock ROM1 / lock ROM0
    const uint8_t r7  = MemESP::romLatch & 0x01;         // 7FFD bit 4
    const uint8_t r1  = (NextReg::port1FFD >> 2) & 0x01; // 1FFD bit 2
    const uint8_t mt  = NextReg::machineType();
    uint8_t rom; bool rom3, alt128n;
    if (mt <= 1) {                                       // 48K
        rom = 0; rom3 = true; alt128n = !(!l1 && l0);
    } else if (mt == 2 || mt == 4) {                     // 128K / Pentagon
        if (l1 || l0) { rom = l1 ? 1 : 0; rom3 = l1; alt128n = l1; }
        else          { rom = r7;         rom3 = r7; alt128n = r7; }
    } else {                                             // +3 / Next
        if (l1 || l0) { rom = (l1 ? 2 : 0) | (l0 ? 1 : 0); rom3 = l1 && l0; alt128n = l1; }
        else          { rom = (r1 << 1) | r7;              rom3 = r1 && r7; alt128n = r7; }
    }
    // Without the NextZXOS set only the two-ROM 128K fallback exists.
    if (!MemESP::nextRomBase) { rom &= 1; rom3 = rom == 1; }
    MemESP::romInUse = rom;
    MemESP::altrom_sel = alt128n ? 1 : 0;
    // divMMC automap "rom3" entry points: the Alt-ROM stands in for ROM3 when
    // it is the 48K half (sram_divmmc_automap_rom3_en).
    MemESP::next_rom3 = MemESP::altrom_rd ? alt128n : rom3;
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
    NT_EV('7', 0x7F, v);
    port7FFD = v;
    applyBankC000();
    uint8_t vl = (v >> 3) & 1;
    if (MemESP::videoLatch != vl) {
        MemESP::videoLatch = vl;
        VIDEO::grmem = vl ? MemESP::ram[7].direct() : MemESP::ram[5].direct();
    }
    reg[0x69] = (uint8_t)((reg[0x69] & ~0x40) | (vl << 6));
    MemESP::romLatch = (v >> 4) & 1;
    if (!(port1FFD & 0x01)) applyRomSelect();
    if (v & 0x20) MemESP::pagingLock = 1;
}

void NextReg::writeDFFD(uint8_t v) {
    NT_EV('D', 0xDF, v);
    portDFFD = v & 0x0F;
    applyBankC000();
}

void NextReg::write1FFD(uint8_t v) {
    NT_EV('1', 0x1F, v);
    uint8_t prev = port1FFD;
    port1FFD = v;
    if (v & 0x01) {
        // +3 special all-RAM configurations: an overlay on top of the MMU —
        // the MMU registers themselves are NOT modified (hw: routing this
        // through applyMMU corrupted mmu[] and the exit path then force-reset
        // slots 2-5 to the 128K default, yanking the OS's own mapping out
        // from under it on every NEXTREG 0x8E ROM-swap trampoline — restart
        // loop right after the loader's first cross-ROM call).
        static const uint8_t maps[4][4] = {
            { 0, 1, 2, 3 }, { 4, 5, 6, 7 }, { 4, 5, 6, 3 }, { 4, 7, 6, 3 }
        };
        const uint8_t* m = maps[(v >> 1) & 3];
        for (int s = 0; s < 4; s++) {
            uint8_t page = (uint8_t)(m[s] * 2);
            uint8_t* p0 = (page < MemESP::NEXT_PAGES) ? MemESP::nextRamPtr[page] : nullptr;
            uint8_t* p1 = (page + 1 < MemESP::NEXT_PAGES) ? MemESP::nextRamPtr[page + 1] : nullptr;
            if (p0) { MemESP::ramCurrent[s * 2] = p0; MemESP::ramContended[s * 2] = false; }
            if (p1) { MemESP::ramCurrent[s * 2 + 1] = p1; MemESP::ramContended[s * 2 + 1] = false; }
        }
        MemESP::next_rom_mask = 0;   // all-RAM: page 0 is writable RAM
    } else {
        if (prev & 0x01) {
            // Leaving all-RAM: restore EXACTLY what the MMU registers say
            for (int s = 0; s < 8; s++)
                MemESP::applyMMU(s, MemESP::mmu[s]);
        }
        // Normal mode: 1FFD only contributes the ROM select high bit
        applyRomSelect();
    }
}



// stFrame 0 means "HALT" to CPU::loop, so the slice ends one T later at the
// earliest; a slice already ending sooner is left alone.
void NextReg::lineIrqReslice() {
    const uint32_t cut = CPU::tstates + 1;
    if (CPU::stFrame > cut) CPU::stFrame = cut;
}
