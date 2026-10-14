// pico-speccy — Description block content (see UiDesc.h).

#include "UiDesc.h"

#include <stdio.h>
#include <string.h>

#include "UiStage.h"
#include "app/Config.h"

namespace nm {

static int addLine(DescLine* out, int n, int max, uint8_t kind, const char* label,
                   const char* value, uint8_t state) {
    if (n >= max) return n;
    out[n].kind  = kind;
    out[n].state = state;
    snprintf(out[n].label, sizeof(out[n].label), "%s", label);
    snprintf(out[n].value, sizeof(out[n].value), "%s", value ? value : "");
    return n + 1;
}

// Subheader: the parameter and its current value ("now ..."), coloured by `st`.
static int sub(DescLine* out, int n, int max, const char* name, const char* now, uint8_t st) {
    return addLine(out, n, max, DESC_SUB, name, now, st);
}
// Rule: requirement + the condition it holds under ("" = always); `applies` greys
// the rules that are not the one in force.
static int rule(DescLine* out, int n, int max, const char* req, const char* cond,
                bool applies = true) {
    return addLine(out, n, max, DESC_RULE, req, cond, applies ? DESC_PLAIN : DESC_DIM);
}

static void mhzNow(char (&buf)[20], unsigned mhz) { snprintf(buf, sizeof(buf), "now %u MHz", mhz); }

// CPU >= minMhz, against the STAGED Overclock pick, so the block follows an edit made
// in this session before it is applied.
static int needCpu(DescLine* out, int n, int max, unsigned minMhz) {
    const unsigned mhz = (unsigned)Stage::get(SET_CPU_MHZ);
    char now[20], req[20];
    mhzNow(now, mhz);
    snprintf(req, sizeof(req), ">=%u MHz", minMhz);
    n = sub(out, n, max, "CPU", now, mhz >= minMhz ? DESC_OK : DESC_BAD);
    return rule(out, n, max, req, "");
}

static int describeMachine(int32_t v, DescLine* out, int max) {
    switch (v) {
        // ZX Evolution: TS-Conf (14 MHz Z80 + whole-line video) and BaseConf (14 MHz
        // ATM manager) both overrun the frame at the lower clocks.
        case NM_MACH(A_ATM,    R_EVO_BASE):
        case NM_MACH(A_TSCONF, R_TSCONF):
        case NM_MACH(A_TSCONF, R_TSCONF_GLUK):
            return needCpu(out, 0, max, 504);
        default: return 0;
    }
}

// General Sound: the GS-Z80 runs on core1 beside the video ISR, so how fast it can be
// clocked without starving depends on the CPU clock. Both rules are listed, the one
// in force at the staged CPU clock judges the staged GS clock.
static int gsClockRules(DescLine* out, int n, int max) {
    static const uint8_t kGsMhz[] = { 12, 13, 14, 20, 24 };   // Config::gs_clock index
    const unsigned cpu = (unsigned)Stage::get(SET_CPU_MHZ);
    const unsigned gi  = (unsigned)Stage::get(SET_GS_CLOCK);
    const unsigned gs  = gi < sizeof(kGsMhz) ? kGsMhz[gi] : 0;
    const bool fast = cpu >= 504;
    const unsigned hi = fast ? 20 : 14;
    char now[20];
    mhzNow(now, gs);
    n = sub (out, n, max, "GS clock", now, (gs >= 12 && gs <= hi) ? DESC_OK : DESC_BAD);
    n = rule(out, n, max, "12-14 MHz", "CPU <504",  !fast);
    n = rule(out, n, max, "12-20 MHz", "CPU >=504",  fast);
    return n;
}

static int describeGs(int32_t v, DescLine* out, int max) {
    switch (v) {
        case 1:  return gsClockRules(out, 0, max);   // classic General Sound
        case 2:  return needCpu(out, 0, max, 504);   // NeoGS (clock is always Auto)
        default: return 0;
    }
}

// V-Sync paces one emulated frame per DISPLAY frame, so the emulation runs at the
// display's refresh: right on a 50 Hz mode, ~20% fast on 60 Hz.
static int describeVsync(int32_t v, DescLine* out, int max) {
    if (v != 1) return 0;
    const unsigned hz = stagedVideoHz();
    char now[20];
    snprintf(now, sizeof(now), "now %u Hz", hz);
    int n = sub (out, 0, max, "Video mode", now, hz == 50 ? DESC_OK : DESC_BAD);
    n = rule(out, n, max, "50 Hz", "normal speed", hz == 50);
    n = rule(out, n, max, "60 Hz", "runs faster",  hz != 50);
    return n;
}

// The machine the commit will end on: the staged pick, else the running one.
static void stagedMachine(ArchIdx& a, RomsetIdx& r) {
    const int32_t m = Stage::get(SET_MACHINE);
    if (m >= 0) { a = (ArchIdx)((m >> 8) & 0xFF); r = (RomsetIdx)(m & 0xFF); }
    else        { a = Config::arch; r = Config::romSet; }
}

// Subheader "Machine  now <short name>", red when `bad`.
static int machineSub(DescLine* out, int n, int max, ArchIdx a, RomsetIdx r, bool bad) {
    const char* fam = nullptr; const char* sh = nullptr;
    char now[20];
    if (machineMenuNameFor(a, r, fam, sh)) snprintf(now, sizeof(now), "now %s", sh);
    else                                    snprintf(now, sizeof(now), "now ?");
    return sub(out, n, max, "Machine", now, bad ? DESC_BAD : DESC_OK);
}

// Machines where Timex Gfx mode is forced off (UiStage resolveConstraints,
// MachineSwitch and the CPU::reset backstop): port #FF / the screen belong to the
// machine itself there.
static int timexMachines(DescLine* out, int max) {
    ArchIdx a; RomsetIdx r;
    stagedMachine(a, r);
    const bool tsAtm = (a == A_TSCONF || a == A_ATM);
    const bool profi = (a == A_PROFI || a == A_KARABAS);
    const bool other = isPlus3Romset(r) || isScorpGmxRomset(r) ||
                       r == R_48K_BY || r == R_128K_BY || r == R_128K_BY_GLUK;
    int n = machineSub(out, 0, max, a, r, tsAtm || profi || other);
    n = rule(out, n, max, "not TS-Conf, ATM",   "", tsAtm);
    n = rule(out, n, max, "not Profi, Karabas", "", profi);
    return rule(out, n, max, "not +3, Byte, GMX", "", other);
}

// SAA1099: off on the Timex machines (their ULA is the SCLD on #FF), and it does not
// work on TS-Conf (owner, 2026-10-14) — the menu does not enforce that one yet.
static int saaMachines(DescLine* out, int max) {
    ArchIdx a; RomsetIdx r;
    stagedMachine(a, r);
    const bool ts = (a == A_TSCONF), tx = isTimexRomset(r);
    int n = machineSub(out, 0, max, a, r, ts || tx);
    n = rule(out, n, max, "not TS-Conf",       "", ts);
    return rule(out, n, max, "not TC2048/TC2068", "", tx);
}

// Timex Gfx mode and the SAA1099 both decode port #FF, so only one of them can be on
// (resolveConstraints turns the other off). Each row names the other one.
static int portClash(DescLine* out, int n, int max, const char* other, uint16_t otherSetting) {
    const bool on = Stage::get(otherSetting) != 0;
    n = sub (out, n, max, other, on ? "now On" : "now Off", on ? DESC_BAD : DESC_OK);
    return rule(out, n, max, "Off", "port #FF");
}

// Overclock: every row there (CPU, VREG, flash, PSRAM) is applied at the next boot,
// and a pick the chip cannot hold leaves it unable to start.
static int overclockWarn(DescLine* out, int max) {
    int n = sub (out, 0, max, "Warning", "", DESC_BAD);    // no value: the name goes red
    n = rule(out, n, max, "chip may not start", "");
    return rule(out, n, max, "after reboot", "");
}

int describe(const Node& n, int32_t value, DescLine* out, int max) {
    if (n.kind != K_RADIO && n.kind != K_BOOL) return 0;
    switch (n.setting) {
        case SET_MACHINE: return describeMachine(value, out, max);
        case SET_GS_MODE: return describeGs(value, out, max);
        case SET_VSYNC:   return describeVsync(value, out, max);
        case SET_CPU_MHZ:
        case SET_VREG:
        case SET_FLASH_FREQ:
        case SET_PSRAM_FREQ: return overclockWarn(out, max);
        case SET_TIMEX:
            return value ? portClash(out, timexMachines(out, max), max, "SAA1099", SET_SAA1099) : 0;
        case SET_SAA1099:
            return value ? portClash(out, saaMachines(out, max), max, "Timex Gfx mode", SET_TIMEX) : 0;
        // The Clock row: the same rules, judged against the staged clock.
        case SET_GS_CLOCK: return gsClockRules(out, 0, max);
        default:          return 0;
    }
}

} // namespace nm
