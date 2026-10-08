// ZX Spectrum Next interrupt controller + CTC — see NextInt.h.
#include "NextInt.h"
#include "NextReg.h"
#include "app/ESPectrum.h"
#include "speccy/z80/CPU.h"
#include "speccy/z80/z80.h"
#include "speccy/video/Video.h"
#include <cstdint>

#if NEXT_TRACE
// NEXT cpu: trace (NextReg.cpp): line pulses raised / already over when seen /
// by how many 3.5 MHz T, INT acknowledges, the line event's frame position.
volatile uint32_t g_li_raise = 0, g_li_late = 0, g_li_late_by = 0, g_int_ack = 0;
#endif
namespace NextInt {
uint8_t ex = 0;
bool retnArmed = false;
}

namespace {

enum { D_LINE = 0, D_CTC0 = 3, D_ULA = 11, D_NUM = 14 };
enum : uint8_t { S_0, S_REQ, S_ACK, S_ISR };              // im2_device.vhd
enum : uint8_t { C_RESET, C_RESET_TC, C_TRIGGER, C_RUN, C_RUN_TC };   // ctc_chan.vhd

constexpr int64_t kNever = INT64_MAX / 4;

uint8_t  s_st[D_NUM];      // im2_device state per source
uint16_t s_req;            // im2_int_req (hardware IM2 pending, cleared at RETI)
uint16_t s_status;         // int_status (NR 0x20 / 0xC8-0xCA), cleared by writes
uint8_t  s_c0;             // NR 0xC0: 7:5 vector, 3 stackless NMI, 0 hw IM2
uint8_t  s_c4hi;           // NR 0xC4 bit 7 (expansion bus INT enable, stored)
bool     s_lineEn;
uint16_t s_lineNum;        // NR 0x22 bit 0 : NR 0x23
int64_t  s_linePos28 = -1; // frame-relative 28 MHz position of the line event, -1 none

int64_t  s_base28;         // absolute 28 MHz clock at frame-relative T 0
int64_t  s_last28;         // absolute time of the last sync
int64_t  s_pulse = -kNever;   // start of the newest pulse (pulse mode)
bool     s_unqPulse;       // an NR 0x20 pulse keeps ex up until it ends

struct Ctc {
    uint8_t  st, cw;       // state, control word bits 7..3
    uint16_t tc;           // time constant 1..256
    uint16_t cnt;          // down counter 1..256 (timer: as of ref)
    int64_t  run0;         // entry into RUN = prescaler phase origin
    int64_t  ref;          // timer: time cnt refers to
};
Ctc s_ctc[4];

struct Stream { int64_t first, period; int64_t n; };   // n ZC/TO pulses, first, first+period, ...

inline uint32_t f28() { return 8u >> ESPectrum::multiplicator; }
inline int64_t now28() { return s_base28 + (int64_t)CPU::tstates * f28(); }
inline bool im2Hw() { return s_c0 & 1; }
inline bool cpuIm2() { return Z80::getIM() == Z80::IM2; }

// zxnext.vhd 2030: pulse_count_end = count(5) and (48K or +3 or count(2)) —
// 32 CPU cycles in 48K/+3 timing, 36 in 128K/Pentagon.
inline int64_t pulseLen28() {
    const uint8_t t = NextReg::machineTiming();   // 1 48K, 2 128K, 3 +3, 4 Pentagon
    return (int64_t)((t == 1 || t == 3) ? 32 : 36) * f28();
}

inline int64_t fdiv(int64_t a, int64_t b) { return a >= 0 ? a / b : -((-a + b - 1) / b); }

// The classic frame-INT window, exactly as Z80Ops::isActiveINT tests it.
bool classicUla() {
    if (NextReg::portFFIntDisable) return false;
    int32_t tmp = (int32_t)CPU::tstates + CPU::latetiming;
    if (tmp >= (int32_t)CPU::statesInFrame) tmp -= CPU::statesInFrame;
    return tmp >= CPU::IntStart && tmp < CPU::IntEnd;
}

void exRecalc() {
    bool ctcInt = false;
    for (auto& c : s_ctc) ctcInt |= (c.cw & 0x80) != 0;
    const uint8_t was = NextInt::ex;
    NextInt::ex = (s_lineEn || im2Hw() || ctcInt || s_unqPulse) ? 1 : 0;
    if (NextInt::ex && !was) NextInt::wake();
}

// A source fired at absolute time t. `pulse` false = the ULA frame event, whose
// pulse is the classic window.
void raise(int d, int64_t t, bool pulse) {
    s_status |= (uint16_t)(1u << d);
    if (im2Hw()) {
        s_req |= (uint16_t)(1u << d);
        if (s_st[d] == S_0) s_st[d] = S_REQ;
    } else if (pulse && t > s_pulse) {
        s_pulse = t;
    }
}

// --- CTC (ctc_chan.vhd). Prescaler: p_count counts 28 MHz cycles from RUN entry,
// a tick every 16 (or 256) cycles at p_count = 15 (255); the counter reloads from
// the time constant at ZC/TO. Counter mode counts the previous channel's ZC/TO
// (channel 0 counts channel 3's). ---
inline bool running(const Ctc& c) { return c.st == C_RUN || c.st == C_RUN_TC; }
inline bool timer(const Ctc& c) { return !(c.cw & 0x40); }
inline int64_t presc(const Ctc& c) { return (c.cw & 0x20) ? 256 : 16; }
inline int64_t ticksTo(const Ctc& c, int64_t t) { return fdiv(t - c.run0 + 1, presc(c)); }

void advanceTimer(Ctc& c, int64_t t, Stream& s) {
    s.n = 0;
    if (t <= c.ref) return;
    const int64_t P = presc(c);
    const int64_t k0 = ticksTo(c, c.ref);
    const int64_t n = ticksTo(c, t) - k0;
    if (n < c.cnt) { c.cnt = (uint16_t)(c.cnt - n); c.ref = t; return; }
    s.first = c.run0 + (k0 + c.cnt) * P - 1;
    const int64_t rem = n - c.cnt;
    s.n = 1 + rem / c.tc;
    s.period = (int64_t)c.tc * P;
    c.cnt = (uint16_t)(c.tc - rem % c.tc);
    c.ref = t;
}

void counterFrom(Ctc& c, const Stream& src, Stream& s) {
    s.n = 0;
    if (src.n <= 0) return;
    if (src.n < c.cnt) { c.cnt = (uint16_t)(c.cnt - src.n); return; }
    s.first = src.first + (int64_t)(c.cnt - 1) * src.period;
    const int64_t rem = src.n - c.cnt;
    s.n = 1 + rem / c.tc;
    s.period = (int64_t)c.tc * src.period;
    c.cnt = (uint16_t)(c.tc - rem % c.tc);
}

void emit(int ch, const Stream& s) {
    if (s.n <= 0) return;
    const int d = D_CTC0 + ch;
    s_status |= (uint16_t)(1u << d);
    if (!(s_ctc[ch].cw & 0x80)) return;
    raise(d, s.first + (s.n - 1) * s.period, true);
}

void propagate(int i, Stream s, bool* done, int64_t t) {
    int j = (i + 1) & 3;
    for (int k = 0; k < 3 && s.n > 0; k++, j = (j + 1) & 3) {
        if (done[j]) break;
        Ctc& c = s_ctc[j];
        Stream o{0, 0, 0};
        if (running(c) && !timer(c)) {
            counterFrom(c, s, o);
        } else if (c.st == C_TRIGGER && timer(c) && (c.cw & 0x08)) {
            c.st = C_RUN; c.run0 = c.ref = s.first; c.cnt = c.tc;
            advanceTimer(c, t, o);
        } else break;
        done[j] = true;
        emit(j, o);
        s = o;
    }
}

void ctcAdvance(int64_t t) {
    bool done[4] = {false, false, false, false};
    for (int i = 0; i < 4; i++) {
        Ctc& c = s_ctc[i];
        if (done[i] || !running(c) || !timer(c)) continue;
        Stream s;
        advanceTimer(c, t, s);
        done[i] = true;
        emit(i, s);
        propagate(i, s, done, t);
    }
}

// k-th ZC/TO of channel ch from the synced state, kNever if none can come.
int64_t ctcNextZc(int ch, int64_t k, int depth) {
    if (depth > 4) return kNever;
    const Ctc& c = s_ctc[ch];
    if (running(c) && timer(c))
        return c.run0 + (ticksTo(c, c.ref) + c.cnt + (k - 1) * c.tc) * presc(c) - 1;
    const int src = (ch + 3) & 3;
    if (running(c))
        return ctcNextZc(src, c.cnt + (k - 1) * c.tc, depth + 1);
    if (c.st == C_TRIGGER && timer(c) && (c.cw & 0x08)) {
        const int64_t s = ctcNextZc(src, 1, depth + 1);
        return s >= kNever ? kNever : s + (int64_t)c.tc * k * presc(c) - 1;
    }
    return kNever;
}

void sync() {
    const int64_t now = now28();
    if (now <= s_last28) {
        if (now < s_last28) s_last28 = now;   // CPU::reset moved tstates back
        return;
    }
    const int64_t relPrev = s_last28 - s_base28, relNow = now - s_base28;
    if (s_lineEn && s_linePos28 >= 0 && s_linePos28 > relPrev && s_linePos28 <= relNow) {
        raise(D_LINE, s_base28 + s_linePos28, true);
#if NEXT_TRACE
        g_li_raise++;
        // A pulse already over by the time the loop saw it: the CPU could not
        // take it (bus held by a DMA, or a slice that ran past the event).
        // With interrupts disabled nothing stops the slice there, and the
        // pulse is lost on the board too: not counted (RAMTEST runs DI).
        if (!im2Hw() && Z80::isIFF1() && now >= s_base28 + s_linePos28 + pulseLen28()) {
            g_li_late++;
            g_li_late_by = (uint32_t)((now - s_base28 - s_linePos28) / 8);
        }
#endif
    }
    for (auto& c : s_ctc)
        if (c.st >= C_TRIGGER) { ctcAdvance(now); break; }
    s_last28 = now;
    if (s_unqPulse && now >= s_pulse + pulseLen28()) { s_unqPulse = false; exRecalc(); }
}

bool pulseUp() { return now28() - s_pulse < pulseLen28() && now28() >= s_pulse; }

// The source currently pulling /INT in hardware IM2 mode, -1 none. Daisy chain:
// a source in service (ACK/ISR) blocks every lower one; one requesting blocks
// them too (im2_device: ieo = iei and reti_decode in S_REQ).
int presented() {
    for (int d = 0; d < D_NUM; d++) {
        if (s_st[d] == S_0 && ((s_req >> d) & 1)) s_st[d] = S_REQ;   // im2_device S_0 -> S_REQ on i_int_req
        if (s_st[d] == S_REQ) return d;
        if (s_st[d] != S_0) return -1;
    }
    return -1;
}

// NR 0x22 bit 7: the pulse generator's own /INT (not pulse_int_n), i.e. the
// pulse sources — hardware IM2 requests do not show there.
bool pulseLine() {
    if (im2Hw()) return !cpuIm2() && classicUla();
    return classicUla() || pulseUp();
}

void ctcClockEdge(int ch, int64_t now) {
    Ctc& c = s_ctc[ch];
    if (running(c) && !timer(c)) {
        if (--c.cnt == 0) {
            c.cnt = c.tc;
            bool done[4] = {false, false, false, false};
            done[ch] = true;
            const Stream s{now, 1, 1};
            emit(ch, s);
            propagate(ch, s, done, now);
        }
    } else if (c.st == C_TRIGGER && timer(c) && (c.cw & 0x08)) {
        c.st = C_RUN; c.run0 = c.ref = now; c.cnt = c.tc;
    }
}

} // namespace

namespace NextInt {

void reset(bool) {
    // The board resets the whole interrupt block and the CTC on soft and hard
    // reset alike (zxnext_top_issue5.vhd:880 reset <= hard or soft;
    // zxnext.vhd 5092-5105 for the NR 0xC0-0xCE defaults).
    for (auto& s : s_st) s = S_0;
    s_req = s_status = 0;
    s_c0 = 0; s_c4hi = 0x80;
    s_lineEn = false; s_lineNum = 0;
    retnArmed = false;
    s_unqPulse = false;
    s_pulse = -kNever;
    for (auto& c : s_ctc) { c.st = C_RESET; c.cw = 0; c.tc = 256; c.cnt = 256; c.run0 = c.ref = 0; }
    for (uint8_t r = 0xC0; r <= 0xCE; r++) NextReg::reg[r] = 0;
    NextReg::reg[0xC4] = 0x80;
    s_last28 = now28();
    lineRecalc();
    exRecalc();
}

void lineRecalc() {
    s_linePos28 = -1;
    const uint32_t tpl = VIDEO::tStatesPerLine;
    if (!tpl) return;
    const uint32_t lines = (CPU::statesInFrame >> ESPectrum::multiplicator) / tpl;
    if (!lines || s_lineNum >= lines) return;   // cvc never reaches it
    // zxula_timing.vhd 561-585: the line interrupt fires at hc_ula = 255 (the
    // right edge of the paper, 12 pixels ahead of the display) on the line
    // BEFORE the programmed one, counted by the copper counter cvc, which NR 0x64
    // offsets at the paper top. Rows are counted from the frame INT exactly as
    // the renderer does (NextReg::paperLine); the column is the VHDL's distance
    // from the ULA interrupt (c_int_h) to hc_ula 255, in T-states.
    const uint32_t off = NextReg::reg[0x64] % lines;
    const uint32_t rel = (s_lineNum + 2 * lines - 1 - off) % lines;
    const uint32_t row = (NextReg::paperLine() + rel) % lines;
    const uint8_t t = NextReg::machineTiming();
    const uint32_t hoff = t == 1 ? 128 : t == 3 ? 127 : t == 4 ? 190 : 126;
    s_linePos28 = ((int64_t)row * tpl + hoff) * 8;
}

bool lineEnabled() { return s_lineEn; }
int32_t linePosT() { return s_linePos28 < 0 ? -1 : (int32_t)(s_linePos28 / 8); }
uint16_t lineNumber() { return s_lineNum; }

void wake() {
    if (CPU::stFrame > CPU::tstates + 1) CPU::stFrame = CPU::tstates + 1;
}

bool intLine() {
    sync();
    if (im2Hw()) return cpuIm2() ? presented() >= 0 : classicUla();
    return classicUla() || pulseUp();
}

uint32_t nextEventT(uint32_t limit) {
    sync();
    const int64_t now = s_last28;
    int64_t best = kNever;
    if (s_lineEn && s_linePos28 >= 0 && s_base28 + s_linePos28 > now) best = s_base28 + s_linePos28;
    for (int ch = 0; ch < 4; ch++) {
        if (!(s_ctc[ch].cw & 0x80)) continue;
        const int64_t t = ctcNextZc(ch, 1, 0);
        if (t > now && t < best) best = t;
    }
    const uint32_t lo = CPU::tstates + 1;
    if (best >= kNever) return limit > lo ? limit : lo;
    const uint32_t f = f28();
    const int64_t rel = best - s_base28;
    const int64_t T = (rel + f - 1) / f;
    if (T >= (int64_t)limit) return limit > lo ? limit : lo;
    return (uint32_t)T > lo ? (uint32_t)T : lo;
}

uint8_t ack() {
    sync();
#if NEXT_TRACE
    g_int_ack++;
#endif
    if (!im2Hw() || !cpuIm2()) return 0xFF;
    const int d = presented();
    if (d < 0) return 0xFF;
    s_st[d] = S_ISR;   // S_ACK, then S_ISR as soon as /M1 rises
    return (uint8_t)((s_c0 & 0xE0) | (d << 1));
}

void reti() {
    if (!im2Hw() || !cpuIm2()) return;
    sync();
    for (int d = 0; d < D_NUM; d++) {
        if (s_st[d] == S_ACK || s_st[d] == S_ISR) {
            s_st[d] = S_0;
            s_req &= (uint16_t)~(1u << d);
            break;
        }
    }
    wake();
}

void frameEnd() {
    sync();
    s_base28 += (int64_t)CPU::statesInFrame * f28();
    lineRecalc();
    // The ULA frame interrupt of the new frame: its status bit, and in hardware
    // IM2 mode its latched request (in pulse mode the pulse is the classic window).
    if (!NextReg::portFFIntDisable) raise(D_ULA, s_base28, false);
}

static void setUlaDisable(bool dis) {
    NextReg::portFFIntDisable = dis;
    NextReg::portFF = (uint8_t)((NextReg::portFF & ~0x40) | (dis ? 0x40 : 0));
}

void nrWrite(uint8_t r, uint8_t v) {
    sync();
    switch (r) {
        case 0x20:   // generate interrupts in software (im2_int_unq): status, IM2 latch or a pulse
            if (v & 0x80) raise(D_LINE, s_last28, true);
            if (v & 0x40) raise(D_ULA, s_last28, true);
            for (int ch = 0; ch < 4; ch++) if (v & (1 << ch)) raise(D_CTC0 + ch, s_last28, true);
            if ((v & 0xCF) && !im2Hw()) s_unqPulse = true;
            break;
        case 0x22:   // bit 2 = port #FF bit 6 (one latch), bit 1 line enable, bit 0 line MSB
            setUlaDisable(v & 0x04);
            s_lineEn = v & 0x02;
            s_lineNum = (uint16_t)((s_lineNum & 0xFF) | ((v & 0x01) << 8));
            lineRecalc();
            break;
        case 0x23:
            s_lineNum = (uint16_t)((s_lineNum & 0x100) | v);
            lineRecalc();
            break;
        case 0x64:
            lineRecalc();
            break;
        case 0xC0:
            s_c0 = v & 0xE9;
            if (!(v & 0x01)) { for (auto& s : s_st) s = S_0; s_req = 0; }   // im2_reset_n
            if (!(v & 0x08)) retnArmed = false;
            break;
        case 0xC4:   // bit 7 expansion bus, bit 1 line (= NR 0x22 bit 1), bit 0 ULA (= NOT #FF bit 6)
            s_c4hi = v & 0x80;
            s_lineEn = v & 0x02;
            setUlaDisable(!(v & 0x01));
            lineRecalc();
            break;
        case 0xC5:   // CTC channel interrupt enables = control word bit 7
            for (int ch = 0; ch < 4; ch++)
                s_ctc[ch].cw = (uint8_t)((s_ctc[ch].cw & 0x7F) | ((v >> ch) & 1 ? 0x80 : 0));
            break;
        case 0xC8:
            if (v & 0x02) s_status &= (uint16_t)~(1u << D_LINE);
            if (v & 0x01) s_status &= (uint16_t)~(1u << D_ULA);
            break;
        case 0xC9:
            s_status &= (uint16_t)~((uint16_t)v << D_CTC0);
            break;
        // 0xC2/0xC3 (NMI return address), 0xC6/0xCA (UART), 0xCC-0xCE (DMA
        // interruption enables) live in NextReg::reg.
        default: break;
    }
    exRecalc();
}

bool nrRead(uint8_t r, uint8_t& out) {
    switch (r) {
        case 0x20: {
            sync();
            const uint16_t s = s_status | s_req;
            out = (uint8_t)(((s >> D_LINE) & 1) << 7 | ((s >> D_ULA) & 1) << 6 | ((s >> D_CTC0) & 0x0F));
            return true;
        }
        case 0x22:
            sync();
            out = (uint8_t)((pulseLine() ? 0x80 : 0) | (NextReg::portFFIntDisable ? 0x04 : 0) |
                            (s_lineEn ? 0x02 : 0) | (s_lineNum >> 8));
            return true;
        case 0x23: out = (uint8_t)s_lineNum; return true;
        case 0xC0: {
            const Z80::IntMode m = Z80::getIM();
            const uint8_t im = m == Z80::IM2 ? 2 : m == Z80::IM1 ? 1 : 0;
            out = (uint8_t)((s_c0 & 0xE9) | (im << 1));
            return true;
        }
        case 0xC4: out = (uint8_t)(s_c4hi | (s_lineEn ? 0x02 : 0) | (NextReg::portFFIntDisable ? 0 : 0x01)); return true;
        case 0xC5: {
            uint8_t e = 0;
            for (int ch = 0; ch < 4; ch++) if (s_ctc[ch].cw & 0x80) e |= (uint8_t)(1 << ch);
            out = e;
            return true;
        }
        case 0xC6: case 0xCE: out = NextReg::reg[r] & 0x77; return true;
        case 0xCC: out = NextReg::reg[r] & 0x83; return true;
        case 0xC8: {
            sync();
            const uint16_t s = s_status | s_req;
            out = (uint8_t)(((s >> D_LINE) & 1) << 1 | ((s >> D_ULA) & 1));
            return true;
        }
        case 0xC9: {
            sync();
            out = (uint8_t)(((s_status | s_req) >> D_CTC0) & 0x0F);
            return true;
        }
        case 0xCA: out = 0; return true;   // no UART
        default: return false;
    }
}

uint8_t ctcRead(uint8_t ch) {
    if (ch > 3) return 0;   // channels 4..7 are not instantiated (zxnext.vhd 4064)
    sync();
    const Ctc& c = s_ctc[ch];
    return (uint8_t)((c.st == C_RUN || c.st == C_RUN_TC) ? c.cnt : c.tc);
}

void ctcWrite(uint8_t ch, uint8_t v) {
    if (ch > 3) return;
    sync();
    const int64_t now = s_last28;
    Ctc& c = s_ctc[ch];
    if (c.st == C_RESET_TC || c.st == C_RUN_TC) {          // time constant
        c.tc = v ? v : 256;
        if (c.st == C_RESET_TC) {
            c.cnt = c.tc;
            if (timer(c) && (c.cw & 0x08)) c.st = C_TRIGGER;
            else { c.st = C_RUN; c.run0 = c.ref = now; }
        } else {
            c.st = C_RUN;                                   // used at the next reload
        }
    } else if (v & 0x01) {                                  // control word
        const bool edgeChg = ((v ^ c.cw) & 0x10) != 0;
        c.cw = v & 0xF8;
        c.ref = now;
        if (v & 0x02) {                                     // software reset
            c.st = (v & 0x04) ? C_RESET_TC : C_RESET;
            c.cnt = c.tc;
        } else {
            if (v & 0x04) {
                if (c.st == C_RESET || c.st == C_TRIGGER) c.st = C_RESET_TC;
                else if (c.st == C_RUN) c.st = C_RUN_TC;
            }
            if (edgeChg) ctcClockEdge(ch, now);             // counts as a clock edge
        }
    }
    // bit 0 = 0: interrupt vector — the Next takes vectors from NR 0xC0 (open in the VHDL)
    exRecalc();
}

bool nmiStackless(uint16_t pc) {
    NextReg::reg[0xC2] = (uint8_t)pc;
    NextReg::reg[0xC3] = (uint8_t)(pc >> 8);
    if (!(s_c0 & 0x08)) return false;
    retnArmed = true;
    return true;
}

uint16_t stacklessRetn() {
    retnArmed = false;
    return (uint16_t)(NextReg::reg[0xC2] | (NextReg::reg[0xC3] << 8));
}

}
