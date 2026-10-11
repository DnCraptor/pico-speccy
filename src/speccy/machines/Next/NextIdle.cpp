// ZX Next memory-poll idle loop fast-forward — see NextIdle.h.
#include "NextIdle.h"
#include "speccy/machines/Next/NextVideo.h"
#include "speccy/z80/z80.h"
#include "speccy/z80/CPU.h"
#include "speccy/core/MemESP.h"
#include "speccy/core/Rzx.h"
#include "app/CodeOverlay.h"

namespace NextIdle {
uint16_t tgt = 0xFFFF, tgt2 = 0xFFFF, tgt3 = 0xFFFF;
uint16_t rej = 0xFFFF;

static uint16_t s_ok = 0xFFFF;     // validated loop address
static uint32_t s_lastT = 0, s_period = 0;
static uint32_t s_lastR = 0, s_dR = 0;   // opcode fetches per pass (Z80::regR)
static uint16_t s_regs[11];              // register file at the last pass start
static uint16_t s_rej[4] = { 0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF };   // recent non-loops
static uint8_t  s_rejW = 0;

// Opcodes a pass may contain: they change registers and flags only, or read
// memory, or push / pop (the stack gets the same bytes every pass once the
// registers repeat — jump() checks that). Length in bytes, 0 = not allowed.
static int plainLen(uint16_t a) {
    const uint8_t o = MemESP::dbgPeek(a);
    if (o >= 0x40 && o < 0x80) return (o >= 0x70 && o < 0x78) ? 0 : 1;   // LD r,r' / r,(HL); not LD (HL),r / HALT
    if (o >= 0x80 && o < 0xC0) return 1;                                   // ALU A,r / (HL)
    switch (o) {
        case 0x00: case 0x07: case 0x0F: case 0x17: case 0x1F: case 0x27: case 0x2F: case 0x37: case 0x3F:
        case 0x04: case 0x05: case 0x0C: case 0x0D: case 0x14: case 0x15: case 0x1C: case 0x1D:
        case 0x24: case 0x25: case 0x2C: case 0x2D: case 0x3C: case 0x3D:
        case 0x03: case 0x0B: case 0x13: case 0x1B: case 0x23: case 0x2B:
        case 0x09: case 0x19: case 0x29: case 0x39:
        case 0x0A: case 0x1A: case 0x08: case 0xD9: case 0xEB:
        case 0xC1: case 0xD1: case 0xE1: case 0xF1: case 0xC5: case 0xD5: case 0xE5: case 0xF5:
            return 1;
        case 0x06: case 0x0E: case 0x16: case 0x1E: case 0x26: case 0x2E: case 0x3E:
        case 0xC6: case 0xCE: case 0xD6: case 0xDE: case 0xE6: case 0xEE: case 0xF6: case 0xFE:
            return 2;
        case 0x01: case 0x11: case 0x21: case 0x2A: case 0x3A:
            return 3;
        case 0xCB: {
            const uint8_t x = MemESP::dbgPeek(a + 1);
            return ((x & 0xC0) == 0x40 || (x & 7) != 6) ? 2 : 0;   // BIT b,any; rotates / RES / SET on registers
        }
        case 0xED: {
            const uint8_t x = MemESP::dbgPeek(a + 1);
            if (x == 0x4B || x == 0x5B || x == 0x6B) return 4;            // LD BC/DE/HL,(nn)
            if ((x & 0xC7) == 0x42 || (x & 0xC7) == 0x4A) return 2;       // SBC / ADC HL,rr
            if (x == 0x44) return 2;                                      // NEG
            return 0;
        }
        default: return 0;
    }
}

// The pass from S: every path that does not leave the loop must come back to
// S through allowed instructions only. A conditional JP / JR to S closes the
// loop (its other side is the exit, not followed); any other conditional jump
// is followed both ways; CALL / RET inside the pass are followed, a RET cc in a
// called routine both ways, an unconditional RET at the top level is refused.
// Shapes seen on hw:
//   Pogie     [EI] LD A,(nn) / OR A / JP Z,S
//   NextSID   LD A,(HL) / S: CP A / CP (HL) / JR Z,S   (E093, 96% of the frame)
//   NextBench S: LD DE,(nn) / LD HL,(nn) / CALL f / OR A / JP NZ,S
//             f: XOR A / SBC HL,DE / RET NZ / INC A / RET
static bool validate(uint16_t s) {
    if (s < 0x4000) return false;                    // divMMC automap / ROM space
    struct Path { uint16_t pc; uint8_t sp; uint16_t ret[4]; };
    Path work[8];
    int nw = 0, steps = 0;
    bool closed = false;
    Path p0 = {};
    p0.pc = s;
    if (MemESP::dbgPeek(s) == 0xFB) p0.pc++;         // EI
    work[nw++] = p0;
    while (nw > 0) {
        Path p = work[--nw];
        for (;;) {
            if (++steps > 48) return false;
            if (p.pc == s && steps > 1) { closed = true; break; }
            const uint16_t a = p.pc;
            if (a < 0x4000) return false;
            const uint8_t o = MemESP::dbgPeek(a);
            const int len = plainLen(a);
            if (len) { p.pc = (uint16_t)(a + len); continue; }
            const uint16_t nn = (uint16_t)(MemESP::dbgPeek(a + 1) | (MemESP::dbgPeek(a + 2) << 8));
            const uint16_t jr = (uint16_t)(a + 2 + (int8_t)MemESP::dbgPeek(a + 1));
            if (o == 0xC3) { p.pc = nn; continue; }                   // JP nn
            if (o == 0x18) { p.pc = jr; continue; }                   // JR e
            if (o == 0xCD) {                                          // CALL nn
                if (p.sp >= 4) return false;
                p.ret[p.sp++] = (uint16_t)(a + 3);
                p.pc = nn;
                continue;
            }
            if (o == 0xC9) {                                          // RET
                if (!p.sp) return false;
                p.pc = p.ret[--p.sp];
                continue;
            }
            const bool jpcc = (o & 0xC7) == 0xC2, jrcc = o == 0x20 || o == 0x28 || o == 0x30 || o == 0x38;
            if (jpcc || jrcc) {
                const uint16_t to = jpcc ? nn : jr, nx = (uint16_t)(a + (jpcc ? 3 : 2));
                if (to == s && !p.sp) { closed = true; p.pc = to; break; }   // loop back; the fall-through exits
                if (nw >= 8) return false;
                Path q = p; q.pc = to; work[nw++] = q;
                p.pc = nx;
                continue;
            }
            if ((o & 0xC7) == 0xC0) {                                 // RET cc
                if (!p.sp) { p.pc = (uint16_t)(a + 1); continue; }    // top level: an exit
                if (nw >= 8) return false;
                Path q = p; q.pc = q.ret[--q.sp]; work[nw++] = q;
                p.pc = (uint16_t)(a + 1);
                continue;
            }
            return false;
        }
    }
    return closed;
}

static void readRegs(uint16_t r[11]) {
    r[0] = Z80::getRegAF();  r[1] = Z80::getRegBC();  r[2] = Z80::getRegDE();  r[3] = Z80::getRegHL();
    r[4] = Z80::getRegAFx(); r[5] = Z80::getRegBCx(); r[6] = Z80::getRegDEx(); r[7] = Z80::getRegHLx();
    r[8] = Z80::getRegIX();  r[9] = Z80::getRegIY();  r[10] = Z80::getRegSP();
}

#if NEXT_TRACE
volatile uint32_t g_nx_idle_n = 0, g_nx_idle_t = 0;
#endif

// One call per taken jump to a new target: one copy instead of the ~30 the
// opcode switch would inline. Plain SRAM (~40 B), NOT the .nxovl window: that
// window shares its addresses with .tsovl / .ftovl, and with this frame on
// core0's stack GDB found a TS-Conf function at the return address and died
// unwinding (-stack-list-frames: "Recursive internal problem", hw 2026-10-11).
__attribute__((noinline)) __not_in_flash("next") void noteNew(uint16_t pc) {
    if ((pc == tgt2 || pc == tgt3) && z80_in_nocheck && pc != rej) jump();
    tgt3 = tgt2; tgt2 = tgt; tgt = pc;
}

void jump() {
    const uint16_t s = Z80::getRegPC();
    if (s != s_ok) {
        for (int i = 0; i < 4; i++) if (s_rej[i] == s) { rej = s; return; }
        if (!validate(s)) { rej = s; s_rej[s_rejW++ & 3] = s; return; }
        s_ok = s;
        s_period = 0;
        s_lastT = CPU::tstates;
        s_lastR = Z80::getFetchCounter();
        readRegs(s_regs);
        return;
    }
    const uint32_t now = CPU::tstates, r = Z80::getFetchCounter();
    const uint32_t d = now - s_lastT, dR = r - s_lastR;
    s_lastT = now; s_lastR = r;
    // the pass repeats exactly only if it starts from the same registers
    uint16_t regs[11];
    readRegs(regs);
    bool same = true;
    for (int i = 0; i < 11; i++) { same &= regs[i] == s_regs[i]; s_regs[i] = regs[i]; }
    if (!same || d != s_period || dR != s_dR) { s_period = d; s_dR = dR; return; }   // two equal passes first
    if (Rzx::mode || !Z80::isIFF1() || (int32_t)now < CPU::IntEnd || d == 0) return;
    uint32_t lim = CPU::statesInFrame - (uint32_t)CPU::IntEnd;
    if (CPU::stFrame < lim) lim = CPU::stFrame;
    if (lim <= now) return;
    const uint32_t k = (lim - now) / d;
    if (k < 2) return;
    if (!NEXTVID::idleAdvanceTo(now + k * d)) return;
    Z80::incRegR(k * dR);
    s_lastT = CPU::tstates;
    s_lastR = Z80::getFetchCounter();
#if NEXT_TRACE
    g_nx_idle_n = g_nx_idle_n + 1;
    g_nx_idle_t = g_nx_idle_t + (CPU::tstates - now);
#endif
}
} // namespace NextIdle
