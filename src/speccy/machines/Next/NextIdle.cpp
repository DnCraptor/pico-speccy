// ZX Next memory-poll idle loop fast-forward — see NextIdle.h.
#include "NextIdle.h"
#include "speccy/machines/Next/NextVideo.h"
#include "speccy/z80/z80.h"
#include "speccy/z80/CPU.h"
#include "speccy/core/MemESP.h"
#include "speccy/core/Rzx.h"

namespace NextIdle {
uint16_t tgt = 0xFFFF;
uint16_t rej = 0xFFFF;

static uint16_t s_ok = 0xFFFF;     // validated loop address
static uint8_t  s_m1 = 0;          // opcode fetches per pass (R)
static uint32_t s_lastT = 0, s_period = 0;

// The pass at S is a pure read-and-test loop that jumps back to S: every
// instruction only loads A from memory or sets flags (each pass repeats the
// previous one exactly while memory stays put), and the pass ends with a
// conditional JP / JR back to S. Shapes seen on hw:
//   Pogie   [EI] LD A,(nn) / OR A / JP Z,S
//   NextSID LD A,(HL) / S: CP A / CP (HL) / JR Z,S   (E093, 96% of the frame)
static bool validate(uint16_t s) {
    if (s < 0x4000) return false;                    // divMMC automap / ROM space
    uint16_t i = s;
    uint8_t m1 = 0;
    if (MemESP::dbgPeek(i) == 0xFB) { i++; m1++; }   // EI
    for (int n = 0; n < 8; n++) {
        const uint8_t o = MemESP::dbgPeek(i);
        uint16_t to;
        if (o == 0xC2 || o == 0xCA || o == 0xD2 || o == 0xDA) {          // JP NZ/Z/NC/C,S
            to = (uint16_t)(MemESP::dbgPeek(i + 1) | (MemESP::dbgPeek(i + 2) << 8));
        } else if (o == 0x20 || o == 0x28 || o == 0x30 || o == 0x38) {   // JR NZ/Z/NC/C,S
            to = (uint16_t)(i + 2 + (int8_t)MemESP::dbgPeek(i + 1));
        } else {
            if (o == 0x3A) i += 3;                                       // LD A,(nn)
            else if (o == 0x7E || o == 0x0A || o == 0x1A) i += 1;        // LD A,(HL)/(BC)/(DE)
            else if (o >= 0xB8 && o <= 0xBF) i += 1;                     // CP r / CP (HL)
            else if (o == 0xB7 || o == 0xA7) i += 1;                     // OR A / AND A
            else if (o == 0xFE || o == 0xE6 || o == 0xF6) i += 2;        // CP n / AND n / OR n
            else if (o == 0xCB && (MemESP::dbgPeek(i + 1) & 0xC0) == 0x40) { i += 2; m1++; } // BIT b,r/(HL)
            else return false;
            m1++;
            continue;
        }
        if (to != s || n == 0) return false;   // a bare jump to itself is not a poll
        s_m1 = (uint8_t)(m1 + 1);
        return true;
    }
    return false;
}

#if NEXT_TRACE
volatile uint32_t g_nx_idle_n = 0, g_nx_idle_t = 0;
#endif

void jump() {
    const uint16_t s = Z80::getRegPC();
    if (s != s_ok) {
        if (!validate(s)) { rej = s; return; }
        s_ok = s;
        s_period = 0;
        s_lastT = CPU::tstates;
        return;
    }
    const uint32_t now = CPU::tstates;
    const uint32_t d = now - s_lastT;
    s_lastT = now;
    if (d != s_period) { s_period = d; return; }     // two equal passes first
    if (Rzx::mode || !Z80::isIFF1() || (int32_t)now < CPU::IntEnd || d == 0) return;
    uint32_t lim = CPU::statesInFrame - (uint32_t)CPU::IntEnd;
    if (CPU::stFrame < lim) lim = CPU::stFrame;
    if (lim <= now) return;
    const uint32_t k = (lim - now) / d;
    if (k < 2) return;
    if (!NEXTVID::idleAdvanceTo(now + k * d)) return;
    Z80::incRegR(k * s_m1);
    s_lastT = CPU::tstates;
#if NEXT_TRACE
    g_nx_idle_n = g_nx_idle_n + 1;
    g_nx_idle_t = g_nx_idle_t + (CPU::tstates - now);
#endif
}
} // namespace NextIdle
