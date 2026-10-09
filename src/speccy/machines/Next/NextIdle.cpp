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

// The pass at S is a pure read-and-test loop that jumps back to S.
static bool validate(uint16_t s) {
    if (s < 0x4000) return false;                    // divMMC automap / ROM space
    uint16_t i = s;
    uint8_t m1 = 0;
    if (MemESP::dbgPeek(i) == 0xFB) { i++; m1++; }   // EI
    if (MemESP::dbgPeek(i) != 0x3A) return false;    // LD A,(nn)
    i += 3; m1++;
    const uint8_t t = MemESP::dbgPeek(i);
    if (t == 0xB7 || t == 0xA7) { i += 1; m1++; }                       // OR A / AND A
    else if (t == 0xFE || t == 0xE6) { i += 2; m1++; }                  // CP n / AND n
    else if (t == 0xCB && (MemESP::dbgPeek(i + 1) & 0xC7) == 0x47) {    // BIT b,A
        i += 2; m1 += 2;
    } else return false;
    const uint8_t j = MemESP::dbgPeek(i);
    if (j != 0xC2 && j != 0xCA && j != 0xD2 && j != 0xDA) return false; // JP NZ/Z/NC/C
    if ((uint16_t)(MemESP::dbgPeek(i + 1) | (MemESP::dbgPeek(i + 2) << 8)) != s) return false;
    s_m1 = (uint8_t)(m1 + 1);
    return true;
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
