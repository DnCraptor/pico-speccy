// Host regression for the actual inline Next operand readers.
// g++ -std=c++17 -O2 -Wall -Wextra -Isrc tools/next_memory_test.cpp -o /tmp/next_memory_test && /tmp/next_memory_test
#include <cassert>
#include <cstdint>
#include <cstdio>
#include <initializer_list>

struct CPU { static uint32_t tstates; };
struct VIDEO { static uint32_t ts_line_t; };
struct MemESP { static uint8_t* ramCurrent[8]; };
struct Z80Ops {
    static uint8_t peek8(uint16_t);
    static uint16_t peek16(uint16_t);
};
uint32_t CPU::tstates;
uint32_t VIDEO::ts_line_t;
uint8_t* MemESP::ramCurrent[8];
bool g_next_fastrd;

#include "speccy/z80/next_memory.h"

static uint8_t pages[9][8192];
static unsigned calls;
static uint32_t entered;
static bool remap;

static void tick(unsigned cycles) {
    ++calls;
    entered = CPU::tstates;
    CPU::tstates += cycles;
    if (remap && CPU::tstates >= VIDEO::ts_line_t) {
        MemESP::ramCurrent[0] = pages[8];
        VIDEO::ts_line_t += 228;
    }
}
uint8_t Z80Ops::peek8(uint16_t a) {
    tick(3);
    return MemESP::ramCurrent[a >> 13][a & 8191];
}
uint16_t Z80Ops::peek16(uint16_t a) {
    tick(6);
    const uint16_t b = a + 1;
    return MemESP::ramCurrent[a >> 13][a & 8191] |
           (MemESP::ramCurrent[b >> 13][b & 8191] << 8);
}

int main() {
    for (unsigned p = 0; p < 9; ++p)
        for (unsigned i = 0; i < 8192; ++i)
            pages[p][i] = (p * 29 + i * 7 + (i >> 8)) & 255;
    // Deliberately noncontiguous 8K slots; no 16K/unaligned-load shortcut.
    for (unsigned s = 0; s < 8; ++s) MemESP::ramCurrent[s] = pages[s ^ 5];

    for (bool enabled : {false, true}) {
        g_next_fastrd = enabled;
        VIDEO::ts_line_t = 228;
        for (unsigned a = 0; a < 65536; ++a) {
            CPU::tstates = 100; calls = 0;
            assert(NextMemory::peek8(a) == MemESP::ramCurrent[a >> 13][a & 8191]);
            assert(CPU::tstates == 103 && calls == unsigned(!enabled));
            CPU::tstates = 100; calls = 0;
            const uint16_t b = a + 1;
            const uint16_t expected = MemESP::ramCurrent[a >> 13][a & 8191] |
                (MemESP::ramCurrent[b >> 13][b & 8191] << 8);
            assert(NextMemory::peek16(a) == expected);
            assert(CPU::tstates == 106 && calls == unsigned(!enabled));
        }
    }

    // Exact boundary and overshoot must delegate BEFORE adding any cycles.
    // A copper MMU write at the tick must affect the read, including 0xFFFF
    // wrapping into the newly mapped slot 0.
    g_next_fastrd = true;
    remap = true;
    for (unsigned start = 221; start <= 230; ++start) {
        MemESP::ramCurrent[0] = pages[0];
        CPU::tstates = start; VIDEO::ts_line_t = 228; calls = 0;
        const bool crosses8 = start + 3 >= 228;
        assert(NextMemory::peek8(0) == pages[crosses8 ? 8 : 0][0]);
        assert(calls == unsigned(crosses8) && CPU::tstates == start + 3);
        if (calls) assert(entered == start);

        MemESP::ramCurrent[0] = pages[0];
        CPU::tstates = start; VIDEO::ts_line_t = 228; calls = 0;
        const bool crosses16 = start + 6 >= 228;
        const uint16_t expected = MemESP::ramCurrent[7][8191] |
            (pages[crosses16 ? 8 : 0][0] << 8);
        assert(NextMemory::peek16(0xFFFF) == expected);
        assert(calls == unsigned(crosses16) && CPU::tstates == start + 6);
        if (calls) assert(entered == start);
    }
    puts("OK: all 65536 addresses, 8K slot crossings, wrap, disabled path and MMU changes at line boundaries");
}
