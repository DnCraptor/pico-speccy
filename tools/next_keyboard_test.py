#!/usr/bin/env python3
"""Compare the actual no-tape Next FE helper with the original ULA equations."""
from pathlib import Path
import subprocess
import tempfile

src = (Path(__file__).resolve().parents[1] / 'src/speccy/core/Ports.cpp').read_text()
start = src.index('static inline __attribute__((always_inline)) uint8_t nextKeyboardRead(')
end = src.index('\ntemplate<bool PROFI>', start)
helper = src[start:end]
start = src.index('  if (!PROFI && Z80Ops::isNext &&')
end = src.index('return nextKeyboardRead(address);', start) + len('return nextKeyboardRead(address);')
guard = src[start:end]
harness = r'''
#include <cstdint>
#include <cassert>
#include <cstdio>
namespace Ports { uint8_t port[128], latch; uint8_t feLatch() { return latch; } }
namespace NextReg { uint8_t reg[256]; }
namespace Tape { int tapeFileType, tapeStatus; bool tapeEarBit; }
namespace Z80Ops { bool isNext; }
enum { TAPE_FTYPE_EMPTY = 0, TAPE_LOADING = 1 };
bool g_ula_io_contend, mutate;
unsigned cycles, draws;
namespace VIDEO {
void Draw(unsigned n, bool c) {
    assert(n == 3 && c == g_ula_io_contend);
    cycles += n; ++draws;
    // Copper may change the issue-2 register during the late I/O cycles.
    if (mutate) NextReg::reg[8] ^= 1;
}
}
'''
main = r'''
int main() {
    unsigned cases = 0;
    // Every row selector, individual keyboard bit, latch value and EAR state.
    // Also cover no keys, several rows pressed, both contention settings and a
    // register change DURING Draw (feedback must use the new value).
    for (unsigned pattern = 0; pattern < 66; ++pattern) {
      for (unsigned r = 0; r < 8; ++r)
        Ports::port[r] = pattern < 64 ? (r == pattern / 8 ? 255 ^ (1 << (pattern % 8)) : 255)
                          : pattern == 64 ? 255 : (255 ^ (1 << (r % 5)));
      for (unsigned high = 0; high < 256; ++high)
       for (unsigned latch = 0; latch < 256; ++latch)
        for (unsigned state = 0; state < 8; ++state) {
          Ports::latch = latch;
          NextReg::reg[8] = (state & 1) | 0xA0;
          Tape::tapeEarBit = state & 2;
          mutate = state & 4;
          g_ula_io_contend = high & 1;
          cycles = draws = 0;
          unsigned expected = 0xBF;
          for (unsigned r = 0; r < 8; ++r)
            if (!(high & (1 << r))) expected &= Ports::port[r];
          bool issue2 = bool(state & 1) != mutate;
          if ((latch & 0x10) || (issue2 && (latch & 8))) expected |= 0x40;
          if (Tape::tapeEarBit) expected ^= 0x40;
          assert(nextKeyboardRead((high << 8) | 0xFE) == expected);
          assert(draws == 1 && cycles == 3);
          ++cases;
        }
    }
    for (unsigned a = 0; a < 65536; ++a)
      for (unsigned state = 0; state < 32; ++state) {
        Z80Ops::isNext = state & 1;
        Tape::tapeFileType = (state >> 1) & 3;
        Tape::tapeStatus = (state >> 3) & 1;
        bool profi = state & 16;
        bool allowed = !profi && Z80Ops::isNext && (a & 255) == 254 &&
                       Tape::tapeFileType == 0 && Tape::tapeStatus != TAPE_LOADING;
        int result = profi ? guarded<true>(a) : guarded<false>(a);
        assert((result >= 0) == allowed);
      }
    printf("OK: %u keyboard/EAR/timing cases and 2097152 decode/tape guards\n", cases);
}
'''
with tempfile.TemporaryDirectory(prefix='next-keyboard-') as tmp:
    cpp = Path(tmp) / 'test.cpp'
    cpp.write_text(harness + helper + '\ntemplate<bool PROFI> int guarded(uint16_t address) {\n'
                   + guard + '\nreturn -1;\n}\n' + main)
    exe = Path(tmp) / 'test'
    subprocess.run(['g++', '-std=c++17', '-O2', '-Wall', '-Wextra', '-fsanitize=undefined',
                    str(cpp), '-o', str(exe)], check=True)
    subprocess.run([str(exe)], check=True)
