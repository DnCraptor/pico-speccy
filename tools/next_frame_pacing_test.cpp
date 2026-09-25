// g++ -std=c++17 -O2 -Wall -Wextra -Isrc tools/next_frame_pacing_test.cpp -o /tmp/next_frame_pacing_test && /tmp/next_frame_pacing_test
#include "speccy/machines/Next/NextFramePacing.h"
#include <cassert>
#include <cstdio>
#include <initializer_list>

int main() {
    for (uint32_t origin : {0u, 0xfffffff0u}) {
        for (uint32_t step : {1000u, 10000u, 19992u, 20000u, 47000u}) {
            NextFramePacing pacing;
            pacing.reset(origin);
            uint32_t last = origin;
            unsigned drawn = 0;
            for (unsigned f = 0; f < 1000; ++f) {
                const uint32_t now = origin + f * step;
                if (!pacing.skip(true, now)) {
                    if (drawn) {
                        assert(uint32_t(now - last) >= NextFramePacing::intervalUs);
                        assert(uint32_t(now - last) < NextFramePacing::intervalUs + step);
                    }
                    last = now;
                    ++drawn;
                }
            }
            // BIOS near 50 Hz must draw at least every other frame;
            // at 21 Hz (47 ms/frame), Max speed must not skip anything.
            if (step >= 19992) assert(drawn >= 500);
            if (step >= 20000) assert(drawn == 1000);
        }
        NextFramePacing pacing;
        pacing.reset(origin);
        assert(!pacing.skip(true, origin));
        assert(pacing.skip(true, origin + 1));
        assert(!pacing.skip(false, origin + 2)); // disable immediately
        assert(!pacing.skip(false, origin + 3));
        pacing.reset(origin + 4);
        assert(!pacing.skip(true, origin + 4)); // reset draws immediately
    }
    puts("OK: fast and slow emulation, BIOS cadence, mode changes, reset and timer wrap");
}
