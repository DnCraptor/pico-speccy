// Wall-clock limit for Next rendering in Max speed mode. The emulated CPU
// and copper still run every frame; only framebuffer composition is skipped.
#pragma once
#include <stdint.h>

class NextFramePacing {
    uint32_t lastDrawUs = 0;
public:
    static constexpr uint32_t intervalUs = 20000; // up to 50 drawn frames/s
    void reset(uint32_t now) { lastDrawUs = now - intervalUs; }
    bool skip(bool maxSpeed, uint32_t now) {
        // Unsigned subtraction also handles the 32-bit timer wrapping.
        if (maxSpeed && (uint32_t)(now - lastDrawUs) < intervalUs) return true;
        lastDrawUs = now;
        return false;
    }
};
