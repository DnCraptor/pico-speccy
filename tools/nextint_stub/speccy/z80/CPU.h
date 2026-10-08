#pragma once
#include <stdint.h>
// Host stub for tools/nextint_test.cpp
struct CPU {
    static uint32_t tstates, statesInFrame, stFrame;
    static uint8_t latetiming;
    static int32_t IntStart, IntEnd;
};
