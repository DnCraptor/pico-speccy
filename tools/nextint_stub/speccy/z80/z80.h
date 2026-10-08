#pragma once
// Host stub for tools/nextint_test.cpp
struct Z80 {
    enum IntMode { IM0, IM1, IM2 };
    static IntMode im;
    static IntMode getIM() { return im; }
};
