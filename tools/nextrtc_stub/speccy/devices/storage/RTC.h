// Host stub for tools/nextrtc_test.cpp — the two RTC:: calls NextRtc uses.
#pragma once
struct RTC {
    static bool now(int& y, int& mo, int& d, int& h, int& mi, int& s);
    static void setDateTime(int y, int mo, int d, int h, int mi, int s);
};
