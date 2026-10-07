// Host test for src/speccy/machines/Next/NextRtc.cpp (Next I2C + DS1307).
// Drives the bus with the exact pin sequences of the TBBLUE distro drivers
// (src/asm/rtc/rtcack.asm = nextzxos/RTC.SYS, rtcsig.asm, time.asm writes).
//
//   g++ -O2 -Wall -Wextra -Itools/nextrtc_stub -Isrc -o /tmp/nextrtc_test
//       tools/nextrtc_test.cpp src/speccy/machines/Next/NextRtc.cpp && /tmp/nextrtc_test
// (the stub dir FIRST: its RTC.h / Config.h / Debug.h stand in for the firmware's)
#include <cstdio>
#include <cstdarg>
#include "speccy/machines/Next/NextRtc.h"
#include "speccy/devices/storage/RTC.h"
#include "app/Config.h"

// ---- stubs ----
bool Config::rtc_enabled = true;
static bool g_valid = true;
static int g_t[6] = {2026, 10, 7, 21, 5, 9};
bool RTC::now(int& y, int& mo, int& d, int& h, int& mi, int& s) {
    if (!g_valid) return false;
    y = g_t[0]; mo = g_t[1]; d = g_t[2]; h = g_t[3]; mi = g_t[4]; s = g_t[5]; return true;
}
static int g_set[6]; static int g_setCount = 0;
void RTC::setDateTime(int y, int mo, int d, int h, int mi, int s) {
    g_set[0] = y; g_set[1] = mo; g_set[2] = d; g_set[3] = h; g_set[4] = mi; g_set[5] = s; g_setCount++;
}
namespace Debug { void log(const char*, ...) {} }

// ---- the driver's primitives (rtcack.asm) ----
static void SCL(int v) { NextRtc::writeScl((uint8_t)v); }
static void SDA(int v) { NextRtc::writeSda((uint8_t)v); }
static int  rdSDA() { return NextRtc::readSda() & 1; }
static void pulse() { SCL(1); SCL(0); }
static void start() { SCL(1); SDA(1); SDA(0); SCL(0); }
static void stop() { SDA(0); SCL(1); SDA(1); }
// SEND_DATA + WAIT_ACK: returns true if ACKed (>= 2 of 4 polls low)
static bool send(uint8_t b) {
    for (int i = 7; i >= 0; i--) { SDA((b >> i) & 1); pulse(); }
    SDA(1); SCL(1);
    int low = 0;
    for (int i = 0; i < 4; i++) if (!rdSDA()) low++;
    SCL(0);
    return low >= 2;
}
static uint8_t readByte() {
    SDA(1);
    uint8_t v = 0;
    for (int i = 0; i < 8; i++) { SCL(1); v = (uint8_t)((v << 1) | rdSDA()); SCL(0); }
    return v;
}
static void ackNack(int nack) { SDA(nack); pulse(); SDA(1); }

static int fails = 0;
#define CHECK(c, ...) do { if (!(c)) { printf("FAIL %s:%d: ", __FILE__, __LINE__); printf(__VA_ARGS__); printf("\n"); fails++; } } while (0)

int main() {
    NextRtc::reset();
    // 1. rtcack.asm: read the 7 clock registers
    start();
    CHECK(send(0xD0), "no ACK on D0");
    CHECK(send(0x00), "no ACK on pointer");
    start();
    CHECK(send(0xD1), "no ACK on D1");
    uint8_t r[7];
    for (int i = 0; i < 7; i++) { r[i] = readByte(); ackNack(i == 6); }
    stop();
    CHECK(r[0] == 0x09 && r[1] == 0x05 && r[2] == 0x21, "time %02X %02X %02X", r[0], r[1], r[2]);
    CHECK(r[4] == 0x07 && r[5] == 0x10 && r[6] == 0x26, "date %02X %02X %02X", r[4], r[5], r[6]);
    CHECK(r[3] == 4, "2026-10-07 is a Wednesday (4, Sunday=1), got %u", r[3]);

    // 2. rtcsig.asm: 10 bytes from 0x3E — signature then wrap to register 0
    start(); send(0xD0); send(0x3E); start(); send(0xD1);
    uint8_t s[10];
    for (int i = 0; i < 10; i++) { s[i] = readByte(); ackNack(i == 9); }
    stop();
    CHECK(s[0] == 'Z' && s[1] == 'X', "signature %02X %02X", s[0], s[1]);
    CHECK(s[2] == 0x09 && s[8] == 0x26, "wrap to reg 0: %02X ... %02X", s[2], s[8]);

    // 3. .TIME / .DATE write: pointer 0, then 7 bytes, STOP commits
    start(); send(0xD0); send(0x00);
    const uint8_t w[7] = {0x30, 0x45, 0x13, 0x02, 0x31, 0x12, 0x27};
    bool ok = true;
    for (int i = 0; i < 7; i++) ok &= send(w[i]);
    stop();
    CHECK(ok, "data bytes not ACKed");
    CHECK(g_setCount == 1 && g_set[0] == 2027 && g_set[1] == 12 && g_set[2] == 31 &&
          g_set[3] == 13 && g_set[4] == 45 && g_set[5] == 30,
          "setDateTime %d (%d-%d-%d %d:%d:%d)", g_setCount, g_set[0], g_set[1], g_set[2], g_set[3], g_set[4], g_set[5]);

    // 4. RAM write + read back (register 0x10)
    start(); send(0xD0); send(0x10); send(0xA5); send(0x5A); stop();
    start(); send(0xD0); send(0x10); start(); send(0xD1);
    uint8_t a = readByte(); ackNack(0); uint8_t b = readByte(); ackNack(1); stop();
    CHECK(a == 0xA5 && b == 0x5A, "RAM %02X %02X", a, b);

    // 5. another device address: no ACK, bus released
    start(); CHECK(!send(0xA0), "ACK for 0xA0"); stop();
    CHECK(rdSDA() == 1, "SDA held after NACK");

    // 6. chip absent (CMOS + NVRAM off): no ACK — RTC.SYS reports no clock
    Config::rtc_enabled = false;
    start(); CHECK(!send(0xD0), "ACK with the RTC off"); stop();
    Config::rtc_enabled = true;

    // 7. lines idle high, SCL reads back
    CHECK(NextRtc::readScl() == 0xFF && NextRtc::readSda() == 0xFF, "idle %02X %02X",
          NextRtc::readScl(), NextRtc::readSda());

    printf(fails ? "%d FAILED\n" : "OK\n", fails);
    return fails != 0;
}
