// ZX Spectrum Next I2C bus + DS1307 — see NextRtc.h.
#include "NextRtc.h"
#include "speccy/devices/storage/RTC.h"
#include "app/Config.h"
#include "app/Debug.h"

namespace {

enum Phase : uint8_t {
    P_IDLE,      // not addressed: SDA released, ignore clocks until START
    P_RECV,      // receiving a byte from the master (address, pointer, data)
    P_ACK_OUT,   // driving our ACK for the byte just received
    P_SEND,      // driving a data byte to the master
    P_ACK_IN,    // listening for the master's ACK / NACK after a byte we sent
};

uint8_t scl = 1, sdaM = 1;      // the master's lines (written by the guest)
uint8_t sdaS = 1;               // what the DS1307 drives (1 = released)
uint8_t phase = P_IDLE;
uint8_t bits = 0;               // bits received (P_RECV) / bit index (P_SEND)
uint8_t shift = 0;
uint8_t byteNo = 0;             // 0 = device address, 1 = register pointer, 2+ = data
bool    reading = false;
uint8_t ptr = 0;                // register pointer, wraps at 64
uint8_t txByte = 0;
uint8_t ackBit = 1;
bool    timeWritten = false;
uint8_t regs[64];               // 0-6 clock (BCD), 7 control, 8-63 RAM
bool    inited = false;

uint8_t bcd(int v) { return (uint8_t)(((v / 10) << 4) | (v % 10)); }
int     unbcd(uint8_t v) { return (v >> 4) * 10 + (v & 0x0F); }

// Day of week 1..7, Sunday = 1 (the convention RTC:: uses for the MC146818).
uint8_t dow(int y, int m, int d) {
    static const int t[12] = {0, 3, 2, 5, 0, 3, 5, 1, 4, 6, 2, 4};
    if (m < 3) y -= 1;
    return (uint8_t)((y + y / 4 - y / 100 + y / 400 + t[m - 1] + d) % 7 + 1);
}

void init() {
    if (inited) return;
    inited = true;
    for (int i = 0; i < 64; i++) regs[i] = 0;
    regs[3] = 7; regs[4] = 1; regs[5] = 1;     // Sat 2000-01-01 00:00:00 (power-on)
    regs[62] = 'Z'; regs[63] = 'X';            // the signature RTCSIG.SYS checks
}

// The DS1307 copies its counters into the read buffer at every START; ours
// come from RTC:: when it has a time (SNTP / the CMOS clock) — otherwise the
// last values written stay, as on a chip that was never set.
void latchTime() {
    int y, mo, d, h, mi, s;
    if (!RTC::now(y, mo, d, h, mi, s) || y < 2000 || y > 2099) return;
    regs[0] = bcd(s); regs[1] = bcd(mi); regs[2] = bcd(h);   // bit 6 = 0: 24-hour
    regs[3] = dow(y, mo, d); regs[4] = bcd(d); regs[5] = bcd(mo); regs[6] = bcd(y - 2000);
}

void commitTime() {
    if (!timeWritten) return;
    timeWritten = false;
    int h = (regs[2] & 0x40) ? (unbcd(regs[2] & 0x1F) % 12 + ((regs[2] & 0x20) ? 12 : 0))
                             : unbcd(regs[2] & 0x3F);
    const int s = unbcd(regs[0] & 0x7F), mi = unbcd(regs[1] & 0x7F);
    const int d = unbcd(regs[4] & 0x3F), mo = unbcd(regs[5] & 0x1F), y = 2000 + unbcd(regs[6]);
    if (s > 59 || mi > 59 || h > 23 || d < 1 || d > 31 || mo < 1 || mo > 12) return;
    RTC::setDateTime(y, mo, d, h, mi, s);
    Debug::log("[NXRTC] guest set %04d-%02d-%02d %02d:%02d:%02d", y, mo, d, h, mi, s);
}

void byteReceived() {
    if (byteNo == 0) {
        if ((shift >> 1) != 0x68 || !Config::rtc_enabled) {   // not us / no chip: no ACK
            phase = P_IDLE; sdaS = 1;
            return;
        }
        reading = shift & 1;
    } else if (byteNo == 1) {
        ptr = shift & 63;
    } else {
        regs[ptr] = shift;
        if (ptr < 7) timeWritten = true;
        ptr = (ptr + 1) & 63;
    }
    byteNo++;
    phase = P_ACK_OUT;
    sdaS = 0;                                   // ACK
}

void loadTx() {
    txByte = regs[ptr];
    ptr = (ptr + 1) & 63;
    bits = 7;
    sdaS = (txByte >> 7) & 1;
    phase = P_SEND;
}

} // namespace

void NextRtc::reset() {
    init();
    scl = sdaM = sdaS = 1;
    phase = P_IDLE;
    bits = byteNo = 0;
}

void NextRtc::writeSda(uint8_t v) {
    init();
    v &= 1;
    if (scl && v != sdaM) {
        if (!v) {                               // START (or repeated START)
            latchTime();
            phase = P_RECV; bits = 0; byteNo = 0; shift = 0; sdaS = 1;
        } else {                                // STOP
            commitTime();
            phase = P_IDLE; sdaS = 1;
        }
    }
    sdaM = v;
}

void NextRtc::writeScl(uint8_t v) {
    init();
    v &= 1;
    if (v == scl) return;
    scl = v;
    const uint8_t line = sdaM & sdaS;
    if (v) {                                    // rising: the receiver samples SDA
        if (phase == P_RECV) { shift = (uint8_t)((shift << 1) | line); bits++; }
        else if (phase == P_ACK_IN) ackBit = line;
        return;
    }
    // falling: the transmitter moves to its next bit
    switch (phase) {
        case P_RECV:
            if (bits == 8) byteReceived();
            break;
        case P_ACK_OUT:
            if (reading && byteNo >= 1) loadTx();
            else { phase = P_RECV; bits = 0; shift = 0; sdaS = 1; }
            break;
        case P_SEND:
            if (bits) { bits--; sdaS = (txByte >> bits) & 1; }
            else { sdaS = 1; phase = P_ACK_IN; ackBit = 1; }
            break;
        case P_ACK_IN:
            if (!ackBit) loadTx();              // ACK: the next register
            else { phase = P_IDLE; sdaS = 1; }  // NACK: the master is done
            break;
        default:
            break;
    }
}

uint8_t NextRtc::readScl() { return (uint8_t)(0xFE | scl); }
uint8_t NextRtc::readSda() { return (uint8_t)(0xFE | (sdaM & sdaS)); }
