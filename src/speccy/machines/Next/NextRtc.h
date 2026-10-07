#pragma once
#include <stdint.h>

// ZX Spectrum Next I2C bus (ports #103B SCL / #113B SDA, bit 0, bit-banged by
// the guest) with the optional DS1307 real-time clock on it at address 0x68.
//
// Sources: wiki.specnext.dev "I2C clock" / "RTC" (DS1307 at 0x68, open-drain
// lines idle at 1); zxnext.vhd 3236-3266 (the writes latch the master's line,
// a read returns "1111111" & line); the TBBLUE distro's RTC drivers
// src/asm/rtc/rtcack.asm (= nextzxos/RTC.SYS: it REQUIRES an ACK on every byte
// it sends — four SDA polls with SCL high) and rtcsig.asm (reads 10 bytes from
// register 0x3E and wants the signature 'Z','X' there).
//
// The clock fields come from RTC:: (SNTP / the CMOS clock) and a guest write
// of them (.TIME / .DATE) sets RTC:: on STOP. The chip answers only while
// Devices > "CMOS + NVRAM" is on (Config::rtc_enabled) — the board's RTC is an
// add-on, exactly like the clocks of the other machines here.
namespace NextRtc {
    void    reset();                  // machine reset: bus idle, both lines released
    void    writeScl(uint8_t v);      // OUT (#103B)
    void    writeSda(uint8_t v);      // OUT (#113B)
    uint8_t readScl();                // IN  (#103B)
    uint8_t readSda();                // IN  (#113B)
}
