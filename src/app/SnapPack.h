// Little-endian byte packer for the device parts of a .pss snapshot (Tape, uPD765,
// IDE, the SD/MMC interfaces). Header-only and dependency-free, so a device module
// can serialize its own state without knowing anything about the .pss container.
#pragma once

#include <stdint.h>
#include <string.h>

struct SnapOut {
    uint8_t* p;
    uint32_t n = 0;
    explicit SnapOut(uint8_t* buf) : p(buf) {}
    void u8(uint8_t v)   { p[n++] = v; }
    void u16(uint16_t v) { u8((uint8_t)v); u8((uint8_t)(v >> 8)); }
    void u32(uint32_t v) { u16((uint16_t)v); u16((uint16_t)(v >> 16)); }
    void u64(uint64_t v) { u32((uint32_t)v); u32((uint32_t)(v >> 32)); }
    void raw(const void* s, uint32_t len) { memcpy(p + n, s, len); n += len; }
};

// Reads past the end yield 0 and clear `ok`, so a truncated block is detected once
// at the end instead of at every field.
struct SnapIn {
    const uint8_t* p;
    uint32_t n, at = 0;
    bool ok = true;
    SnapIn(const uint8_t* buf, uint32_t len) : p(buf), n(len) {}
    uint8_t u8() { if (at >= n) { ok = false; return 0; } return p[at++]; }
    uint16_t u16() { const uint16_t lo = u8(); return (uint16_t)(lo | (u8() << 8)); }
    uint32_t u32() { const uint32_t lo = u16(); return lo | ((uint32_t)u16() << 16); }
    uint64_t u64() { const uint64_t lo = u32(); return lo | ((uint64_t)u32() << 32); }
    void raw(void* d, uint32_t len) {
        if (at + len > n) { ok = false; memset(d, 0, len); return; }
        memcpy(d, p + at, len); at += len;
    }
};
