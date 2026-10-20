// CswFormat.h — the CSW ("Compressed Square Wave") tape format, firmware-free.
//
// Format (Ramsoft / Fred Ramsay, acorn.huininga.nl/pub/unsorted/software/pc/CSW/csw.html):
//   0x00  22  "Compressed Square Wave"
//   0x16   1  0x1A
//   0x17   1  major version (1 or 2)
//   0x18   1  minor version
//   v1.01:
//   0x19   2  sample rate, Hz
//   0x1B   1  compression: 1 = RLE (the only one v1 has)
//   0x1C   1  flags: bit 0 = initial polarity (1 = high)
//   0x1D   3  reserved
//   0x20      data
//   v2.0:
//   0x19   4  sample rate, Hz
//   0x1D   4  total number of pulses (after decompression)
//   0x21   1  compression: 1 = RLE, 2 = Z-RLE (the RLE stream deflated, zlib header)
//   0x22   1  flags: bit 0 = initial polarity
//   0x23   1  header extension length (HDR)
//   0x24  16  encoding application, ASCII
//   0x34 HDR  header extension
//   0x34+HDR  data
// RLE: each byte is one pulse (a run of samples at one level); a 0 byte is
// followed by the pulse as a 32-bit little-endian count.
//
// Kept apart from Tape.cpp so tools/csw_test.cpp can drive the parser and the
// decoder on a host; the decoder depends on nothing but tinfl and a read callback.

#pragma once

#include <stdint.h>
#include <string.h>
#include <stddef.h>
#include "miniz/miniz.h"

namespace csw {

static constexpr uint32_t HDR_MIN = 0x20;     // enough to tell v1 from v2
static constexpr uint32_t HDR_MAX = 0x34 + 255;

struct Header {
    uint8_t  major = 0, minor = 0;
    uint32_t rate = 0;          // samples per second
    uint32_t pulses = 0;        // v2 only, 0 = unknown
    uint8_t  compression = 0;   // 1 RLE, 2 Z-RLE
    bool     polarity = false;  // initial level high
    uint32_t dataStart = 0;     // file offset of the pulse stream
    char     app[17] = {};      // v2 encoding application
};

static inline uint32_t rd16(const uint8_t* p) { return p[0] | (p[1] << 8); }
static inline uint32_t rd32(const uint8_t* p) {
    return p[0] | (p[1] << 8) | (p[2] << 16) | ((uint32_t)p[3] << 24);
}

// `b` holds the first `n` bytes of the file. Needs 0x20 bytes for v1, 0x34 for
// v2 (the extension is skipped by offset, never read). Returns false for
// anything that is not a playable CSW.
static inline bool parseHeader(const uint8_t* b, size_t n, size_t fileSize, Header& h) {
    h = Header();
    if (n < HDR_MIN || memcmp(b, "Compressed Square Wave", 22) != 0 || b[0x16] != 0x1A)
        return false;
    h.major = b[0x17];
    h.minor = b[0x18];
    if (h.major == 1) {
        h.rate = rd16(b + 0x19);
        h.compression = b[0x1B];
        h.polarity = b[0x1C] & 1;
        h.dataStart = 0x20;
    } else if (h.major == 2) {
        if (n < 0x34) return false;
        h.rate = rd32(b + 0x19);
        h.pulses = rd32(b + 0x1D);
        h.compression = b[0x21];
        h.polarity = b[0x22] & 1;
        h.dataStart = 0x34 + b[0x23];
        memcpy(h.app, b + 0x24, 16);
        h.app[16] = 0;
        for (int i = 15; i >= 0 && (h.app[i] == ' ' || h.app[i] == 0); i--) h.app[i] = 0;
        for (int i = 0; i < 16; i++)
            if (h.app[i] && (h.app[i] < 0x20 || h.app[i] > 0x7E)) h.app[i] = '?';
    } else {
        return false;
    }
    if (h.rate == 0 || h.rate > 10000000) return false;
    if (h.compression != 1 && !(h.major == 2 && h.compression == 2)) return false;
    if (h.dataStart > fileSize) return false;
    return true;
}

// T-states per sample in Q16. The tape code counts 3.5 MHz T-states (TAP/TZX
// timings, the TZX CSW block), so this does too.
static inline uint32_t tPerSampleQ16(uint32_t rate, uint32_t clock = 3500000) {
    return (uint32_t)(((uint64_t)clock << 16) / rate);
}

// Pulse stream reader. The compressed data is pulled through `read` (returns the
// bytes delivered, 0 at the end); a Z-RLE stream is inflated through `z`, a
// caller-owned block (tinfl state + the 32 KB window) that it allocates once.
typedef int (*ReadFn)(void* ctx, uint8_t* buf, int n);

struct ZState {
    tinfl_decompressor dec;
    uint8_t dict[TINFL_LZ_DICT_SIZE];
};

struct Decoder {
    static constexpr int IN_SZ = 512;

    ReadFn   read = nullptr;
    void*    ctx = nullptr;
    ZState*  z = nullptr;          // nullptr = RLE
    uint8_t  in[IN_SZ];
    int      inPos = 0, inLen = 0;
    bool     inEof = false;
    // Z-RLE output window
    uint32_t outRd = 0, outAvail = 0, outWr = 0;
    bool     zDone = false, bad = false;
    // pulse -> T-state conversion
    uint32_t tpsQ16 = 0, frac = 0;
    uint32_t pulsesRead = 0;

    void begin(ReadFn r, void* c, ZState* zs, uint32_t rate) {
        read = r; ctx = c; z = zs;
        inPos = inLen = 0; inEof = false;
        outRd = outAvail = outWr = 0; zDone = false; bad = false;
        tpsQ16 = tPerSampleQ16(rate); frac = 0; pulsesRead = 0;
        if (z) tinfl_init(&z->dec);
    }

    bool fill() {
        if (inEof) return false;
        inPos = 0;
        inLen = read(ctx, in, IN_SZ);
        if (inLen <= 0) { inLen = 0; inEof = true; return false; }
        return true;
    }

    int rawByte() {
        if (inPos >= inLen && !fill()) return -1;
        return in[inPos++];
    }

    int zByte() {
        while (outAvail == 0) {
            if (zDone || bad) return -1;
            if (inPos >= inLen) fill();
            size_t inSz = (size_t)(inLen - inPos);
            size_t outSz = TINFL_LZ_DICT_SIZE - outWr;
            const mz_uint32 flags = TINFL_FLAG_PARSE_ZLIB_HEADER |
                                    (inEof ? 0 : TINFL_FLAG_HAS_MORE_INPUT);
            const tinfl_status st = tinfl_decompress(&z->dec, in + inPos, &inSz,
                                                     z->dict, z->dict + outWr, &outSz, flags);
            inPos += (int)inSz;
            outRd = outWr;
            outAvail = (uint32_t)outSz;
            outWr = (outWr + (uint32_t)outSz) & (TINFL_LZ_DICT_SIZE - 1);
            if (st == TINFL_STATUS_DONE) zDone = true;
            else if (st < 0) { bad = true; }
            else if (st == TINFL_STATUS_NEEDS_MORE_INPUT && inEof && outSz == 0) bad = true;
        }
        outAvail--;
        return z->dict[outRd++];
    }

    int byte() { return z ? zByte() : rawByte(); }

    // Next pulse in samples; false at the end of the stream.
    bool pulse(uint32_t& samples) {
        int b = byte();
        if (b < 0) return false;
        if (b == 0) {
            uint32_t v = 0;
            for (int i = 0; i < 4; i++) {
                const int c = byte();
                if (c < 0) return false;
                v |= (uint32_t)c << (8 * i);
            }
            samples = v ? v : 1;
        } else {
            samples = (uint32_t)b;
        }
        pulsesRead++;
        return true;
    }

    // Samples -> T-states, carrying the fraction so a long tape does not drift
    // (3500000/44100 = 79.37 T: truncating to 79 runs ~0.5% fast).
    uint32_t toT(uint32_t samples) {
        const uint64_t t = (uint64_t)samples * tpsQ16 + frac;
        frac = (uint32_t)(t & 0xFFFF);
        const uint64_t r = t >> 16;
        return r == 0 ? 1u : (r > 0xFFFFFFF0ull ? 0xFFFFFFF0u : (uint32_t)r);
    }
};

} // namespace csw
