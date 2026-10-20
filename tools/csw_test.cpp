// Host test for src/speccy/devices/tape/CswFormat.h (the .csw parser + pulse decoder).
//
//   gcc -O2 -c -Iexternal -o /tmp/miniz.o external/miniz/miniz.c
//   g++ -O2 -Wall -Wextra -Isrc -Iexternal -DMINIZ_NO_ZLIB_COMPATIBLE_NAMES \
//       -o /tmp/csw_test tools/csw_test.cpp /tmp/miniz.o -lz
//   /tmp/csw_test [file.csw ...]
//
// Files are built byte by byte here; the Z-RLE ones are compressed with the host
// zlib, not miniz, so a decoder bug cannot hide behind a matching encoder.
// With arguments it also decodes real files and prints their totals.

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <vector>
#include <string>
#include <zlib.h>
#include "speccy/devices/tape/CswFormat.h"

static int g_fail = 0;
#define CHECK(c, ...) do { if (!(c)) { printf("FAIL %s:%d: ", __FILE__, __LINE__); printf(__VA_ARGS__); printf("\n"); g_fail++; } } while (0)

typedef std::vector<uint8_t> Bytes;

static void put16(Bytes& b, uint32_t v) { b.push_back(v & 0xFF); b.push_back((v >> 8) & 0xFF); }
static void put32(Bytes& b, uint32_t v) { for (int i = 0; i < 4; i++) b.push_back((v >> (8 * i)) & 0xFF); }

static Bytes rleOf(const std::vector<uint32_t>& pulses) {
    Bytes r;
    for (uint32_t p : pulses) {
        if (p >= 1 && p <= 255) r.push_back((uint8_t)p);
        else { r.push_back(0); put32(r, p); }
    }
    return r;
}

static Bytes sig() {
    Bytes b((const uint8_t*)"Compressed Square Wave", (const uint8_t*)"Compressed Square Wave" + 22);
    b.push_back(0x1A);
    return b;
}

static Bytes makeV1(uint32_t rate, bool pol, const Bytes& rle) {
    Bytes b = sig();
    b.push_back(1); b.push_back(1);
    put16(b, rate);
    b.push_back(1);
    b.push_back(pol ? 1 : 0);
    b.push_back(0); b.push_back(0); b.push_back(0);
    b.insert(b.end(), rle.begin(), rle.end());
    return b;
}

static Bytes makeV2(uint32_t rate, uint32_t pulses, uint8_t comp, bool pol, const Bytes& data,
                    uint8_t ext = 0) {
    Bytes b = sig();
    b.push_back(2); b.push_back(0);
    put32(b, rate);
    put32(b, pulses);
    b.push_back(comp);
    b.push_back(pol ? 1 : 0);
    b.push_back(ext);
    const char* app = "csw_test        ";
    b.insert(b.end(), app, app + 16);
    for (int i = 0; i < ext; i++) b.push_back(0xEE);
    b.insert(b.end(), data.begin(), data.end());
    return b;
}

static Bytes zlibOf(const Bytes& in) {
    uLongf n = compressBound(in.size());
    Bytes out(n);
    compress2(out.data(), &n, in.data(), in.size(), 9);
    out.resize(n);
    return out;
}

// Read callback over a buffer, delivering at most `chunk` bytes per call (forces
// the decoder through many refills and window turns).
struct Src { const Bytes* b; size_t pos; int chunk; };
static int srcRead(void* c, uint8_t* buf, int n) {
    Src* s = (Src*)c;
    int k = (int)std::min<size_t>((size_t)std::min(n, s->chunk), s->b->size() - s->pos);
    memcpy(buf, s->b->data() + s->pos, k);
    s->pos += k;
    return k;
}

static csw::ZState g_z;

static bool decodeAll(const Bytes& file, std::vector<uint32_t>& out, csw::Header& h, int chunk,
                      csw::Decoder& d) {
    out.clear();
    if (!csw::parseHeader(file.data(), std::min<size_t>(file.size(), 0x34), file.size(), h)) return false;
    static Src s;
    s = Src{ &file, h.dataStart, chunk };
    d.begin(srcRead, &s, h.compression == 2 ? &g_z : nullptr, h.rate);
    uint32_t p;
    while (d.pulse(p)) out.push_back(p);
    return true;
}

static std::vector<uint32_t> samplePulses(int n, unsigned seed) {
    std::vector<uint32_t> v;
    srand(seed);
    for (int i = 0; i < n; i++) {
        int r = rand() % 100;
        if (r < 2) v.push_back(256 + rand() % 200000);   // long pause, extended form
        else if (r < 3) v.push_back(255);
        else v.push_back(1 + rand() % 60);               // ordinary edges
    }
    return v;
}

int main(int argc, char** argv) {
    // 1. v1 RLE, polarity, extended pulses.
    {
        std::vector<uint32_t> pulses = {27, 27, 9, 300, 255, 1, 70000};
        Bytes f = makeV1(44100, true, rleOf(pulses));
        csw::Header h; csw::Decoder d; std::vector<uint32_t> got;
        CHECK(decodeAll(f, got, h, 512, d), "v1 parse");
        CHECK(h.major == 1 && h.rate == 44100 && h.compression == 1 && h.polarity && h.dataStart == 0x20,
              "v1 header fields");
        CHECK(got == pulses, "v1 pulses (%zu vs %zu)", got.size(), pulses.size());
    }
    // 2. v2 RLE with a header extension.
    {
        std::vector<uint32_t> pulses = samplePulses(5000, 1);
        Bytes f = makeV2(22050, (uint32_t)pulses.size(), 1, false, rleOf(pulses), 7);
        csw::Header h; csw::Decoder d; std::vector<uint32_t> got;
        CHECK(decodeAll(f, got, h, 37, d), "v2 rle parse");
        CHECK(h.dataStart == 0x34 + 7 && h.pulses == pulses.size() && !h.polarity, "v2 header fields");
        CHECK(strcmp(h.app, "csw_test") == 0, "app '%s'", h.app);
        CHECK(got == pulses, "v2 rle pulses");
    }
    // 3. v2 Z-RLE, big enough to wrap the 32 KB window several times, at
    //    several input chunk sizes.
    {
        std::vector<uint32_t> pulses = samplePulses(300000, 2);
        Bytes rle = rleOf(pulses);
        CHECK(rle.size() > 4 * TINFL_LZ_DICT_SIZE, "test stream too small (%zu)", rle.size());
        Bytes f = makeV2(44100, (uint32_t)pulses.size(), 2, true, zlibOf(rle));
        const int chunks[] = {1, 7, 512, 100000};
        for (int c : chunks) {
            csw::Header h; csw::Decoder d; std::vector<uint32_t> got;
            CHECK(decodeAll(f, got, h, c, d), "z-rle parse");
            CHECK(got == pulses, "z-rle pulses chunk=%d (%zu vs %zu)", c, got.size(), pulses.size());
            CHECK(!d.bad, "z-rle bad flag chunk=%d", c);
        }
    }
    // 4. Truncated Z-RLE: decodes a prefix, then stops with bad set, no hang.
    {
        std::vector<uint32_t> pulses = samplePulses(20000, 3);
        Bytes z = zlibOf(rleOf(pulses));
        z.resize(z.size() / 2);
        Bytes f = makeV2(44100, 0, 2, false, z);
        csw::Header h; csw::Decoder d; std::vector<uint32_t> got;
        CHECK(decodeAll(f, got, h, 512, d), "trunc parse");
        CHECK(got.size() < pulses.size() && d.bad, "trunc: %zu pulses, bad=%d", got.size(), (int)d.bad);
        for (size_t i = 0; i + 1 < got.size(); i++)
            if (got[i] != pulses[i]) { CHECK(false, "trunc prefix differs at %zu", i); break; }
    }
    // 5. Header rejects.
    {
        Bytes good = makeV1(44100, false, rleOf({10}));
        csw::Header h;
        Bytes b = good; b[0] = 'X';
        CHECK(!csw::parseHeader(b.data(), b.size(), b.size(), h), "bad signature accepted");
        b = good; b[0x17] = 3;
        CHECK(!csw::parseHeader(b.data(), b.size(), b.size(), h), "v3 accepted");
        b = good; b[0x1B] = 2;
        CHECK(!csw::parseHeader(b.data(), b.size(), b.size(), h), "v1 Z-RLE accepted");
        b = good; b[0x19] = 0; b[0x1A] = 0;
        CHECK(!csw::parseHeader(b.data(), b.size(), b.size(), h), "rate 0 accepted");
        CHECK(!csw::parseHeader(good.data(), 0x1F, good.size(), h), "short header accepted");
        Bytes v2 = makeV2(44100, 1, 1, false, rleOf({10}), 200);
        CHECK(!csw::parseHeader(v2.data(), 0x34, 0x40, h), "data beyond file accepted");
    }
    // 6. T-state conversion carries the fraction: 44100 samples = 1 s = 3500000 T.
    {
        csw::Decoder d;
        d.begin(srcRead, nullptr, nullptr, 44100);
        uint64_t t = 0;
        for (int i = 0; i < 44100; i++) t += d.toT(1);
        CHECK(t >= 3499999 && t <= 3500001, "1 s of 1-sample pulses = %llu T", (unsigned long long)t);
        d.begin(srcRead, nullptr, nullptr, 44100);
        CHECK(d.toT(0x7FFFFFFF) == 0xFFFFFFF0u, "huge pulse not clamped");
    }

    // Real files from the command line.
    for (int i = 1; i < argc; i++) {
        FILE* fp = fopen(argv[i], "rb");
        if (!fp) { printf("%s: cannot open\n", argv[i]); continue; }
        Bytes f; uint8_t buf[65536]; size_t n;
        while ((n = fread(buf, 1, sizeof(buf), fp)) > 0) f.insert(f.end(), buf, buf + n);
        fclose(fp);
        csw::Header h; csw::Decoder d; std::vector<uint32_t> got;
        if (!decodeAll(f, got, h, 512, d)) { printf("%s: not a playable CSW\n", argv[i]); g_fail++; continue; }
        uint64_t samples = 0; for (uint32_t p : got) samples += p;
        printf("%s: v%u.%02u %s %u Hz pulses=%zu (hdr %u) %.1f s bad=%d app='%s'\n", argv[i],
               h.major, h.minor, h.compression == 2 ? "Z-RLE" : "RLE", (unsigned)h.rate, got.size(),
               (unsigned)h.pulses, (double)samples / h.rate, (int)d.bad, h.app);
        if (h.pulses && h.pulses != got.size()) { printf("  pulse count mismatch\n"); g_fail++; }
    }

    printf(g_fail ? "csw_test: %d FAILED\n" : "csw_test: OK\n", g_fail);
    return g_fail ? 1 : 0;
}
