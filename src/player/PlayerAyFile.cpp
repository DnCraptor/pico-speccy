// pico-speccy — Pico-Zx-Player: .ay (ZXAYEMUL) — music ripped together with the
// program's own Z80 player code (the Project AY archive format).
//
// Played the way every AY emulator plays it (the ZXAYEMUL / Ay_Emul
// convention): #0000-#00FF = RET, #0100-#3FFF = #FF, #4000-#FFFF = 0, EI at
// #0038, the song's blocks copied in, then a tiny driver at #0000
//     DI; CALL INIT; loop: IM 2; EI; HALT; JR loop                  (INTERRUPT = 0)
//     DI; CALL INIT; loop: IM 1; EI; HALT; CALL INTERRUPT; JR loop  (otherwise)
// on the player's own Z80 at the 48K speed (69888 T per 50 Hz frame, the INT
// held for the first 32 T). OUT (#FFFD) / (#BFFD) drive a private AySound at the
// ZX AY clock; an even port is the ULA, whose EAR/MIC bits are the BEEPER — a
// good third of the archive is beeper music, integrated here per T-state over
// every output sample (the emulator's own speaker_values levels). ZX Spectrum
// songs only: CPC rips reach the AY through the PPI and come out silent, which
// the silence cut-off then skips.
//
// Every sub-song plays in turn, starting at FirstSong; each lasts its own
// SongLength (1/50 s), 3 minutes when it gives none, and is cut short after
// 3 s of silence (the usual rule for rips that end by going quiet).

#include "PicoPlayer.h"
#include "PlayerZ80.h"
#include "Buffer.h"
#include "TryAlloc.h"
#include "Debug.h"
#include "AySound.h"

#include "ff.h"
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <new>

namespace pp {

namespace {

#ifndef PP_AY_INT_T
#define PP_AY_INT_T 32
#endif
#ifndef PP_AY_FRAME_T
#define PP_AY_FRAME_T 69888
#endif
constexpr int32_t  FRAME_T = PP_AY_FRAME_T;
constexpr int      FRAME = RATE / 50;                    // 625 samples
constexpr uint32_t DEFAULT_MS = 3 * 60 * 1000;
constexpr uint32_t MAX_FILE = 512 * 1024;
constexpr int      SILENCE_FRAMES = 150;                 // 3 s
// ESPectrum's beeper levels (Ports::speaker_values without the tape EAR bit),
// indexed by (EAR << 2) | MIC.
// The bits an AY register really holds (what IN reads back).
constexpr uint8_t  kAyMask[14] = { 0xFF, 0x0F, 0xFF, 0x0F, 0xFF, 0x0F, 0x1F, 0xFF, 0x1F, 0x1F, 0x1F, 0xFF, 0xFF, 0x0F };
constexpr uint8_t  kSpeaker[8] = { 0, 19, 0, 0, 97, 101, 0, 0 };

class AyFileDecoder : public Decoder {
public:
    ~AyFileDecoder() override { close(); }
    bool open(const char* path) override;
    int  render(int16_t* lr, int n) override;
    int  channels() const override { return 4; }
    const char* chanName(int i) const override { static const char* const nm[4] = { "A", "B", "C", "Bp" }; return nm[i]; }
    const char* groupName(int i) const override { return i == 0 ? "AY" : i == 3 ? "ULA" : nullptr; }
    void levels(uint8_t* out) override;
    uint32_t posMs() const override { return (uint32_t)((uint64_t)outFrames_ * 1000 / RATE); }
    uint32_t lenMs() const override { return lenMs_; }

    // touched by the Z80 callbacks
    uint8_t* ram_ = nullptr;
    Z80      cpu_;
    AySound* ay_ = nullptr;
    uint8_t  sel_ = 0, reg_[16] = {};
    int32_t  frameBase_ = 0;        // frame T at the start of the current z80_run
    uint8_t  beepLvl_ = 0;
    uint32_t beepToggles_ = 0;
    bool     ayVolHit_ = false;     // a non-zero volume was written this frame
    void beepAdvance(int32_t t);

private:
    uint8_t* file_ = nullptr;
    uint32_t size_ = 0;
    int      nSongs_ = 0, first_ = 0, played_ = 0;
    char     album_[64] = {};
    uint32_t outFrames_ = 0, lenMs_ = DEFAULT_MS;
    int      left_ = 0;
    bool     ended_ = false;
    int32_t  over_ = 0;              // T-states run past the previous frame end
    int      silent_ = 0;
    uint8_t  beepMeter_ = 0;
    int32_t  dcL_ = 0, dcR_ = 0;
    // beeper integration over one frame
    uint8_t  beep_[FRAME];
    int      beepIdx_ = 0;
    int32_t  beepLast_ = 0;
    uint32_t beepAcc_ = 0;

    void close();
    bool startSong(int k);
    void runFrame();
};

zuint8 cbRead(void* ctx, zuint16 a) { return ((AyFileDecoder*)ctx)->ram_[a]; }
void   cbWrite(void* ctx, zuint16 a, zuint8 v) { ((AyFileDecoder*)ctx)->ram_[a] = v; }
// IN (#FFFD) reads the selected AY register back (players that keep their state
// in the chip do read-modify-write); everything else is an idle bus.
zuint8 ayRead(void* ctx, zuint16 port) {
    const AyFileDecoder* d = (const AyFileDecoder*)ctx;
    return ((port & 0xC002) == 0xC000 && d->sel_ < 14) ? d->reg_[d->sel_] : 0xFF;
}
#ifdef PP_AY_TRACE
extern "C" void pp_ay_trace_in(zuint16 port);
extern "C" void pp_ay_trace_op(zuint16 pc, zuint8 op, zuint8 a);
zuint8 cbIn(void* ctx, zuint16 port) { pp_ay_trace_in(port); return ayRead(ctx, port); }
zuint8 cbFetchOp(void* ctx, zuint16 a) { zuint8 v = ((AyFileDecoder*)ctx)->ram_[a]; pp_ay_trace_op(a, v, Z80_A(((AyFileDecoder*)ctx)->cpu_)); return v; }
#else
zuint8 cbIn(void* ctx, zuint16 port) { return ayRead(ctx, port); }
#define cbFetchOp cbRead
#endif
void   cbOut(void* ctx, zuint16 port, zuint8 v) {
    AyFileDecoder* d = (AyFileDecoder*)ctx;
    if ((port & 0xC002) == 0xC000) d->sel_ = v;                         // #FFFD
    else if ((port & 0xC002) == 0x8000) {                               // #BFFD
        if (d->sel_ < 14) {
            d->reg_[d->sel_] = v & kAyMask[d->sel_];
            if (d->sel_ >= 8 && d->sel_ <= 10 && (v & 0x1F)) d->ayVolHit_ = true;  // digitised drums
            d->ay_->selectRegister(d->sel_);
            d->ay_->setRegisterData(v);
        }
    } else if (!(port & 1)) {                                           // ULA
        const uint8_t lvl = kSpeaker[((v >> 2) & 4) | ((v >> 3) & 1)];
        if (lvl != d->beepLvl_) {
            d->beepAdvance(d->frameBase_ + (int32_t)d->cpu_.cycles);
            d->beepLvl_ = lvl;
            d->beepToggles_++;
        }
    }
}
zuint8 cbInta(void*, zuint16) { return 0xFF; }

// ZXAYEMUL pointers: signed 16-bit big-endian, relative to their own address.
int32_t rel(const uint8_t* f, uint32_t size, uint32_t at) {
    if (at + 2 > size) return -1;
    const int16_t o = (int16_t)((f[at] << 8) | f[at + 1]);
    const int32_t t = (int32_t)at + o;
    return (t < 0 || (uint32_t)t >= size) ? -1 : t;
}
uint16_t be16(const uint8_t* p) { return (uint16_t)((p[0] << 8) | p[1]); }

// NUL-terminated string at a relative pointer (bounded by the file).
void relText(char* dst, size_t cap, const uint8_t* f, uint32_t size, uint32_t at) {
    dst[0] = 0;
    const int32_t p = rel(f, size, at);
    if (p < 0) return;
    size_t l = 0;
    while ((uint32_t)p + l < size && f[p + l] && l < 63) l++;
    textCopy(dst, cap, f + p, l, TE_CP1251);
}

// Integrates the beeper level from the last change up to frame time `t`,
// closing every output sample it passes.
void AyFileDecoder::beepAdvance(int32_t t) {
    if (t > FRAME_T) t = FRAME_T;
    while (beepIdx_ < FRAME) {
        const int32_t end = (int32_t)((int64_t)(beepIdx_ + 1) * FRAME_T / FRAME);
        if (t < end) {
            if (t > beepLast_) { beepAcc_ += (uint32_t)beepLvl_ * (uint32_t)(t - beepLast_); beepLast_ = t; }
            return;
        }
        const int32_t start = (int32_t)((int64_t)beepIdx_ * FRAME_T / FRAME);
        if (end > beepLast_) beepAcc_ += (uint32_t)beepLvl_ * (uint32_t)(end - beepLast_);
        beep_[beepIdx_++] = (uint8_t)(beepAcc_ / (uint32_t)(end - start));
        beepAcc_ = 0;
        beepLast_ = end;
    }
}

bool AyFileDecoder::open(const char* path) {
    FIL* f = (FIL*)tryMalloc(sizeof(FIL));
    uint8_t* bounce = (uint8_t*)tryMalloc(4096);
    bool ok = false;
    do {
        if (!f || !bounce) { err = "Out of memory"; break; }
        memset(f, 0, sizeof(FIL));
        if (f_open(f, path, FA_READ) != FR_OK) { err = "Cannot open file"; break; }
        size_ = f_size(f);
        if (size_ < 20 || size_ > MAX_FILE) { err = "Bad file size"; f_close(f); break; }
        file_ = (uint8_t*)Buffer::palloc(size_, Buffer::NEED_POINTER | Buffer::PREFER_PSRAM);
        if (!file_) { err = "Out of memory"; f_close(f); break; }
        uint32_t off = 0; UINT br = 0;
        while (off < size_) {
            const uint32_t want = size_ - off < 4096 ? size_ - off : 4096;
            if (f_read(f, bounce, want, &br) != FR_OK || br != want) break;
            memcpy(file_ + off, bounce, br);
            off += br;
        }
        f_close(f);
        if (off != size_) { err = "Read error"; break; }
        ok = true;
    } while (0);
    if (bounce) free(bounce);
    if (f) free(f);
    if (!ok) return false;

    if (memcmp(file_, "ZXAYEMUL", 8)) { err = "Not a ZXAYEMUL .ay file"; return false; }
    nSongs_ = file_[16] + 1;
    first_ = file_[17] < nSongs_ ? file_[17] : 0;
    const int32_t songs = rel(file_, size_, 18);
    if (songs < 0 || (uint32_t)songs + nSongs_ * 4 > size_) { err = "Bad song table"; return false; }
    relText(meta.author, sizeof(meta.author), file_, size_, 12);
    relText(album_, sizeof(album_), file_, size_, 14);           // "Misc": usually the game

    ram_ = (uint8_t*)Buffer::palloc(0x10000, Buffer::NEED_POINTER | Buffer::PREFER_PSRAM);
    void* a = tryMalloc(sizeof(AySound));
    if (!a) a = Buffer::palloc(sizeof(AySound), Buffer::NEED_POINTER | Buffer::PREFER_PSRAM);
    if (!ram_ || !a) { if (a) Buffer::pfree(a); err = "Out of memory"; return false; }
    ay_ = new (a) AySound(10);
    ay_->init();
    ay_->set_sound_format(RATE, 1, 8);
    ay_->set_stereo(AYEMU_ABC, nullptr);
    ay_->set_chip_freq(1773400);                 // the ZX 128 AY clock
    ay_->prepare_generation();

    for (int i = 0; i < nSongs_; i++)
        if (startSong((first_ + i) % nSongs_)) { played_ = i + 1; return true; }
    err = "No playable song";
    return false;
}

bool AyFileDecoder::startSong(int k) {
    const uint32_t se = (uint32_t)rel(file_, size_, 18) + (uint32_t)k * 4;
    const int32_t sd = rel(file_, size_, se + 2);
    if (sd < 0 || (uint32_t)sd + 14 > size_) return false;
    const uint8_t* s = file_ + sd;
    const int32_t pts = rel(file_, size_, (uint32_t)sd + 10), adr = rel(file_, size_, (uint32_t)sd + 12);
    if (pts < 0 || (uint32_t)pts + 6 > size_ || adr < 0) return false;
    const uint16_t stack = be16(file_ + pts), init0 = be16(file_ + pts + 2), intr = be16(file_ + pts + 4);

    memset(ram_, 0xC9, 0x100);
    memset(ram_ + 0x100, 0xFF, 0x3F00);
    memset(ram_ + 0x4000, 0x00, 0xC000);
    ram_[0x38] = 0xFB;
    // The driver goes in BEFORE the blocks (the format's own order): a rip may
    // carry a block at #0000 or #0038 that replaces it.
    uint16_t init = init0 ? init0 : be16(file_ + adr);   // INIT 0 = the first block
    if (!init) return false;
    uint8_t* c = ram_;
    c[0] = 0xF3; c[1] = 0xCD; c[2] = (uint8_t)init; c[3] = (uint8_t)(init >> 8);
    if (!intr) { c[4] = 0xED; c[5] = 0x5E; c[6] = 0xFB; c[7] = 0x76; c[8] = 0x18; c[9] = 0xFA; }
    else {
        c[4] = 0xED; c[5] = 0x56; c[6] = 0xFB; c[7] = 0x76;
        c[8] = 0xCD; c[9] = (uint8_t)intr; c[10] = (uint8_t)(intr >> 8);
        c[11] = 0x18; c[12] = 0xF7;
    }
    for (uint32_t b = (uint32_t)adr, n = 0; b + 6 <= size_ && n < 256; b += 6, n++) {
        const uint16_t at = be16(file_ + b);
        if (!at) break;
        uint32_t len = be16(file_ + b + 2);
        const int32_t off = rel(file_, size_, b + 4);
        if (off < 0) continue;
        if (at + len > 0x10000) len = 0x10000 - at;
        if ((uint32_t)off + len > size_) len = size_ - (uint32_t)off;
        memcpy(ram_ + at, file_ + off, len);
    }

    ay_->reset();
    memset(reg_, 0, sizeof(reg_));
    reg_[7] = 0xFF;
    sel_ = 0;
    memset(&cpu_, 0, sizeof(cpu_));
    cpu_.context = this;
    cpu_.fetch_opcode = cbFetchOp; cpu_.fetch = cbRead; cpu_.read = cbRead; cpu_.write = cbWrite;
    cpu_.in = cbIn; cpu_.out = cbOut; cpu_.nop = cbRead; cpu_.inta = cbInta;
    pp_z80_power(&cpu_, Z_TRUE);
    const uint16_t rr = (uint16_t)((s[8] << 8) | s[9]);        // HiReg:LoReg
    cpu_.af.uint16_value = cpu_.bc.uint16_value = cpu_.de.uint16_value = cpu_.hl.uint16_value = rr;
    cpu_.af_.uint16_value = cpu_.bc_.uint16_value = cpu_.de_.uint16_value = cpu_.hl_.uint16_value = rr;
    cpu_.ix_iy[0].uint16_value = cpu_.ix_iy[1].uint16_value = rr;
    cpu_.i = 0;                                   // I = 0, R = LoReg (ZXTune, Ay_Emul)
    cpu_.r = s[9];
    Z80_SP(cpu_) = stack;
    Z80_PC(cpu_) = 0;

    const uint16_t lenFr = be16(s + 4);
    lenMs_ = lenFr ? (uint32_t)lenFr * 20 : DEFAULT_MS;
    outFrames_ = 0; left_ = 0; over_ = 0; silent_ = 0;
    beepLvl_ = 0; beepToggles_ = 0; beepMeter_ = 0;
    relText(meta.title, sizeof(meta.title), file_, size_, se);
    snprintf(meta.album, sizeof(meta.album), "%s", album_);
    if (nSongs_ > 1) snprintf(meta.format, sizeof(meta.format), "AY (ZXAYEMUL) song %d of %d", k + 1, nSongs_);
    else snprintf(meta.format, sizeof(meta.format), "AY (ZXAYEMUL)");
    metaSeq++;
    return true;
}

void AyFileDecoder::close() {
    if (ay_) { ay_->~AySound(); Buffer::pfree(ay_); ay_ = nullptr; }
    if (ram_) { Buffer::pfree(ram_); ram_ = nullptr; }
    if (file_) { Buffer::pfree(file_); file_ = nullptr; }
}

// One 50 Hz frame: INT held for the first 32 T (the ULA's pulse), then the rest.
#ifdef PP_AY_TRACE
extern "C" void pp_ay_trace_frame();              // host test hook
#endif
void AyFileDecoder::runFrame() {
#ifdef PP_AY_TRACE
    pp_ay_trace_frame();
#endif
    beepIdx_ = 0; beepLast_ = 0; beepAcc_ = 0;
    const uint32_t toggles0 = beepToggles_;
    ayVolHit_ = false;
    frameBase_ = over_;
    pp_z80_int(&cpu_, Z_TRUE);
    frameBase_ += (int32_t)pp_z80_run(&cpu_, PP_AY_INT_T);
    pp_z80_int(&cpu_, Z_FALSE);
    while (frameBase_ < FRAME_T) frameBase_ += (int32_t)pp_z80_run(&cpu_, (zusize)(FRAME_T - frameBase_));
    over_ = frameBase_ - FRAME_T;
    frameBase_ = FRAME_T;
    beepAdvance(FRAME_T);

    const bool beeping = beepToggles_ != toggles0;
    beepMeter_ = beeping ? 200 : (uint8_t)(beepMeter_ * 3 / 4);
    bool ayOn = ayVolHit_;
    for (int ch = 0; ch < 3; ch++)
        if ((reg_[8 + ch] & 0x1F) && (((reg_[7] >> ch) & 1) == 0 || ((reg_[7] >> (ch + 3)) & 1) == 0)) ayOn = true;
    silent_ = (beeping || ayOn) ? 0 : silent_ + 1;
}

int AyFileDecoder::render(int16_t* lr, int n) {
    int o = 0;
    while (o < n) {
        if (left_ == 0) {
            if (ended_) break;
            const uint32_t pos = (uint32_t)((uint64_t)outFrames_ * 1000 / RATE);
            if (pos >= lenMs_ || (silent_ >= SILENCE_FRAMES && pos > 1000)) {
                bool next = false;
                while (played_ < nSongs_ && !next) next = startSong((first_ + played_++) % nSongs_);
                if (!next) { ended_ = true; break; }
            }
            runFrame();
            left_ = FRAME;
        }
        int c = n - o;
        if (c > left_) c = left_;
        if (c > 256) c = 256;
        const uint8_t* bp = beep_ + (FRAME - left_);
        ay_->gen_sound(c, 0);
        for (int i = 0; i < c; i++) {
            int32_t l = (int32_t)(ay_->SamplebufAY_L[i] + bp[i]) << 7, r = (int32_t)(ay_->SamplebufAY_R[i] + bp[i]) << 7;
            dcL_ += (int32_t)((((int64_t)l << 16) - dcL_) >> 9);
            dcR_ += (int32_t)((((int64_t)r << 16) - dcR_) >> 9);
            l -= dcL_ >> 16; r -= dcR_ >> 16;
            if (l > 32767) l = 32767; else if (l < -32768) l = -32768;
            if (r > 32767) r = 32767; else if (r < -32768) r = -32768;
            lr[2 * (o + i)] = (int16_t)l; lr[2 * (o + i) + 1] = (int16_t)r;
        }
        left_ -= c;
        o += c;
        outFrames_ += (uint32_t)c;
    }
    return o;
}

void AyFileDecoder::levels(uint8_t* out) {
    for (int ch = 0; ch < 3; ch++) {
        const bool on = ((reg_[7] >> ch) & 1) == 0 || ((reg_[7] >> (ch + 3)) & 1) == 0;
        const uint8_t v = reg_[8 + ch];
        out[ch] = !on ? 0 : (v & 0x10) ? 200 : (uint8_t)((v & 15) * 17);
    }
    out[3] = beepMeter_;
}

} // namespace

Decoder* createAyFileDecoder() { return new (std::nothrow) AyFileDecoder(); }

} // namespace pp
