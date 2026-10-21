/*

pico-speccy — the wavetable (PCM) half of the Yamaha YMF278B (OPL4), for
Mick's ZXM-MoonSound card (micklab.ru, 2015; archived at
github.com/alfishe/zxm-moonsound). The FM half of the chip is the YMF262 we
already emulate (OplFm on ports #C4-#C7); this class is the part the OPL3
does not have: 24 sample channels reading 8/12/16-bit PCM out of a 4 MB
address space (2 MB YRW801 GM ROM + 1 MB SRAM on this card), wave register
address/data on ports #7E/#7F.

The channel model — register layout, the wave-table header, the envelope
generator, LFO, panning, the level interpolation, pseudo reverb and damping —
is a port of ymfm's pcm_engine (Aaron Giles, BSD-3-Clause, see below).
Deliberate differences:

 - It runs at OUR output rate, not the chip's 44.1 kHz: a running Q16 count of
   chip samples decides how many envelope / LFO / level steps each output
   sample takes (1 or 2 at 31250 Hz), and the sample position advances by the
   chip step times that ratio. Pitch and envelope times are exact; the chip
   does no interpolation either, so nothing is lost but the 44.1 kHz grid.
 - Only the DO2 output (the mixed FM + PCM pair the card's DAC takes) is
   produced. Channels routed to DO1 (reg 0x68 bit 4) are silent, as on the
   card. The FM/PCM mix levels (regs 0xF8/0xF9) scale the FM buffer OplFm has
   already filled and the PCM sum, normalised so the FM part keeps exactly
   its plain-OPL3 level at the power-on mix (0xF8 = 0x1B).
 - A key-on/off and a channel recache take effect at the register write (the
   write itself sits at its exact sample position in the OPL write queue),
   where ymfm defers them to the next chip sample.
 - BUSY and LD (wave header load) never read as set: every write completes
   inside the OUT, like the other FM cores here.

ymfm license: Copyright (c) 2021, Aaron Giles. All rights reserved.
Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
1. Redistributions of source code must retain the above copyright notice,
   this list of conditions and the following disclaimer.
2. Redistributions in binary form must reproduce the above copyright notice,
   this list of conditions and the following disclaimer in the documentation
   and/or other materials provided with the distribution.
3. Neither the name of the copyright holder nor the names of its contributors
   may be used to endorse or promote products derived from this software
   without specific prior written permission.
THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE.

*/

#ifndef MoonPcm_h
#define MoonPcm_h

#include <stdint.h>

class MoonPcm {
public:
    // Card memory: the YRW801 ROM at 0x000000-0x1FFFFF, the 1 MB SRAM at
    // 0x200000-0x2FFFFF (mirrored at 0x300000-0x3FFFFF — the 22-bit bus has
    // room for 2 MB of RAM and only half is fitted). Two blocks: the ROM is read
    // in place (the firmware embeds it in flash), the SRAM is the caller's.
    static constexpr uint32_t ROM_SIZE  = 2u << 20;
    static constexpr uint32_t RAM_SIZE  = 1u << 20;
    static constexpr uint32_t MEM_BYTES = ROM_SIZE + RAM_SIZE;
    static constexpr uint32_t CHIP_RATE = 44100;   // 33.8688 MHz / 768
    static constexpr int      CHANNELS  = 24;

    // rom: ROM_SIZE bytes, read only (nullptr = silent ROM tones, RAM samples still
    // work); ram: RAM_SIZE bytes, owned by the caller.
    MoonPcm(const uint8_t* rom, uint8_t* ram);

    void setRate(uint32_t outRate);
    void reset();

    // NEW2 (FM reg 0x105 bit 1) gates every wave DATA write, as on the chip.
    void setNew2(bool on) { m_new2 = on; }
    bool new2() const     { return m_new2; }

    void    writeAddr(uint8_t a) { m_addr = a; }
    uint8_t addr() const         { return m_addr; }
    // Registers 2..6 (memory access + header bank) take effect at once; the
    // caller applies them outside the timed write queue.
    static bool immediate(uint8_t reg) { return reg >= 2 && reg <= 6; }
    void    write(uint8_t reg, uint8_t v);
    uint8_t read(uint8_t reg);

    // Adds the PCM channels into L/R[off .. off+n-1], which already hold the
    // FM half's output, applying the chip's FM/PCM mix levels; saturates.
    void gen(int16_t* L, int16_t* R, int n, int off);

    bool anyActive() const { return m_active != 0; }
    uint8_t memRead(uint32_t a) const {
        a &= 0x3FFFFF;
        if (a < ROM_SIZE) return m_rom ? m_rom[a] : 0;
        return m_ram[a & (RAM_SIZE - 1)];
    }

private:
    enum : uint8_t { EG_ATTACK = 1, EG_DECAY, EG_SUSTAIN, EG_RELEASE, EG_REVERB };

    struct Ch {
        uint32_t base, endpos, looppos, pos;   // pos/loop/end in .16 samples
        uint32_t step;                         // .16 per chip sample (cached)
        uint32_t lfo;
        uint32_t tl, tlTarget;                 // .10
        uint16_t env, sustain;
        uint16_t panL, panR;
        uint8_t  eg, format, keyOn, out1;
        uint8_t  rate[6];
        uint8_t  lfoStep, amDepth, pmDepth;
    };

    void     recache(int ch);
    void     loadWave(int ch);
    void     keyOnOff(int ch, bool on);
    void     clockEnv(Ch& c, int ch, uint32_t counter);
    int16_t  fetch(const Ch& c) const;
    uint32_t memAutoInc();
    void     flashCmd(uint32_t a, uint8_t v);
    uint8_t  flashIdByte(uint32_t a) const;
    void     mixRecalc();

    const uint8_t* m_rom;
    uint8_t*       m_ram;
    uint8_t  m_regs[256];
    Ch       m_ch[CHANNELS];
    uint32_t m_active;          // bit per channel: not yet quiet after release
    uint32_t m_envCounter;
    uint32_t m_acc;             // Q16 fraction of a chip sample
    uint32_t m_ratio;           // chip samples per output sample, Q16
    int32_t  m_fmFL, m_fmFR, m_pcmFL, m_pcmFR;   // mix factors, Q12 (FM 4096 = unchanged)
    uint8_t  m_addr;
    bool     m_new2;
    uint8_t  m_flashCyc;        // AM29F016 command cycle (flashCmd)
    bool     m_flashId;         // autoselect mode: ROM reads give the chip ID
};

// The one card, allocated in ESPectrum::setup while Config::moonsound is on and
// its 1 MB SRAM landed in butter PSRAM (16 MB firmware only, MOONSOUND=1);
// nullptr otherwise.
// The 4 MB firmware has no card at all (MOONSOUND=0): the pointer is a constant
// nullptr there, so every `if (moonpcm)` folds away with the code behind it.
// MOONSOUND undefined = a host build (tools/moonpcm_test.cpp).
#if !defined(MOONSOUND) || MOONSOUND
extern MoonPcm* moonpcm;
#else
constexpr MoonPcm* moonpcm = nullptr;
#endif

#endif
