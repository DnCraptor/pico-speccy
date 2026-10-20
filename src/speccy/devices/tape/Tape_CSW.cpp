// Tape_CSW.cpp — standalone .csw tapes (v1.01 RLE, v2.0 RLE and Z-RLE).
//
// A CSW is ONE stream of pulses, no blocks, so it plays through its own phase
// (TAPE_PHASE_CSWF) of Tape::Read: on every pulse expiry CSW_Step() toggles the
// ear bit and loads the next pulse. Nothing else of the block machinery runs.
//
// Z-RLE is inflated on the fly (CswFormat.h): the tinfl state + the 32 KB window
// are one Buffer block (PSRAM first), held while the tape is mounted. The TZX
// 0x18 block still inflates into a /tmp file (inflateCSW) — untouched.
//
// No fast load: there are no bytes in a CSW, only pulses, so the in-ROM trap and
// the turbo flash-loaders never engage (Z80_JLS gates them on TAP/TZX/PZX). The
// auto-run (LOAD "" typed by the loader snapshot) still works, then the tape plays
// in real time. Tape wear applies like to any other pulse format.
//
// Stop keeps the position: a CSW has nowhere else to resume from. Play after the
// end, or Enter in the tape browser, rewinds.

#include <stdio.h>
#include <string>
#include <new>

#include "Tape.h"
#include "CswFormat.h"
#include "fs/FileUtils.h"
#include "ui/OSDMain.h"
#include "app/Config.h"
#include "app/messages.h"
#include "app/Buffer.h"
#include "app/Debug.h"
#include "speccy/z80/CPU.h"

using namespace std;

namespace {

struct CswSession {
    csw::Header   hdr;
    csw::Decoder  dec;
    csw::ZState*  z = nullptr;
    uint32_t      curT = 0;      // length of the pulse being played, T
    uint8_t       level = 0;     // ear level of that pulse
    bool          ended = false;
};

CswSession* s_csw = nullptr;

int cswRead(void*, uint8_t* buf, int n) {
    UINT br = 0;
    if (f_read(&Tape::tape, buf, (UINT)n, &br) != FR_OK) return 0;
    return (int)br;
}

void cswFree() {
    if (!s_csw) return;
    if (s_csw->z) Buffer::pfree(s_csw->z);
    Buffer::pfree(s_csw);
    s_csw = nullptr;
}

// Back to the first pulse. False when the stream has none.
bool cswRewind() {
    if (!s_csw || !Tape::tape.obj.fs) return false;
    f_lseek(&Tape::tape, s_csw->hdr.dataStart);
    s_csw->dec.begin(cswRead, nullptr, s_csw->z, s_csw->hdr.rate);
    s_csw->ended = false;
    s_csw->level = s_csw->hdr.polarity ? 1 : 0;
    uint32_t smp;
    if (!s_csw->dec.pulse(smp)) { s_csw->ended = true; return false; }
    s_csw->curT = s_csw->dec.toT(smp);
    return true;
}

const char* compName(uint8_t c) { return c == 2 ? "Z-RLE" : "RLE"; }

} // namespace

void Tape::CSW_Close() {
    cswFree();
}

void Tape::CSW_Open(const string& name) {

    cswFree();
    if (tapeFileType != TAPE_FTYPE_EMPTY) {
        f_close(&tape);
        tapeFileType = TAPE_FTYPE_EMPTY;
    }

    const string fname = FileUtils::TAP_Path + name;   // TAP_Path already ends in '/'
    if (f_open(&tape, fname.c_str(), FA_READ) != FR_OK) {
        OSD::osdCenteredMsg(OSD_TAPE_LOAD_ERR, LEVEL_ERROR);
        return;
    }
    tapeFileSize = f_size(&tape);

    uint8_t hb[0x34];
    UINT br = 0;
    f_read(&tape, hb, sizeof(hb), &br);
    csw::Header h;
    if (!csw::parseHeader(hb, br, tapeFileSize, h)) {
        Debug::log("[CSW] %s: not a playable CSW (%u bytes read)", name.c_str(), (unsigned)br);
        OSD::osdCenteredMsg("Unsupported CSW file", LEVEL_ERROR);
        f_close(&tape);
        return;
    }

    s_csw = (CswSession*)Buffer::palloc(sizeof(CswSession),
                                        Buffer::NEED_POINTER | Buffer::PREFER_PSRAM);
    if (s_csw) {
        new (s_csw) CswSession();
        s_csw->hdr = h;
        if (h.compression == 2) {
            s_csw->z = (csw::ZState*)Buffer::palloc(sizeof(csw::ZState),
                                                    Buffer::NEED_POINTER | Buffer::PREFER_PSRAM);
            if (!s_csw->z) cswFree();
        }
    }
    if (!s_csw) {
        Debug::log("[CSW] no memory for the %s decoder", compName(h.compression));
        OSD::osdCenteredMsg("Not enough memory for CSW", LEVEL_ERROR);
        f_close(&tape);
        return;
    }

    tapeFileName = name;
    Config::tape_file = FileUtils::TAP_Path + name;   // remembered, re-mounted after F11/reboot

    TapeListing.clear();
    std::vector<TapeBlock>().swap(TapeListing);
    tapeCurBlock = 0;
    tapeNumBlocks = 1;
    tapebufByteCount = 0;
    tapePlayOffset = 0;
    tapeFileType = TAPE_FTYPE_CSW;

    if (!cswRewind()) {
        Debug::log("[CSW] %s: empty pulse stream", name.c_str());
        OSD::osdCenteredMsg("Empty CSW file", LEVEL_ERROR);
        Eject();
        return;
    }

    Debug::log("[CSW] %s: v%u.%02u %s %u Hz pulses=%u data@%u app='%s' pol=%d",
               name.c_str(), h.major, h.minor, compName(h.compression),
               (unsigned)h.rate, (unsigned)h.pulses, (unsigned)h.dataStart, h.app,
               (int)h.polarity);
}

// Play(): resume where Stop() left it, or rewind after the end.
bool Tape::CSW_Play() {
    if (!s_csw) return false;
    if (s_csw->ended && !cswRewind()) return false;
    GetBlock = &CSW_GetBlock;
    tapePhase = TAPE_PHASE_CSWF;
    tapeEarBit = s_csw->level;
    tapeNext = s_csw->curT;
    tapePlayOffset = 0;
    tapebufByteCount = (uint32_t)f_tell(&tape);
    return true;
}

// Called from Tape::Read when a pulse has run out.
void Tape::CSW_Step() {
    tapeEarBit ^= 1;
    uint32_t smp;
    if (!s_csw || !s_csw->dec.pulse(smp)) {
        if (s_csw) {
            s_csw->ended = true;
            if (s_csw->dec.bad)
                Debug::log("[CSW] corrupt Z-RLE stream after %u pulses",
                           (unsigned)s_csw->dec.pulsesRead);
        }
        tapeCurBlock = 0;
        Stop();
        tapeNext = 0xFFFFFFFF;
        return;
    }
    s_csw->curT = s_csw->dec.toT(smp);
    s_csw->level = tapeEarBit;
    tapeNext = s_csw->curT;
    tapebufByteCount = (uint32_t)f_tell(&tape);
}

// Never reached in normal play (the CSW phase does not use the block chain);
// anything that asks for "the next block" gets the end of the tape.
void Tape::CSW_GetBlock() {
    tapeCurBlock = 0;
    if (s_csw) s_csw->ended = true;
    Stop();
}

void Tape::CSW_RewindTo0() {
    if (!s_csw) return;
    const bool playing = tapeStatus == TAPE_LOADING;
    if (playing) Stop();
    cswRewind();
    tapebufByteCount = 0;
    if (playing) Play();
}

// One line for the tape browser's single row.
string Tape::cswBlockReadData() {
    char buf[48];
    if (!s_csw) return "CSW";
    const csw::Header& h = s_csw->hdr;
    const unsigned pct = tapeFileSize ? (unsigned)((uint64_t)f_tell(&tape) * 100 / tapeFileSize) : 0;
    snprintf(buf, sizeof(buf), "0001 CSW%u %s %uHz %3u%%%s", h.major, compName(h.compression),
             (unsigned)h.rate, pct, s_csw->ended ? " end" : "");
    return buf;
}
