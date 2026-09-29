// pico-speccy — Pico-Zx-Player page (Menu > Pico-Zx-Player, and Enter/F2 on a music
// file in the F5 browser).
//
// A fullscreen page in the nm:: chrome, like Pico-Scwong: its own key loop,
// drawn with the nm:: rasteriser (standard 8bpp and Profi DS80 alike). The
// emulation is paused behind the menu; the page owns core0.
//
// Audio: the decoder (src/player/) renders 31250 Hz stereo int16 into a ring
// that pcm_call_inner plays INSTEAD of the emulator's frame buffer and the live
// GS (pcm_player_attach). The ring is fed from this loop and from uiIdle() via
// uiIdleHook, so music keeps playing while the nested file browser is open.
// Leaving the page stops playback and hands the output back to the emulator.
//
// Playlist = the playable files of one folder AND its subfolders (8 levels
// deep, relative paths), sorted by path, names pooled in
// butter PSRAM. Autoplay walks it; shuffle picks at random (with a short
// history so Prev goes back). Session state only — nothing is written to NVS.

#include "OSDNewMenu.h"

#include "UiActions.h"
#include "UiBrowser.h"
#include "UiStrings.h"
#include "UiGfx.h"
#include "UiFont.h"
#include "UiRender.h"
#include "UiNav.h"
#include "ESPectrum.h"
#include "Config.h"
#include "Video.h"
#include "FileUtils.h"
#include "Buffer.h"
#include "TryAlloc.h"
#include "Debug.h"
#include "pwm_audio.h"
#include "player/PicoPlayer.h"
#if ZIFI_NET_CLIENT
#include "RemoteFs.h"
#include "OSDMain.h"
bool osdPlayerHasLocations();       // OSDMain.cpp: the F5 location chooser
bool osdPlayerLocations();
#endif

#include "ff.h"
#include "pico/time.h"
#include "pico/rand.h"
#include "hardware/sync.h"

#include <string>
#include <string.h>
#include <strings.h>
#include <stdio.h>
#include <math.h>
#include <algorithm>
#include <vector>
#include <utility>

using std::string;

extern size_t getFreeHeap(void);   // OSDMain.cpp

namespace nm {

// ── session state (survives leaving the page, not a reboot) ──────────────────
static string  s_dir;                  // folder of the playlist, ends with '/'
static string  s_cur;                  // name of the current track (for re-entry)
static int     s_vol = 14;             // 0..20, 16 = 0 dB
static bool    s_shuffle = false;
static bool    s_auto = true;
static bool    s_repeat = false;       // replay the current track when it ends
static bool    s_legend = false;        // F1: show / hide the hot-key legend column
static string  s_curPath;              // file the current decoder was opened from
static bool    s_quitReq = false;      // the browser handed over to a network flow
#if ZIFI_NET_CLIENT
// Non-null while the playlist is a remote folder (Web catalog / FTP): entries
// are "display\0basename\0" and each track is downloaded to /tmp before it
// plays. Only for the length of one playerRemote() call.
static RemoteFs* s_rfs = nullptr;
#endif

namespace {

constexpr int RING_FRAMES_BIG   = 4096;   // ~131 ms
constexpr int RING_FRAMES_SMALL = 2048;
constexpr int FEED = 256;                 // frames per render call
// Playlist capacity: tried big first, halved while the PSRAM arena says no
// (a 4 MB NeoGS or a big GM.DLS bank share it). ~64 B of path per track.
constexpr int MAX_TRACKS_BIG = 16384;
constexpr int MAX_TRACKS_MIN = 1024;
int MAX_TRACKS = 0;                     // capacity of the playlist being built
int NAME_POOL = 0;
constexpr int HIST = 32;
constexpr int PL_MAX_DEPTH = 8;         // subfolder levels walked into the playlist

struct Engine {
    int16_t*     ring = nullptr;
    uint32_t     frames = 0;
    int16_t*     tmp = nullptr;          // FEED stereo frames
    pp::Decoder* dec = nullptr;
    bool         paused = false;
    bool         muted = false;
    bool         ended = false;          // decoder returned 0
    int32_t      gainQ8 = 256;
    bool         busy = false;           // re-entrancy guard (uiIdleHook)
    int32_t      seekTo = -1;            // ms: a seek in progress (feed() stands aside)
};
Engine E;

struct Playlist {
    char*     pool = nullptr;
    uint32_t* off = nullptr;
    int       n = 0;
    int       cur = -1;
    int       hist[HIST];
    int       nh = 0;
    const char* name(int i) const { return pool + off[i]; }
    // The real file name (extension!) — differs from name() only for a remote
    // playlist, whose display names come from the catalog without an extension.
    const char* base(int i) const {
        const char* n = pool + off[i];
#if ZIFI_NET_CLIENT
        if (s_rfs) return n + strlen(n) + 1;
#endif
        return n;
    }
};
Playlist P;

int32_t volGain(int v) {
    if (v <= 0) return 0;
    return (int32_t)(256.0f * powf(10.0f, (float)(v - 16) * 1.5f / 20.0f) + 0.5f);   // 1.5 dB steps
}

// ── audio feeding ─────────────────────────────────────────────────────────────
void feed() {
    if (E.busy || !E.ring || !E.dec || E.paused || E.ended || E.seekTo >= 0) return;
    E.busy = true;
    uint32_t w = pcm_player_w;
    for (int guard = 0; guard < 64; guard++) {
        const uint32_t used = w - pcm_player_r;
        if (E.frames - used < (uint32_t)FEED) break;
        const int n = E.dec->render(E.tmp, FEED);
        if (n <= 0) { E.ended = true; break; }
        const int32_t g = E.muted ? 0 : E.gainQ8;
        const uint32_t m = E.frames - 1;
        for (int i = 0; i < n; i++) {
            int32_t l = (E.tmp[2 * i] * g) >> 8, r = (E.tmp[2 * i + 1] * g) >> 8;
            if (l > 32767) l = 32767; else if (l < -32768) l = -32768;
            if (r > 32767) r = 32767; else if (r < -32768) r = -32768;
            const uint32_t k = ((w + i) & m) << 1;
            E.ring[k] = (int16_t)l; E.ring[k + 1] = (int16_t)r;
        }
        __dmb();
        w += (uint32_t)n;
        pcm_player_w = w;
    }
    E.busy = false;
}

void idleHook() { feed(); }

bool ringDrained() { return pcm_player_w == pcm_player_r; }
void ringFlush()   { pcm_player_r = pcm_player_w; }

bool engineStart() {
    E.frames = RING_FRAMES_BIG;
    const size_t tmpBytes = FEED * 4;
    E.ring = nullptr;
#if ZIFI_NET_CLIENT
    // A remote playlist downloads every track over HTTPS, and mbedTLS wants its
    // ~20 KB from the heap — leave the heap to it and put the ring in PSRAM.
    if (s_rfs)
        E.ring = (int16_t*)Buffer::palloc(E.frames * 4 + tmpBytes, Buffer::NEED_POINTER | Buffer::PREFER_PSRAM);
#endif
    if (!E.ring) E.ring = (int16_t*)tryMalloc(E.frames * 4 + tmpBytes);
    if (!E.ring) {
        E.frames = RING_FRAMES_SMALL;
        E.ring = (int16_t*)tryMalloc(E.frames * 4 + tmpBytes);
    }
    if (!E.ring) {
        E.frames = RING_FRAMES_BIG;
        E.ring = (int16_t*)Buffer::palloc(E.frames * 4 + tmpBytes, Buffer::NEED_POINTER | Buffer::PREFER_PSRAM);
    }
    if (!E.ring) return false;
    E.tmp = E.ring + E.frames * 2;
    E.gainQ8 = volGain(s_vol);
    pcm_player_attach(E.ring, E.frames);
    uiIdleHook = idleHook;
    return true;
}

void engineStop() {
    uiIdleHook = nullptr;
    pcm_player_detach();
    if (E.dec) { delete E.dec; E.dec = nullptr; }
    if (E.ring) { Buffer::pfree(E.ring); E.ring = nullptr; }
    E.tmp = nullptr;
}

// ── playlist ──────────────────────────────────────────────────────────────────
// ── lazy folder scan ─────────────────────────────────────────────────────────
// The playlist is built incrementally from the page loop: a big tree (4096
// tracks) takes seconds of directory reads, and the track the user picked must
// not wait for it. Order = depth-first, a folder's own files (sorted) before its
// subfolders (sorted). P.n counts only COMPLETED folders — the folder being read
// is appended past it and published sorted in one go — so every index the page
// already holds (P.cur, the history) stays valid while the list grows.
struct PlScan {
    bool     active = false;
    string   base;                                // playlist folder, ends with '/'
    FILINFO* fi = nullptr;
    DIR*     d = nullptr;                         // the folder being read, or null
    string   rel;                                 // its path relative to base
    int      depth = 0;
    int      nTot = 0;                            // entries incl. the unpublished folder
    uint32_t used = 0;                            // name-pool bytes
    uint32_t seen = 0;
    string   subs;                                // its subfolders, NUL-separated
    std::vector<std::pair<string, int>> stack;    // folders still to read (DFS)
    string   pending;                             // playing by path, index not known yet
};
PlScan SC;

void plScanClose() {
    if (SC.d)  { f_closedir(SC.d); free(SC.d); SC.d = nullptr; }
    if (SC.fi) { free(SC.fi); SC.fi = nullptr; }
    SC.stack.clear(); SC.stack.shrink_to_fit();
    SC.subs.clear(); SC.subs.shrink_to_fit();
    SC.active = false;
}

void plFree() {
    plScanClose();
    SC.pending.clear();
    if (P.pool) { Buffer::pfree(P.pool); P.pool = nullptr; }
    if (P.off)  { Buffer::pfree(P.off);  P.off = nullptr; }
    P.n = 0; P.cur = -1; P.nh = 0;
}

// Name pool + offset table for up to MAX_TRACKS_BIG tracks, or as many as fit.
void plAlloc() {
    for (int t = MAX_TRACKS_BIG; t >= MAX_TRACKS_MIN && !P.off; t /= 2) {
        P.pool = (char*)Buffer::palloc((size_t)t * 64, Buffer::NEED_POINTER | Buffer::PREFER_PSRAM);
        if (!P.pool) continue;
        P.off = (uint32_t*)Buffer::palloc((size_t)t * 4, Buffer::NEED_POINTER | Buffer::PREFER_PSRAM);
        if (!P.off) { Buffer::pfree(P.pool); P.pool = nullptr; continue; }
        MAX_TRACKS = t; NAME_POOL = t * 64;
    }
    if (P.off) Debug::log("Player: playlist capacity %d tracks", MAX_TRACKS);
}

// The folder just read is complete: sort it, publish it, queue its subfolders.
void plFolderDone() {
    char* pool = P.pool;
    std::sort(P.off + P.n, P.off + SC.nTot, [pool](uint32_t a, uint32_t b) {
        return strcasecmp(pool + a, pool + b) < 0;
    });
    const int first = P.n;
    P.n = SC.nTot;
    if (!SC.pending.empty()) {
        for (int i = first; i < P.n; i++)
            if (SC.pending == P.name(i)) { P.cur = i; SC.pending.clear(); break; }
    }
    std::vector<string> v;
    for (size_t i = 0; i < SC.subs.size(); ) {
        const size_t e = SC.subs.find('\0', i);
        v.push_back(SC.subs.substr(i, e - i) + "/");   // "a/" vs "a-b/": '/' sorts as a name char
        i = e + 1;
    }
    SC.subs.clear();
    std::sort(v.begin(), v.end(), [](const string& x, const string& y) {
        return strcasecmp(x.c_str(), y.c_str()) < 0;
    });
    for (auto it = v.rbegin(); it != v.rend(); ++it) SC.stack.emplace_back(SC.rel + *it, SC.depth + 1);
}

// Reads directory entries for up to `budgetUs`; returns false once the walk is done.
bool plScanStep(uint32_t budgetUs) {
    if (!SC.active) return false;
    const uint64_t t0 = time_us_64();
    while (time_us_64() - t0 < budgetUs) {
        if (!SC.d) {
            if (SC.stack.empty() || SC.nTot >= MAX_TRACKS) break;
            SC.rel = SC.stack.back().first; SC.depth = SC.stack.back().second;
            SC.stack.pop_back();
            SC.d = (DIR*)tryMalloc(sizeof(DIR));
            if (!SC.d) break;
            if (f_opendir(SC.d, (SC.base + SC.rel).c_str()) != FR_OK) { free(SC.d); SC.d = nullptr; continue; }
        }
        bool eod = false;
        for (int k = 0; k < 16; k++) {                    // a burst, then re-check the clock
            if (SC.nTot >= MAX_TRACKS || f_readdir(SC.d, SC.fi) != FR_OK || !SC.fi->fname[0]) { eod = true; break; }
            if ((++SC.seen & 15) == 0) feed();
            const FILINFO* fi = SC.fi;
            if (fi->fattrib & (AM_HID | AM_SYS)) continue;
            if (fi->fname[0] == '.') continue;
            if (fi->fattrib & AM_DIR) {
                if (SC.depth < PL_MAX_DEPTH) { SC.subs += fi->fname; SC.subs += '\0'; }
                continue;
            }
            const string nm = SC.rel + fi->fname;
            if (!pp::playableExt(FileUtils::getLCaseExt(nm))) continue;
            const uint32_t l = (uint32_t)nm.size() + 1;
            if (SC.used + l > (uint32_t)NAME_POOL) { SC.stack.clear(); eod = true; break; }
            memcpy(P.pool + SC.used, nm.c_str(), l);
            P.off[SC.nTot++] = SC.used;
            SC.used += l;
        }
        if (eod) {                                        // folder read to its end
            f_closedir(SC.d); free(SC.d); SC.d = nullptr;
            plFolderDone();
        }
    }
    if (!SC.d && (SC.stack.empty() || SC.nTot >= MAX_TRACKS)) {
        Debug::log("Player: playlist %s: %d tracks", SC.base.c_str(), P.n);
        plScanClose();
        SC.pending.clear();                               // not in the list: keeps playing unnumbered
        return false;
    }
    return true;
}

// Starts the walk of `dir`. Returns at once; the page loop calls plScanStep().
bool plBuild(const string& dir) {
    plFree();
    plAlloc();
    SC.fi  = (FILINFO*)tryMalloc(sizeof(FILINFO));
    if (!P.pool || !P.off || !SC.fi) { plFree(); return false; }
    SC.base = dir;
    if (SC.base.empty() || SC.base.back() != '/') SC.base += '/';
    SC.nTot = 0; SC.used = 0; SC.seen = 0; SC.rel.clear(); SC.depth = 0;
    SC.stack.emplace_back(string(), 0);
    SC.active = true;
    return true;
}

// Scans until at least one track is published (or the walk ends): what
// pickFirst() needs before it can pick anything.
void plWaitFirst() {
    while (!P.n && plScanStep(20000)) {}
}

#if ZIFI_NET_CLIENT
// Remote playlist: the playable files of the remote cwd, in the source's own
// order (the catalog is pre-sorted; FTP/SFTP get a name sort).
struct PlRemote { uint32_t used; };
bool plBuildRemote(RemoteFs* fs) {
    plFree();
    plAlloc();
    if (!P.pool || !P.off) { plFree(); return false; }
    PlRemote R{ 0 };
    fs->listFiles([](void* c, const char* disp, const char* base) {
        PlRemote& R = *(PlRemote*)c;
        if (P.n >= MAX_TRACKS) return;
        if (!pp::playableExt(FileUtils::getLCaseExt(base))) return;
        const uint32_t ld = (uint32_t)strlen(disp) + 1, lb = (uint32_t)strlen(base) + 1;
        if (R.used + ld + lb > (uint32_t)NAME_POOL) return;
        memcpy(P.pool + R.used, disp, ld);
        memcpy(P.pool + R.used + ld, base, lb);
        P.off[P.n++] = R.used;
        R.used += ld + lb;
        if ((P.n & 15) == 0) feed();
    }, &R);
    if (!fs->preSorted()) {
        char* pool = P.pool;
        std::sort(P.off, P.off + P.n, [pool](uint32_t a, uint32_t b) {
            return strcasecmp(pool + a, pool + b) < 0;
        });
    }
    Debug::log("Player: remote playlist %s: %d tracks", fs->cwdPath().c_str(), P.n);
    return true;
}
#endif

// A playlist entry as the UI font draws it: local names already are CP1251
// (FatFs's code page), a catalog playlist carries UTF-8 display names.
string dispName(int i) {
#if ZIFI_NET_CLIENT
    if (s_rfs && s_rfs->utf8Names()) return FileUtils::utf8ToCp1251(P.name(i));
#endif
    return P.name(i);
}

int plFind(const string& name) {
    for (int i = 0; i < P.n; i++) if (name == P.name(i)) return i;
    return -1;
}

bool playIndex(int i);
void playReset();
bool playOpen(const string& path, const char* disp);

// Plays the playlist entry `name` (relative to s_dir). While the folder scan is
// still running the entry may not be listed yet: the file is then opened by its
// path right away and gets its index when the scan reaches it (plFolderDone).
bool playName(const string& name) {
    const int i = plFind(name);
    if (i >= 0) return playIndex(i);
    if (!SC.active || name.empty()) return false;
    const string path = s_dir + name;
    FILINFO fi;
    if (f_stat(path.c_str(), &fi) != FR_OK || (fi.fattrib & AM_DIR)) return false;
    playReset();
    P.cur = -1;
    s_cur = name;
    SC.pending = name;
    return playOpen(path, name.c_str());
}

// ── layout ────────────────────────────────────────────────────────────────────
constexpr int kLegendCols = 13;       // key column 5 ("PgUp ") + the widest label ("Shuffle")

struct Lay {
    int sc, m, hdr_h;
    int cw;            // content width: the hot-key legend column starts here
    int lgY;           // first legend row
    int infoY;         // first info row
    int timeY;
    int grpY;          // group labels row
    int mtrY0, mtrY1;  // meter bars area (inclusive-exclusive)
    int lblY;          // channel labels row
    int plY;           // playlist rows
    int plRows;
};
Lay L;

void layout() {
    L.sc = Sf.glyphScale;
    L.m = 4 * L.sc;
    L.hdr_h = UI_FONT_H + 6;
    // Right-hand hot-key legend, like the F5 browser's info pane: "key  what".
    L.cw = s_legend ? Sf.w - (kLegendCols * glyphW() + 3 * L.m) : Sf.w;
    L.lgY = L.hdr_h + 4;
    L.infoY = L.hdr_h + 4;
    L.timeY = L.infoY + 5 * UI_FONT_H + 2;
    L.plRows = 5;
    // Legend hidden: one bottom row is kept for the "F1 Help" hint.
    L.plY = Sf.h - 2 - L.plRows * UI_FONT_H - (s_legend ? 0 : UI_FONT_H);
    L.lblY = L.plY - 4 - UI_FONT_H;
    L.grpY = L.timeY + UI_FONT_H + 6;
    L.mtrY0 = L.grpY + UI_FONT_H;
    L.mtrY1 = L.lblY - 1;
}

// Meter state
uint8_t s_lv[pp::MAX_CH], s_disp[pp::MAX_CH], s_peak[pp::MAX_CH], s_hold[pp::MAX_CH];
int     s_drawnH[pp::MAX_CH], s_drawnP[pp::MAX_CH];
int     s_barX0 = 0, s_slot = 0, s_barW = 0, s_nch = 0;

string  s_msg;                          // error / status line
uint32_t s_msgUntil = 0;

const char* kTitle = "PICO-ZX-PLAYER";

// The legend: one row per key; a toggle's label is lit while it is on.
void drawLegend() {
    if (!s_legend) {
        const char* hint = "F1 Help";
        const int y = Sf.h - 2 - UI_FONT_H;
        fill(0, y, Sf.w, UI_FONT_H, C_PANEL);
        text(Sf.w - L.m - textWidth(hint), y, hint, C_TEXT_DIM);
        return;
    }
    const int x = L.cw, w = Sf.w - x;
    fill(x, L.hdr_h, w, Sf.h - L.hdr_h, C_PANEL);
    vline(x, L.hdr_h + 2, Sf.h - L.hdr_h - 4, C_SEP);
    struct Row { const char* k; const char* what; int on; };   // on: -1 = not a toggle
    const Row rows[] = {
        { "Spc",            "Pause",   E.paused ? 1 : 0 },
        { SYM_UP SYM_DOWN,  "Track",   -1 },
        { SYM_LEFT SYM_RIGHT, "Seek",  -1 },
        { "PgUp",           "Prev dir", -1 },
        { "PgDn",           "Next dir", -1 },
        { "F9",             "Vol -",   -1 },
        { "F10",            "Vol +",   -1 },
        { "M",              "Mute",    E.muted ? 1 : 0 },
        { "S",              "Shuffle", s_shuffle ? 1 : 0 },
        { "A",              "Auto",    s_auto ? 1 : 0 },
        { "R",              "Repeat",  s_repeat ? 1 : 0 },
        { "Hm",             "First",   -1 },
        { "End",            "Last",    -1 },
        { "F5",             "Files",   -1 },
        { "F1",             "Help",    -1 },
        { "Esc",            "Close",   -1 },
    };
    const int lh = UI_FONT_H + 2;
    const int kx = x + 2 * L.m, vx = kx + 5 * glyphW();
    int y = L.lgY;
    for (const Row& r : rows) {
        if (y + UI_FONT_H > Sf.h - 2) break;
        text(kx, y, r.k, C_TEXT_DIM);
        text(vx, y, r.what, r.on > 0 ? C_ACCENT : C_TEXT);
        y += lh;
    }
}

void drawHeader() {
    fill(0, 0, Sf.w, L.hdr_h, C_PANEL_ALT);
    text(L.m, 3, kTitle, C_WHITE);
    char s[64];
    // While the folder scan runs the count is still growing: "LOADING  12/345+".
    snprintf(s, sizeof(s), "%s%s%s%s%sVOL %d  %d/%d%s",
             // blinks at ~2 Hz; the blank phase keeps the width so nothing shifts
             SC.active ? ((time_us_64() / 400000) & 1 ? "         " : "LOADING  ") : "",
             E.muted ? "MUTE  " : "", E.paused ? "PAUSE  " : "",
             s_shuffle ? "SHUF  " : "", s_repeat ? "REP  " : s_auto ? "AUTO  " : "",
             s_vol, P.cur >= 0 ? P.cur + 1 : 0, P.n, SC.active ? "+" : "");
    text(Sf.w - L.m - textWidth(s), 3, s, C_TEXT);
}

void infoRow(int row, const char* label, const char* val) {
    const int y = L.infoY + row * UI_FONT_H;
    fill(0, y, L.cw, UI_FONT_H, C_PANEL);
    text(L.m, y, label, C_TEXT_DIM);
    const int x = L.m + 7 * glyphW();
    textClip(x, y, L.cw - x - L.m, val, C_TEXT);
}

void drawInfo() {
    const pp::Meta* mt = E.dec ? &E.dec->meta : nullptr;
    infoRow(0, "File", P.cur >= 0 ? dispName(P.cur).c_str() : !SC.pending.empty() ? SC.pending.c_str() : "-");
    if (!s_msg.empty()) {
        infoRow(1, "Error", s_msg.c_str());
    } else {
        infoRow(1, "Format", mt && mt->format[0] ? mt->format : "-");
    }
    infoRow(2, "Title",  mt && mt->title[0]  ? mt->title  : "");
    infoRow(3, "Author", mt && mt->author[0] ? mt->author : "");
    char a[96];
    snprintf(a, sizeof(a), "%s%s%s", mt ? mt->album : "", mt && mt->album[0] && mt->extra[0] ? " - " : "",
             mt ? mt->extra : "");
    infoRow(4, "Album", a);
}

// The position shown: a seek in progress shows where it is going.
uint32_t shownPosMs() {
    if (!E.dec) return 0;
    return E.seekTo >= 0 ? (uint32_t)E.seekTo : E.dec->posMs();
}

void fmtTime(char* s, size_t cap, uint32_t ms) {
    const uint32_t t = ms / 1000;
    snprintf(s, cap, "%u:%02u", (unsigned)(t / 60), (unsigned)(t % 60));
}

void drawTime() {
    const int y = L.timeY;
    fill(0, y, L.cw, UI_FONT_H, C_PANEL);
    char a[16], b[16], s[40];
    const uint32_t pos = shownPosMs(), len = E.dec ? E.dec->lenMs() : 0;
    fmtTime(a, sizeof(a), pos);
    if (len) { fmtTime(b, sizeof(b), len); snprintf(s, sizeof(s), "%s / %s", a, b); }
    else snprintf(s, sizeof(s), "%s", a);
    text(L.m, y, s, C_WHITE);
    const int bx = L.m + 14 * glyphW(), bw = L.cw - bx - L.m;
    if (bw > 8) {
        frame(bx, y + 2, bw, UI_FONT_H - 4, C_SEP);
        if (len) {
            uint32_t f = (uint32_t)((uint64_t)(bw - 2) * (pos < len ? pos : len) / len);
            fill(bx + 1, y + 3, (int)f, UI_FONT_H - 6, C_ACCENT);
        }
    }
}

void meterLayout() {
    s_nch = E.dec ? E.dec->channels() : 0;
    const int avail = L.cw - 2 * L.m;
    fill(0, L.grpY, L.cw, L.plY - L.grpY - 2, C_PANEL);
    if (!s_nch) return;
    s_slot = avail / s_nch;
    const int maxSlot = 28 * L.sc;
    if (s_slot > maxSlot) s_slot = maxSlot;
    const int gap = s_slot >= 6 * L.sc ? 2 * L.sc : L.sc;
    s_barW = s_slot - gap;
    if (s_barW < 1) s_barW = 1;
    s_barX0 = L.m + (avail - s_slot * s_nch) / 2;
    for (int i = 0; i < s_nch; i++) {
        s_lv[i] = s_disp[i] = s_peak[i] = s_hold[i] = 0;
        s_drawnH[i] = s_drawnP[i] = -1;
        const int x = s_barX0 + i * s_slot;
        // groups
        if (const char* g = E.dec->groupName(i)) {
            int j = i + 1;
            while (j < s_nch && !E.dec->groupName(j)) j++;
            textClip(x, L.grpY, (j - i) * s_slot - 1, g, C_TEXT_DIM);
        }
        const char* nm = E.dec->chanName(i);
        const int tw = textWidth(nm);
        if (nm && tw <= s_slot) text(x + (s_barW - tw) / 2, L.lblY, nm, C_TEXT_DIM);
        fill(x, L.mtrY0, s_barW, L.mtrY1 - L.mtrY0, C_PANEL_ALT);
    }
}

void drawMeter() {
    if (!E.dec || !s_nch) return;
    if (!E.paused) E.dec->levels(s_lv);
    else memset(s_lv, 0, sizeof(s_lv));
    const int H = L.mtrY1 - L.mtrY0;
    for (int i = 0; i < s_nch; i++) {
        int d = s_disp[i] > 14 ? s_disp[i] - 14 : 0;
        if (s_lv[i] > d) d = s_lv[i];
        s_disp[i] = (uint8_t)d;
        if (d >= s_peak[i]) { s_peak[i] = (uint8_t)d; s_hold[i] = 18; }
        else if (s_hold[i]) s_hold[i]--;
        else s_peak[i] = s_peak[i] > 5 ? s_peak[i] - 5 : 0;

        const int h = d * H / 255;
        const int ph = s_peak[i] * H / 255;
        if (h == s_drawnH[i] && ph == s_drawnP[i]) continue;
        const int x = s_barX0 + i * s_slot;
        fill(x, L.mtrY0, s_barW, H - h, C_PANEL_ALT);
        const int g1 = H * 60 / 100, g2 = H * 85 / 100;       // green / yellow / red
        const int top = L.mtrY1 - h;
        for (int y = top; y < L.mtrY1; ) {
            const int lvl = L.mtrY1 - y;                         // height from the bottom
            UiColor c; int next;
            if (lvl > g2)      { c = C_ICON_R; next = L.mtrY1 - g2; }
            else if (lvl > g1) { c = C_ICON_Y; next = L.mtrY1 - g1; }
            else               { c = C_ACCENT; next = L.mtrY1; }
            if (next <= y) next = y + 1;
            fill(x, y, s_barW, next - y, c);
            y = next;
        }
        if (ph > h && ph > 0) fill(x, L.mtrY1 - ph, s_barW, 1, C_WHITE);
        s_drawnH[i] = h; s_drawnP[i] = ph;
    }
}

void drawPlaylist() {
    const int y0 = L.plY;
    fill(0, y0 - 1, L.cw, L.plRows * UI_FONT_H + 1, C_PANEL);
    hline(L.m, y0 - 2, L.cw - 2 * L.m, C_SEP);
    if (!P.n) { text(L.m, y0, "No music files in this folder or below", C_TEXT_DIM); return; }
    int first = P.cur - 1;
    if (first > P.n - L.plRows) first = P.n - L.plRows;
    if (first < 0) first = 0;
    for (int r = 0; r < L.plRows && first + r < P.n; r++) {
        const int i = first + r, y = y0 + r * UI_FONT_H;
        const bool cur = i == P.cur;
        if (cur) fill(L.m, y, L.cw - 2 * L.m, UI_FONT_H, C_SEL_BG);
        char num[8]; snprintf(num, sizeof(num), "%3d ", i + 1);
        const int x = L.m + 2 * L.sc;
        text(x, y, num, cur ? C_WHITE : C_TEXT_DIM);
        const int nx = x + textWidth(num);
        textClip(nx, y, L.cw - L.m - nx - 2 * L.sc, dispName(i).c_str(), cur ? C_WHITE : C_TEXT);
    }
}

void drawAll() {
    fill(0, 0, Sf.w, Sf.h, C_PANEL);
    drawHeader();
    drawInfo();
    drawTime();
    meterLayout();
    drawMeter();
    drawPlaylist();
    drawLegend();
}

// ── transport ─────────────────────────────────────────────────────────────────
void playReset() {
    ringFlush();
    E.seekTo = -1;
    if (E.dec) { delete E.dec; E.dec = nullptr; }
    E.ended = false;
    E.paused = false;
    s_msg.clear();
    s_msgUntil = 0;
}

bool playOpen(const string& path, const char* disp);

bool playIndex(int i) {
    playReset();
    SC.pending.clear();                    // an explicit pick supersedes a by-path start
    if (i < 0 || i >= P.n) { P.cur = -1; return false; }
    P.cur = i;
    s_cur = P.name(i);
    string path = s_dir + P.name(i);
#if ZIFI_NET_CLIENT
    if (s_rfs) {
        // One fixed file per extension, overwritten track after track (the
        // previous decoder is gone by now, so nothing holds it open).
        infoRow(0, "File", dispName(i).c_str());
        infoRow(1, "Format", "downloading...");
        path = string("/tmp/_play.") + FileUtils::getLCaseExt(P.base(i));
        if (!s_rfs->get(P.name(i), path, [](uint32_t, uint32_t) { return true; })) {
            s_msg = "Download failed";
            Debug::log("Player: download %s failed", P.name(i));
            return false;
        }
    }
#endif
    return playOpen(path, dispName(i).c_str());
}

bool playOpen(const string& path, const char* disp) {
    pp::Decoder* d = pp::createDecoder(FileUtils::getLCaseExt(path));
    if (!d) { s_msg = "Unsupported format"; return false; }
    infoRow(0, "File", disp);
    infoRow(1, "Format", "opening...");   // a MIDI bank load from SD takes a moment
    if (!d->open(path.c_str())) {
        s_msg = d->err ? d->err : "Cannot play";
        if (d->meta.format[0]) { s_msg += ": "; s_msg += d->meta.format; }
        Debug::log("Player: %s: %s", path.c_str(), s_msg.c_str());
        delete d;
        return false;
    }
    E.dec = d;
    s_curPath = path;
    Debug::log("Player: %s [%s] free=%u", path.c_str(), d->meta.format, (unsigned)getFreeHeap());
    feed();
    return true;
}

// ── seeking ───────────────────────────────────────────────────────────────────
// The decoders only render forward, so a seek is a fast silent render up to the
// target; going back reopens the track (from the file it was opened from — for
// a catalog track that is its /tmp copy, nothing is downloaded again) and then
// renders forward. It runs in slices from the page loop, so a held arrow key
// just moves the target and the UI keeps up.
constexpr int SEEK_STEP_MS = 5000;
constexpr uint32_t SEEK_SLICE_US = 15000;

void seekBy(int delta) {
    if (!E.dec || E.ended) return;
    int64_t t = (int64_t)shownPosMs() + delta;
    if (t < 0) t = 0;
    const uint32_t len = E.dec->lenMs();
    if (len && t >= (int64_t)len) t = len > 500 ? len - 500 : 0;
    E.seekTo = (int32_t)t;
    ringFlush();
}

void seekStep() {
    if (E.seekTo < 0 || !E.dec) return;
    const uint32_t target = (uint32_t)E.seekTo;
    if (E.dec->posMs() > target + 50) {              // backwards: start the track again
        // The old decoder goes FIRST: several share one global engine (the second
        // Z80 copy, libxmp's page allocator, the MIDI synth, the MP3 arena), and
        // deleting it after the new one opened tore that down under the new one —
        // its first render() returned 0 and autoplay skipped to the next track.
        delete E.dec;
        E.dec = nullptr;
        pp::Decoder* d = pp::createDecoder(FileUtils::getLCaseExt(s_curPath));
        if (!d || !d->open(s_curPath.c_str())) {
            if (d) { s_msg = d->err ? d->err : "Cannot play"; delete d; }
            E.seekTo = -1;
            return;
        }
        E.dec = d;
    }
    const uint64_t t0 = time_us_64();
    while (E.dec->posMs() < target) {
        if (E.dec->render(E.tmp, FEED) <= 0) { E.ended = true; E.seekTo = -1; return; }
        if (time_us_64() - t0 >= SEEK_SLICE_US) return;
    }
    E.seekTo = -1;
    ringFlush();
}

// ── folders ───────────────────────────────────────────────────────────────────
// The playlist keeps each folder's tracks together (plBuild's sort), so a
// folder is a run of equal directory prefixes. PgDn = first track of the next
// folder, PgUp = first track of the previous one; both wrap.
size_t dirLen(int i) {
    const char* n = P.name(i);
    const char* sl = strrchr(n, '/');
    return sl ? (size_t)(sl - n) + 1 : 0;
}
bool sameDir(int a, int b) {
    const size_t la = dirLen(a);
    return la == dirLen(b) && !strncmp(P.name(a), P.name(b), la);
}
int dirStart(int i) {
    while (i > 0 && sameDir(i - 1, i)) i--;
    return i;
}
int folderNext() {
    if (P.n <= 0) return -1;
    const int c = P.cur >= 0 ? P.cur : 0;
    for (int j = c + 1; j < P.n; j++) if (!sameDir(j, c)) return j;
    return 0;
}
int folderPrev() {
    if (P.n <= 0) return -1;
    const int s0 = dirStart(P.cur >= 0 ? P.cur : 0);
    return dirStart(s0 > 0 ? s0 - 1 : P.n - 1);
}

void pushHist(int i) {
    if (i < 0) return;
    if (P.nh == HIST) { memmove(P.hist, P.hist + 1, (HIST - 1) * sizeof(int)); P.nh--; }
    P.hist[P.nh++] = i;
}

int pickNext(bool wrap) {
    if (!P.n) return -1;
    if (s_shuffle && P.n > 1) {
        int j;
        do { j = (int)(get_rand_32() % (uint32_t)P.n); } while (j == P.cur);
        return j;
    }
    const int j = P.cur + 1;
    if (j < P.n) return j;
    return wrap ? 0 : -1;
}

// First track of a freshly opened folder: the top one, or a random one in shuffle.
int pickFirst() {
    if (!P.n) return -1;
    if (s_shuffle && P.n > 1) return (int)(get_rand_32() % (uint32_t)P.n);
    return 0;
}

void next(bool manual) {
    const int j = pickNext(manual || s_shuffle);
    if (j < 0) { ringFlush(); if (E.dec) { delete E.dec; E.dec = nullptr; } return; }
    pushHist(P.cur);
    playIndex(j);
}

void prev() {
    if (E.dec && E.dec->posMs() > 3000) { playIndex(P.cur); return; }
    // The history is for shuffle only, where "previous" means "the one played
    // before". In list order it is the track above — also right after End/Home or
    // a folder jump, which used to lead back to wherever the jump started from.
    int j;
    if (s_shuffle && P.nh) j = P.hist[--P.nh];
    else j = P.cur > 0 ? P.cur - 1 : P.n - 1;
    playIndex(j);
}

// Empties the key queue. Needed after anything that returned on a keypress:
// the input layer queues every Enter/arrow TWICE (a VK_MENU_* twin + the raw
// key), the browser leaves on the first, and the second would reach our loop
// as a fresh Enter — i.e. "pause" right after the track started.
void drainKeys() {
    auto kbd = ESPectrum::PS2Controller.keyboard();
    fabgl::VirtualKeyItem d;
    while (kbd->virtualKeyAvailable()) kbd->getNextVirtualKey(&d);
}

// Opens the file browser on the playlist folder; plays the pick.
bool browse() {
    string dir = s_dir.empty() ? FileUtils::ALL_Path : s_dir;
    string r;
    for (;;) {
#if ZIFI_NET_CLIENT
        // ".." at a volume root leads to the F5 location chooser (SD / USB /
        // Remote / Web Archives), as in the emulator's own browser.
        const bool chooser = !s_rfs && osdPlayerHasLocations();
        const bool prevRoot = OSD::fd_root_parent;
        OSD::fd_root_parent = chooser;
#endif
        r = browseFile(dir, TXT_PLAYER, DISK_MUSFILE);
#if ZIFI_NET_CLIENT
        OSD::fd_root_parent = prevRoot;
        if (r == "\x02UP") {
            // A network flow plays in a player of its own: this one steps aside
            // first, so there are never two engines on the output.
            engineStop();
            plFree();
            const bool local = osdPlayerLocations();
            gfxInstallPalette();
            drainKeys();
            if (!local || !engineStart()) { s_quitReq = true; return false; }
            dir = FileUtils::ALL_Path;
            continue;
        }
#endif
        break;
    }
    gfxInstallPalette();
    drainKeys();                   // the Enter that picked the file has a twin
    if (r.size() < 2 || (r[0] != 'R' && r[0] != 'M')) return false;
    const string name = r.substr(1);
    if (name.back() == '/') {                  // F2 on a folder: play all of it
        s_dir = dir + name;
        plBuild(s_dir);
        plWaitFirst();
        pushHist(P.cur);
        playIndex(pickFirst());
        return true;
    }
    if (!pp::playableExt(FileUtils::getLCaseExt(name))) return false;
    if (dir != s_dir || !P.n) {
        s_dir = dir;
        plBuild(s_dir);
    }
    pushHist(P.cur);
    playName(name);
    return true;
}

enum PlAct : uint8_t { PA_NONE, PA_UP, PA_DOWN, PA_LEFT, PA_RIGHT, PA_PAUSE, PA_BACK,
                       PA_MUTE, PA_SHUF, PA_AUTO, PA_REPEAT, PA_SEEKB, PA_SEEKF, PA_DIRP, PA_DIRN, PA_FILES, PA_HOME, PA_END, PA_LEGEND };

PlAct plAct(fabgl::VirtualKey vk) {
    switch (vk) {
        // F9 / F10 = volume, the emulator's own hot keys (+ / - too)
        case fabgl::VK_F10: case fabgl::VK_PLUS: case fabgl::VK_KP_PLUS:                             return PA_UP;
        case fabgl::VK_F9:  case fabgl::VK_MINUS: case fabgl::VK_KP_MINUS:                           return PA_DOWN;
        case fabgl::VK_UP:    case fabgl::VK_MENU_UP:                                                 return PA_LEFT;    // previous track
        case fabgl::VK_DOWN:  case fabgl::VK_MENU_DOWN:                                               return PA_RIGHT;   // next track
        case fabgl::VK_PAGEUP:                                                                        return PA_DIRP;
        case fabgl::VK_PAGEDOWN:                                                                      return PA_DIRN;
        case fabgl::VK_LEFT:  case fabgl::VK_MENU_LEFT:                                               return PA_SEEKB;
        case fabgl::VK_RIGHT: case fabgl::VK_MENU_RIGHT:                                              return PA_SEEKF;
        case fabgl::VK_SPACE: case fabgl::VK_RETURN: case fabgl::VK_MENU_ENTER:
        case fabgl::VK_p: case fabgl::VK_P:                                                           return PA_PAUSE;
        case fabgl::VK_ESCAPE: case fabgl::VK_MENU_BS:                                               return PA_BACK;
        case fabgl::VK_F1:                                                                            return PA_LEGEND;
        case fabgl::VK_m: case fabgl::VK_M:                                                           return PA_MUTE;
        case fabgl::VK_s: case fabgl::VK_S:                                                           return PA_SHUF;
        case fabgl::VK_a: case fabgl::VK_A:                                                           return PA_AUTO;
        case fabgl::VK_r: case fabgl::VK_R:                                                           return PA_REPEAT;
        case fabgl::VK_F5: case fabgl::VK_TAB: case fabgl::VK_f: case fabgl::VK_F:                   return PA_FILES;
        case fabgl::VK_HOME:                                                                          return PA_HOME;
        case fabgl::VK_END:                                                                           return PA_END;
        default:                                                                                      return PA_NONE;
    }
}


// The page body. `startPath` (full path) starts that file; empty = resume the
// session's folder, or open the browser when there is none.
void run(const string& startPath) {
    s_quitReq = false;
    layout();
    if (!engineStart()) {
        fill(0, 0, Sf.w, Sf.h, C_PANEL);
        text(L.m, Sf.h / 2, "Pico-Zx-Player: not enough memory", C_TEXT);
        uiIdle(1500);
        return;
    }
    drainKeys();

    bool ok = true;
#if ZIFI_NET_CLIENT
    if (s_rfs) {                           // remote folder; startPath = display name or ""
        plBuildRemote(s_rfs);
        drawAll();
        const int i = startPath.empty() ? -1 : plFind(startPath);
        playIndex(i >= 0 ? i : pickFirst());
    } else
#endif
    if (!startPath.empty() && startPath.back() == '/') {   // a whole folder
        s_dir = startPath;
        plBuild(s_dir);
        plWaitFirst();
        drawAll();
        playIndex(pickFirst());
    } else if (!startPath.empty()) {
        const size_t sl = startPath.find_last_of('/');
        s_dir = startPath.substr(0, sl + 1);
        const string name = startPath.substr(sl + 1);
        plBuild(s_dir);
        drawAll();
        playName(name);
    } else {
        if (s_dir.empty()) s_dir = FileUtils::ALL_Path;
        plBuild(s_dir);
        drawAll();
        if (!playName(s_cur)) {                        // no session track: the first one
            plWaitFirst();
            if (P.n) playIndex(0);
            else ok = browse();
        }
    }
    if (!ok && !E.dec) { engineStop(); plFree(); return; }
    drawAll();

    auto kbd = ESPectrum::PS2Controller.keyboard();
    uint64_t nextDraw = time_us_64();
    uint32_t failRun = 0;
    int shownCur = P.cur;
    int shownN = P.n;
    bool shownScan = SC.active;
    uint32_t shownBlink = 0;
    pp::Decoder* shownDec = E.dec;
    bool lastPaused = E.paused, lastMuted = E.muted;
    uint32_t lastSec = 0xFFFFFFFF, lastMetaSeq = E.dec ? E.dec->metaSeq : 0;

    while (true) {
        seekStep();
        feed();

        // keys (verb-collapsed: arrows/Enter/Space arrive with a VK_MENU_* twin)
        fabgl::VirtualKeyItem k;
        PlAct prevAct = PA_NONE;
        bool quit = false, hdr = false;
        while (kbd->virtualKeyAvailable()) {
            if (!ESPectrum::readKbd(&k) || !k.down) continue;
            const PlAct a = plAct(k.vk);
            if (a == PA_NONE || a == prevAct) continue;
            prevAct = a;
            switch (a) {
                case PA_BACK:  quit = true; break;
                case PA_LEGEND:
                    s_legend = !s_legend;
                    layout(); drawAll(); shownDec = E.dec; shownCur = P.cur; lastSec = 0xFFFFFFFF;
                    break;
                case PA_UP:    if (s_vol < 20) s_vol++; E.gainQ8 = volGain(s_vol); hdr = true; break;
                case PA_DOWN:  if (s_vol > 0)  s_vol--; E.gainQ8 = volGain(s_vol); hdr = true; break;
                case PA_MUTE:  E.muted = !E.muted; hdr = true; break;
                case PA_SHUF:  s_shuffle = !s_shuffle; hdr = true; break;
                case PA_AUTO:  s_auto = !s_auto; hdr = true; break;
                case PA_REPEAT: s_repeat = !s_repeat; hdr = true; break;
                case PA_DIRP: case PA_DIRN: {
                    const int j = a == PA_DIRP ? folderPrev() : folderNext();
                    if (j >= 0) { failRun = 0; pushHist(P.cur); playIndex(j); }
                    break;
                }
                case PA_SEEKB: seekBy(-SEEK_STEP_MS); lastSec = 0xFFFFFFFF; break;
                case PA_SEEKF: seekBy(SEEK_STEP_MS); lastSec = 0xFFFFFFFF; break;
                case PA_PAUSE:
                    if (!E.dec) { failRun = 0; playIndex(P.cur >= 0 ? P.cur : 0); }
                    else { E.paused = !E.paused; if (E.paused) ringFlush(); }
                    hdr = true; break;
                case PA_RIGHT: failRun = 0; next(true); break;
                case PA_LEFT:  failRun = 0; prev(); break;
                case PA_HOME:  failRun = 0; pushHist(P.cur); playIndex(0); break;
                case PA_END:   failRun = 0; pushHist(P.cur); playIndex(P.n - 1); break;
                case PA_FILES:
#if ZIFI_NET_CLIENT
                    if (s_rfs) { quit = true; break; }   // back to the catalog
#endif
                    browse();
                    if (s_quitReq) { quit = true; break; } drawAll(); shownDec = E.dec; shownCur = P.cur; break;
                default: break;
            }
            if (quit) break;
        }
        if (quit) break;

        // end of track / failed track → autoplay
        if (E.dec && E.ended && ringDrained()) {
            if (s_repeat) playIndex(P.cur);          // same track again
            else if (s_auto) next(false);
            else { delete E.dec; E.dec = nullptr; }
        } else if (!E.dec && !s_msg.empty() && s_auto && P.n > 1 && failRun < (uint32_t)P.n) {
            // a file that would not open: show it briefly, then move on
            if (!s_msgUntil) s_msgUntil = to_ms_since_boot(get_absolute_time()) + 1500;
            else if ((int32_t)(to_ms_since_boot(get_absolute_time()) - s_msgUntil) >= 0) {
                s_msgUntil = 0; failRun++;
                next(false);
            }
        }
        if (E.dec) failRun = 0;

        if (E.dec != shownDec || P.cur != shownCur) {
            shownDec = E.dec; shownCur = P.cur;
            drawHeader(); drawInfo(); drawTime(); meterLayout(); drawPlaylist();
            lastSec = 0xFFFFFFFF;
            lastMetaSeq = E.dec ? E.dec->metaSeq : 0;
        } else if (E.dec && E.dec->metaSeq != lastMetaSeq) {
            lastMetaSeq = E.dec->metaSeq;
            drawInfo(); drawTime(); lastSec = 0xFFFFFFFF;
        }
        if (hdr || E.paused != lastPaused || E.muted != lastMuted) {
            lastPaused = E.paused; lastMuted = E.muted;
            drawHeader();
            drawLegend();
        }

        // The playlist grows in the background (see PlScan).
        if (SC.active) plScanStep(4000);

        const uint64_t now = time_us_64();
        if ((int64_t)(now - nextDraw) >= 0) {
            nextDraw = now + 40000;                       // 25 fps
            if (P.n != shownN || SC.active != shownScan) {
                shownN = P.n; shownScan = SC.active; drawHeader(); drawPlaylist();
            } else if (SC.active && (uint32_t)(now / 400000) != shownBlink) {
                drawHeader();                             // the LOADING blink
            }
            shownBlink = (uint32_t)(now / 400000);
            drawMeter();
            const uint32_t sec = shownPosMs() / 1000;
            if (sec != lastSec) { lastSec = sec; drawTime(); }
        }
        uiIdle(2);
    }

    engineStop();
    plFree();
    drainKeys();
}

} // namespace

// Menu > Pico-Zx-Player.
void act_player() {
    gfxResumePalette();             // runModal suspended the UI palette
    run(string());
#if ZIFI_NET_CLIENT
    // A launch from Web Archives (reached through the player's F5) closes the menu.
    if (OSD::net_launch_close) requestClose();
    OSD::net_launch_close = false;
    OSD::net_close_all = false;
#endif
}

// F5 browser: play `path` (a full path). Owns its own gfx session, like the
// standalone game entrance.
void playerStandalone(const std::string& path) {
    gfxBegin();
    run(path);
    gfxEnd();
    VIDEO::brdnextframe = true;
}

#if ZIFI_NET_CLIENT
// Web catalog / FTP: play the remote cwd's playable files, starting at the entry
// whose display name is `startDisp` ("" = first, or random in shuffle). The
// local session (folder + track) is left as it was.
void playerRemote(RemoteFs* fs, const std::string& startDisp) {
    const string sd = s_dir, sc = s_cur;
    s_rfs = fs;
    gfxBegin();
    run(startDisp);
    gfxEnd();
    s_rfs = nullptr;
    s_dir = sd; s_cur = sc;
    VIDEO::brdnextframe = true;
}
#endif

} // namespace nm
