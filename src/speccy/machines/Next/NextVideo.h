/*

pico-speccy: ZX Spectrum Next scanline video renderer

In Next mode VIDEO::Draw is swapped to NEXTVID::Tick — a cheap trampoline
that renders whole scanlines when the CPU crosses a line boundary, instead
of the per-T-state ULA beam-chasing used by the 48K/128K machines. Next
software does per-line effects through the Copper and the line interrupt,
so scanline granularity matches how the commercial catalogue is written
(same approach as CSpect).

Palettes: the hardware CLUT keeps the stock layout (indices 17-219 are a
G3R3B2 cube, 0-16 are the Spectrum/OSD colours, 240+ are reserved by the
scanout). Next RGB333 palette entries are translated to the nearest CLUT
index at palette-write time into per-palette LUTs, so mid-frame palette
tricks never touch the hardware CLUT.

*/

#ifndef NextVideo_h
#define NextVideo_h



#include <inttypes.h>

class NEXTVID {
public:
    static void Reset();       // machine reset: geometry, default palettes
    static void EndFrame();    // frame restart (called from VIDEO::EndFrame)

    // VIDEO::Draw replacements
    static void Tick(unsigned int statestoadd, bool contended);
    static void Tick_Opcode(bool contended);
    static void DrawBorderNop();

    // nextreg palette interface (regs 0x40/0x41/0x43/0x44)
    static void palIndex(uint8_t v);
    static void palValue8(uint8_t v);
    static void palValue9(uint8_t v);
    static void palControl(uint8_t v);

    // clip windows (regs 0x18-0x1B, index reset 0x1C)
    // window: 0=Layer2, 1=sprites, 2=ULA, 3=tilemap; values x1,x2,y1,y2
    static void clipWrite(uint8_t win, uint8_t v);
    static void clipIndexReset(uint8_t mask);

    static uint8_t clip[4][4];
    static uint8_t clipIdx[4];

    // sprite interface (ports 0x303B / 0x57 / 0x5B, nextreg 0x35-0x39)
    static void spriteSlotSelect(uint8_t v);   // port 0x303B write
    static uint8_t spriteFlagsRead();          // port 0x303B read
    static void spriteAttrWrite(uint8_t v);    // port 0x57 write
    static void spritePatternWrite(uint8_t v); // port 0x5B write
    static void spriteAttrDirect(uint8_t byteIdx, uint8_t v); // nextreg 0x35-0x39

    // Copper (nextreg 0x60-0x63)
    static void copperDataWrite(uint8_t v);    // reg 0x60: byte at index++
    static void copperIndexLo(uint8_t v);      // reg 0x61
    static void copperControl(uint8_t v);      // reg 0x62: index MSB + mode
    static void copperData16Write(uint8_t v);  // reg 0x63: 16-bit two-write

private:
    static void ScanlineWork();
    static void RenderLine(int row);
    static void RenderSpritesLine(uint8_t* fb, int y);
    static void rebuildLut(uint8_t pal);
    static void setLutEntry(uint8_t pal, uint8_t idx, uint16_t rgb333);
    static void buildRemap();
    static void defaultPalettes();

    // sprite state
    // Sprite/copper/palette storage lives in ONE heap block allocated at the
    // first NEXTVID::Reset() (~25KB) — Next-only cost, not BSS every machine
    // pays. Pointer-to-array types keep the 2D indexing syntax intact.
    static uint8_t* sprPatterns;         // [16384] 64 patterns x 256B
    static uint8_t  (*sprAttr)[5];       // [128][5]
    static uint16_t sprPatWrite;         // pattern upload byte position
    static uint8_t  sprAttrSlot;         // attribute upload sprite index
    static uint8_t  sprAttrByte;         // attribute upload byte index
    static uint8_t  sprFlags;            // bit0 collision, bit1 overflow

    // copper state
    static void copperLine(uint16_t rasterLine);
    static void RenderTilemapLine(uint8_t* fb, int row);
    static void RenderL2Line(uint8_t* fb, int row, bool prioPass);
    static bool palHasPrio[8];           // palette contains L2 priority entries
    static uint8_t* copperMem;           // [2048] 1024 big-endian instructions
    static uint16_t copIndex;            // write index (bytes)
    static uint16_t copPC;               // instruction counter
    static uint8_t  copCtrl;             // reg 0x62 latch (bits 7:6 = mode)
    static bool     copWaiting;
    static uint16_t copWaitLine;
    static bool     cop16Second;
    static uint8_t  cop16First;

    // 4 palettes x 2 banks: 0=ULA1 1=L2-1 2=Spr1 3=TM1 4=ULA2 5=L2-2 6=Spr2 7=TM2
    static uint16_t (*rawPal)[256];  // [8][256] 9-bit RGB333
    static uint8_t  (*lut)[256];     // [8][256] palette entry -> CLUT index
    static uint8_t  remap[256];      // G3R3B2 index -> allowed CLUT index

    static uint8_t palIdx;           // reg 0x40 write index
    static bool    pal9Second;       // expecting 2nd byte of reg 0x44
    static uint8_t pal9First;
    static uint8_t palCtrl;          // reg 0x43 latch

    static uint32_t lineIdx;
    static uint32_t totalLines;
    static uint32_t nextLineTstate;
    static int firstVisibleLine;
    static int paperTopRow;
    static int xPaperOff;
    static bool skipFrame;
};



#endif // NextVideo_h
