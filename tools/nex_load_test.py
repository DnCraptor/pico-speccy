#!/usr/bin/env python3
"""Run the production NEX loader with in-memory files and hardware stubs.

Checks entry mapping/defaults, preserved registers, bank order, and short reads.
Run: python3 tools/nex_load_test.py (requires g++).
"""
from pathlib import Path
import subprocess
import tempfile

root = Path(__file__).resolve().parents[1]
production = (root / 'src/speccy/machines/Next/FileNEX.cpp').read_text()
production = production[production.index('using namespace std;'):]
stub = r'''
#include <algorithm>
#include <cassert>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>
#include <vector>
using UINT = unsigned;
using FSIZE_t = size_t;
constexpr int FR_OK=0, FA_READ=1, LEVEL_WARN=1, LEVEL_INFO=2, A_NEXT=1, R_NEXT=1;
struct FIL { std::vector<uint8_t> data; size_t pos=0; bool closed=false; } file;
int f_read(FIL* f, void* dst, UINT n, UINT* got) {
    *got=std::min<size_t>(n, f->data.size()-f->pos);
    memcpy(dst,f->data.data()+f->pos,*got); f->pos+=*got; return FR_OK;
}
size_t f_size(FIL* f){ return f->data.size(); }
size_t f_tell(FIL* f){ return f->pos; }
int f_lseek(FIL* f, size_t p){ f->pos=std::min(p,f->data.size()); return FR_OK; }
FIL* fopen2(const char*,int){ file.pos=0; file.closed=false; return &file; }
void fclose2(FIL* f){ f->closed=true; }
unsigned butter_psram_size(){ return 2u<<20; }
struct ScopedHeap { void* p; ScopedHeap(int n):p(malloc(n)){} ~ScopedHeap(){free(p);} };
namespace Config { int arch=A_NEXT,romSet=R_NEXT; std::string ram_file; void save(){} }
namespace Z80Ops { bool isNext=true; }
namespace OSD { int errors=0; void osdCenteredMsg(const std::string&,int,int){ ++errors; } void esp_hard_reset(){abort();} }
namespace Debug { template<class... T> void log(const char*,T...){} }
namespace NexLaunch { int start(const std::string&){return 0;} }
namespace CPU { unsigned tstates=0,global_tstates=0; }
namespace MemESP {
    constexpr int NEXT_PAGES=224;
    uint8_t memory[112][16384], *nextRamPtr[224];
    struct Page { uint8_t* p; uint8_t* direct(){return p;} } ram[112];
    bool nextRamReady=true,nextBootRomActive=true,nextConfigMode=true;
    void applyNextConfigWindow(){}
}
namespace NextReg {
    uint8_t reg[256],p7=0,p1=0;
    void write(uint8_t r,uint8_t v){reg[r]=v;}
    void write7FFD(uint8_t v){p7=v;}
    void write1FFD(uint8_t v){p1=v;}
    void writeLayer2Port(uint8_t v){reg[0x69]=v<<6;}
    void contendRecalc(){}
}
namespace DivMMC { bool automap=true,conmem=true; void applyMapping(){} }
namespace ESPectrum {
    int resets=0;
    void reset(){++resets; memset(NextReg::reg,0,256); NextReg::p7=NextReg::p1=0;}
}
namespace VIDEO { uint8_t borderColor; uint32_t brd,border32[8]; }
struct Z80 {
    static inline uint16_t pc=0,sp=0,bc=0;
    enum {IM1=1};
    static void reset(){pc=0;}
    static void setRegPC(uint16_t x){pc=x;}
    static void setRegSP(uint16_t x){sp=x;}
    static void setRegBC(uint16_t x){bc=x;}
    static void setIM(int){} static void setIFF1(bool){} static void setIFF2(bool){}
};
struct FileNEX { static bool load(const std::string&); };
'''
tests = r'''
void fixture(bool preserve=false){
    file.data.assign(512,0); memcpy(file.data.data(),"NextV1.2",8);
    file.data[12]=0x00; file.data[13]=0xbe;
    file.data[14]=0x23; file.data[15]=0xc1; file.data[139]=8;
    file.data[134]=preserve;
    for(int bank: {5,2,0,1,3,4,6,7,8,111}){
        file.data[18+bank]=1; ++file.data[9];
        file.data.insert(file.data.end(),16384,uint8_t(bank));
    }
    for(int b=0;b<112;++b){
        MemESP::ram[b].p=MemESP::memory[b];
        MemESP::nextRamPtr[2*b]=MemESP::memory[b];
        MemESP::nextRamPtr[2*b+1]=MemESP::memory[b]+8192;
    }
    memset(NextReg::reg,0xa5,256);
    MemESP::nextBootRomActive=MemESP::nextConfigMode=true;
    DivMMC::automap=DivMMC::conmem=true;
    Z80::pc=0xdead; OSD::errors=0;
}
void mapping(){
    const uint8_t want[]={10,11,4,5,16,17};
    for(int i=0;i<6;++i)assert(NextReg::reg[0x52+i]==want[i]);
    assert(!MemESP::nextBootRomActive && !MemESP::nextConfigMode);
    assert(!DivMMC::automap && !DivMMC::conmem);
    assert(Z80::pc==0xc123 && Z80::sp==0xbe00 && Z80::bc==0);
    for(int b: {5,2,0,1,3,4,6,7,8,111})
        for(auto v: MemESP::memory[b])assert(v==b);
    assert(file.closed && OSD::errors==0);
}
int main(){
    fixture(); assert(FileNEX::load("test.nex")); mapping();
    assert(NextReg::p7==0x10 && NextReg::p1==4); // 48K ROM, not boot ROM
    assert(NextReg::reg[7]==3 && (NextReg::reg[8]&0xce)==0xce);
    assert(NextReg::reg[0x15]==1 && NextReg::reg[0x8c]==0);
    fixture(true); int resets=ESPectrum::resets;
    assert(FileNEX::load("preserve.nex")); mapping();
    assert(ESPectrum::resets==resets);
    assert(NextReg::reg[7]==0xa5 && NextReg::reg[0x15]==0xa5);
    // Partial palette, each screen type, and partial RAM bank must never jump.
    for(int screen: {0,1,2,4,8,16}){
        fixture(); file.data[10]=screen; file.data.resize(513);
        assert(!FileNEX::load("short.nex"));
        assert(Z80::pc==0xdead && file.closed && OSD::errors==1);
    }
    // Palette complete, Layer 2 screen truncated.
    fixture(); file.data[10]=1; file.data.resize(1025);
    assert(!FileNEX::load("short-layer2.nex")); assert(Z80::pc==0xdead);
    puts("NEX loader: entry state, preserve, bank order and short reads passed");
}
'''
with tempfile.TemporaryDirectory(prefix='nex-load-') as tmp:
    src = Path(tmp) / 'test.cpp'
    src.write_text(stub + production + tests)
    exe = Path(tmp) / 'test'
    subprocess.run(['g++', '-std=c++17', '-Wall', '-Wextra', '-Werror', str(src), '-o', str(exe)], check=True)
    subprocess.run([str(exe)], check=True)
