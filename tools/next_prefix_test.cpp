// g++ -std=c++17 -O2 -Wall -Wextra -fsanitize=undefined -Isrc tools/next_prefix_test.cpp -o /tmp/next_prefix_test && /tmp/next_prefix_test
// Repeat with -DDEBUG=1 to check instrumentation fallback.
#include <cstdint>
#include <cassert>
#include <cstdio>
#include <initializer_list>
#ifndef DEBUG
#define DEBUG 0
#endif
struct CPU { static uint32_t tstates; };
struct VIDEO { static uint32_t ts_line_t; };
struct Z80 { static uint16_t pc; static uint16_t getRegPC(){return pc;} };
struct MemESP { static uint8_t* ramCurrent[8]; };
struct DivMMC { static bool enabled; static uint8_t nextOpcode(uint16_t); };
struct Z80Ops { static uint8_t fetchOpcode(); };
uint32_t CPU::tstates,VIDEO::ts_line_t;
uint16_t Z80::pc;
uint8_t* MemESP::ramCurrent[8];
bool DivMMC::enabled,g_next_fastrd;
static uint8_t pages[10][8192];
static unsigned fallback,trapcalls,ticks;
static uint32_t fallback_start;
#include "speccy/z80/next_opcode.h"

uint8_t DivMMC::nextOpcode(uint16_t pc){
    ++trapcalls;
    // Mapping semantics are exhaustively tested in next_opcode_test.py.
    // These two sentinels ensure the prefix wrapper preserves read ordering.
    if(pc==0x3d00)MemESP::ramCurrent[1]=pages[9];
    const uint8_t op=MemESP::ramCurrent[pc>>13][pc&8191];
    if(pc==0x0008)MemESP::ramCurrent[0]=pages[9];
    return op;
}
uint8_t Z80Ops::fetchOpcode(){
    ++fallback;fallback_start=CPU::tstates;CPU::tstates+=4;
    if(CPU::tstates>=VIDEO::ts_line_t){
        ++ticks;MemESP::ramCurrent[Z80::pc>>13]=pages[8];
    }
    if(DivMMC::enabled)return DivMMC::nextOpcode(Z80::pc);
    return MemESP::ramCurrent[Z80::pc>>13][Z80::pc&8191];
}
void reset(unsigned pc,unsigned t){
    for(unsigned slot=0;slot<8;slot++)MemESP::ramCurrent[slot]=pages[slot];
    Z80::pc=pc;CPU::tstates=t;VIDEO::ts_line_t=228;fallback=trapcalls=ticks=0;
}
int main(){
    for(unsigned page=0;page<10;page++)for(unsigned i=0;i<8192;i++)pages[page][i]=(i*7+page*43)&255;
    unsigned cases=0;
    for(unsigned pc=0;pc<65536;pc++)for(bool enabled:{false,true})for(bool div:{false,true})for(unsigned t:{100,223,224,227,230}){
        g_next_fastrd=enabled;DivMMC::enabled=div;
        reset(pc,t);const uint8_t expected=Z80Ops::fetchOpcode();
        uint8_t* slots[8];for(unsigned i=0;i<8;i++)slots[i]=MemESP::ramCurrent[i];
        const unsigned expected_traps=trapcalls,expected_ticks=ticks;
        reset(pc,t);const uint8_t got=NextOpcode::fetch();
        assert(got==expected&&CPU::tstates==t+4&&Z80::pc==pc);
        assert(trapcalls==expected_traps&&ticks==expected_ticks);
        assert(fallback==unsigned(DEBUG||!enabled||t+4>=228));
        if(fallback)assert(fallback_start==t);
        for(unsigned i=0;i<8;i++)assert(slots[i]==MemESP::ramCurrent[i]);
        ++cases;
    }
    printf("PASS: %u prefix fetch cases (DEBUG=%d)\n",cases,DEBUG);
}
