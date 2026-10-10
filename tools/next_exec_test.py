#!/usr/bin/env python3
"""Exercise production Next executor control flow against the former inner loop.

Synthetic handlers test prefix dispatch, Q/EI state, MMU changes at raster
boundaries, slice termination and fast-memory revocation, not Z80 op semantics.
Run: python3 tools/next_exec_test.py (requires g++).
"""
from pathlib import Path
import subprocess
import tempfile

src = (Path(__file__).resolve().parents[1] / 'src/speccy/z80/Z80_JLS.cpp').read_text()
a = src.index('IRAM_ATTR __attribute__((noinline, optimize("O2"))) void Z80::exec_nocheck_next')
b = src.index('\nIRAM_ATTR void Z80::exec_nocheck()', a)
production = src[a:b].replace('IRAM_ATTR ', '')

stub = r'''
#include <cassert>
#include <cstdint>
#include <cstdio>
#include <vector>
#include <tuple>
#define PERF_TRACE 0
#define PERF_HIST 0
struct State {
    uint16_t pc; uint32_t r,t,end,line;
    uint8_t prefix,op; bool q,last,ei,fast;
    unsigned page,handlers,limit;
    bool operator==(const State& v) const {
        return std::tie(pc,r,t,end,line,prefix,op,q,last,ei,fast,page,handlers,limit)==
               std::tie(v.pc,v.r,v.t,v.end,v.line,v.prefix,v.op,v.q,v.last,v.ei,v.fast,v.page,v.handlers,v.limit);
    }
} s;
#define REG_PC s.pc
#define regR s.r
#define prefixOpcode s.prefix
#define opCode s.op
#define flagQ s.q
#define lastFlagQ s.last
#define pendingEI s.ei
bool& g_next_fastrd=s.fast;
namespace CPU { uint32_t &tstates=s.t,&stFrame=s.end; }
uint8_t pages[2][65536];
namespace MemESP { uint8_t* ramCurrent[8]; }
void remap(){for(unsigned i=0;i<8;i++)MemESP::ramCurrent[i]=pages[s.page]+i*8192;}
std::vector<uint32_t> events;
void tsFastTick(unsigned t){
    s.t+=t;
    if(s.t>=s.line){s.page^=1;s.line+=28;remap();events.push_back(0x10000000|s.t);}
}
namespace DivMMC {
    void preOpcFetch(uint16_t pc){events.push_back(0x20000000|pc);}
    void postOpcFetch(){events.push_back(0x30000000|s.pc);}
    uint8_t nextOpcode(uint16_t pc){preOpcFetch(pc);auto op=MemESP::ramCurrent[pc>>13][pc&8191];postOpcFetch();return op;}
}
void handler(){
    events.push_back(0x40000000|(s.prefix<<16)|(s.op<<8)|s.last);
    s.t+=3;s.q=(s.op&1)!=0;s.ei=s.op==0xfb;
    if(s.op==0xdd||s.op==0xed||s.op==0xfd)s.prefix=s.op;
    if(s.op==0x76)s.end=0; // HALT/EI-style slice termination
    if(++s.handlers>=s.limit)s.fast=false;
}
void (*dcOpcode[256])();
int regIX=0,regIY=1;
struct Z80 {
    static void exec_nocheck_next(bool);
    static void decodeDDFD(int which){events.push_back(0x50000000|which);handler();}
    static void decodeED(){events.push_back(0x60000000);handler();}
};
'''
# Former Next inner loop: only the debugger branch is omitted (the new
# entry point is called exclusively when numPcBP <= 0).
reference = r'''
void baseline(bool divmmc){
    do {
        const uint16_t pc=REG_PC;
        tsFastTick(4);
        if(divmmc)DivMMC::preOpcFetch(pc);
        opCode=MemESP::ramCurrent[pc>>13][pc&0x1fff];
        if(divmmc)DivMMC::postOpcFetch();
        regR++;REG_PC=pc+1;
        if(prefixOpcode==0){
            flagQ=pendingEI=false;
            dcOpcode[opCode]();lastFlagQ=flagQ;continue;
        }
        if(prefixOpcode==0xdd){prefixOpcode=0;Z80::decodeDDFD(regIX);}
        else if(prefixOpcode==0xed){prefixOpcode=0;Z80::decodeED();}
        else if(prefixOpcode==0xfd){prefixOpcode=0;Z80::decodeDDFD(regIY);}
        else continue;
        if(prefixOpcode==0)lastFlagQ=flagQ;
    }while(CPU::tstates<CPU::stFrame&&g_next_fastrd);
}
int main(){
    for(auto& f:dcOpcode)f=handler;
    const uint8_t ops[]={0,1,0xdd,0xed,0xfd,0xfb,3,0x76};
    for(unsigned p=0;p<2;p++)for(unsigned i=0;i<65536;i++)pages[p][i]=ops[(i*7+(i>>5)+p)%8];
    unsigned count=0;
    for(unsigned pc=0;pc<65536;pc+=7)for(unsigned pref:{0,0xdd,0xed,0xfd,0xcb})for(bool divmmc:{false,true}){
        s={uint16_t(pc),0xfe,0,1+pc%127,1+pc%28,uint8_t(pref),0,true,true,true,true,pc%2,0,1+pc%13};
        const State initial=s;
        remap();events.clear();baseline(divmmc);const State expected=s;const auto calls=events;
        s=initial;remap();events.clear();Z80::exec_nocheck_next(divmmc);
        assert(s==expected);assert(events==calls);++count;
    }
    printf("PASS: %u executor control-flow comparisons\n",count);
}
'''
with tempfile.TemporaryDirectory(prefix='next-exec-test-') as work:
    cpp = Path(work) / 'test.cpp'
    binary = Path(work) / 'test'
    cpp.write_text(stub + production + reference)
    subprocess.run(['g++', '-std=c++17', '-O2', '-Wall', '-Wextra',
                    '-fsanitize=undefined', str(cpp), '-o', str(binary)], check=True)
    subprocess.run([str(binary)], check=True)
