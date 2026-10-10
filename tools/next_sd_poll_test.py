#!/usr/bin/env python3
"""Differential test of the production SD poll batch against instruction stepping.

Run: python3 tools/next_sd_poll_test.py (requires g++).
Hardware I/O is stubbed; every read and its guest timestamp are compared.
"""
from pathlib import Path
import subprocess
import tempfile
src = (Path(__file__).resolve().parents[1] / 'src/speccy/z80/Z80_JLS.cpp').read_text()
a = src.index('NX_OVL_CODE void Z80::nextSdPollBatch()')
b = src.index('\nNX_OVL_CODE void Z80::nextInirBatch()', a)
body = src[a:b].replace('NX_OVL_CODE ', '')
h = r'''
#include <cstdint>
#include <cassert>
#include <cstdio>
#include <vector>
struct State {
 uint8_t a=255,b=0,r=250,f=0x93,op=0xdb;
 uint16_t pc=0x192a,wz=0xffec;
 bool carry=true,q=false;
 uint32_t t=100;
 unsigned reads=0;
 bool operator==(const State& o) const {
 return a==o.a && b==o.b && r==o.r && f==o.f && op==o.op && pc==o.pc && wz==o.wz && carry==o.carry && q==o.q && t==o.t && reads==o.reads;
 }
} s;
#define regA s.a
#define REG_B s.b
#define regR s.r
#define REG_PC s.pc
#define REG_WZ s.wz
#define flagQ s.q
#define opCode s.op
bool z80_in_nocheck=true,g_next_fastrd=true,ffIFF1=false;
namespace Config { int numPcBP=0,numPortReadBP=0,numMemReadBP=0; }
namespace Rzx { int mode=0; }
uint8_t rom[8192]{};
namespace MemESP { bool divmmc_mapped=false; int romInUse=2; uint8_t* ramCurrent[]={rom}; }
namespace DivMMC { bool enabled=true,divide_mode=false; }
namespace NextReg { enum { PE_SPI }; bool spi=true; bool portOn(int){return spi;} }
namespace CPU { uint32_t& tstates=s.t; uint32_t stFrame=0; }
namespace VIDEO { uint32_t ts_line_t=0; }
void tsFastTick(uint32_t n) {s.t+=n; assert(s.t<VIDEO::ts_line_t);}
unsigned ready;
uint8_t reply;
std::vector<uint32_t> calls;
namespace Ports { uint8_t input(uint16_t p) { assert(p==0xffeb); calls.push_back(s.t); ++s.t; return s.reads++>=ready?reply:255; } }
struct Z80 { static void nextSdPollBatch(); static void cp(uint8_t n){assert(n==255 && s.a==255);s.f=0x6a;s.carry=false;s.q=true;} };
'''
main = r'''
// Independent instruction stepping for DB EB / FE FF / C0 / 10 F9.
void step(State& r, std::vector<uint32_t>& reads) {
 r.q=false; ++r.r;
 switch(r.pc) {
 case 0x1928: r.op=0xdb; reads.push_back(r.t+7);r.t+=8;r.a=r.reads++>=ready?reply:255;r.wz=0xffec;r.pc+=2;break;
 case 0x192a: assert(r.a==255);r.op=0xfe;r.t+=7;r.f=0x6a;r.carry=false;r.q=true;r.pc+=2;break;
 case 0x192c: assert(r.f&0x40);r.op=0xc0;r.t+=5;r.pc++;break;
 case 0x192d: assert(r.b!=1);r.op=0x10;r.t+=13;--r.b;r.pc=r.wz=0x1928;break;
 default: assert(false);
 }
}
int main(){
 const uint8_t code[]={0xdb,0xeb,0xfe,0xff,0xc0,0x10,0xf9};
 for(unsigned i=0;i<7;i++)rom[0x1928+i]=code[i];
 unsigned count=0;
 for(unsigned b=0;b<256;b++)for(unsigned room=0;room<1100;room++)for(unsigned when: {0u,1u,7u,31u,32u,100u}){
   s=State{};s.b=b;State ref=s;ready=when;reply=(when&1)?0:0xfe;
   CPU::stFrame=100+room;VIDEO::ts_line_t=100+((room*37)%1101);
   calls.clear();Z80::nextSdPollBatch();
   std::vector<uint32_t> reads;
   while(ref.t<s.t)step(ref,reads);
   assert(s==ref);assert(calls==reads);assert(s.t<=CPU::stFrame);
   ++count;
 }
 // Every pattern byte is required; each runtime guard must leave state intact.
 CPU::stFrame=VIDEO::ts_line_t=10000;ready=100;reply=0xfe;
 for(unsigned i=0;i<7;i++){rom[0x1928+i]^=1;s=State{};State ref=s;Z80::nextSdPollBatch();assert(s==ref);rom[0x1928+i]^=1;}
 auto unchanged=[](){s=State{};State ref=s;Z80::nextSdPollBatch();assert(s==ref);};
 for(bool* p: {&ffIFF1,&MemESP::divmmc_mapped,&DivMMC::divide_mode}){*p=true;unchanged();*p=false;}
 for(bool* p: {&z80_in_nocheck,&g_next_fastrd,&DivMMC::enabled,&NextReg::spi}){*p=false;unchanged();*p=true;}
 for(int* p: {&Config::numPcBP,&Config::numPortReadBP,&Config::numMemReadBP,&Rzx::mode}){*p=1;unchanged();*p=0;}
 MemESP::romInUse=3;unchanged();
 printf("PASS: %u differential cases, pattern and runtime guards\n",count);
}
'''
with tempfile.TemporaryDirectory(prefix="next-poll-test-") as work:
    cpp = Path(work) / "test.cpp"
    binary = Path(work) / "test"
    cpp.write_text(h + body + main)
    subprocess.run(['g++', '-std=c++17', '-O2', '-Wall', '-Wextra',
                    '-fsanitize=undefined', str(cpp), '-o', str(binary)], check=True)
    subprocess.run([str(binary)], check=True)
