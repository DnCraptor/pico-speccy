#!/usr/bin/env python3
"""Production deferred NXJMP dump and atomic UART enqueue, with host stubs."""
from pathlib import Path
import subprocess
import tempfile

root = Path(__file__).resolve().parents[1]
z80 = (root / 'src/speccy/z80/Z80_JLS.cpp').read_text()
debug = (root / 'src/app/Debug.cpp').read_text()

def function(src, signature):
    a = src.index(signature)
    b = src.index('{', a)
    depth, end = 1, b + 1
    while depth:
        depth += (src[end] == '{') - (src[end] == '}')
        end += 1
    return src[a:end]

decl = z80[z80.index('struct NxJmp {'):z80.index('// NextMf::retn():')]
body = '\n'.join(function(z80, s) for s in (
    'Z80_COLD void nxJumpArmAfter(', 'static void nxJumpDump(const char* why) {',
    'Z80_COLD void nxJumpPump()'))
enqueue = function(debug, 'bool Debug::tryLog(')
harness = r'''
#include <cassert>
#include <cstdint>
#include <cstdarg>
#include <cstdio>
#include <string>
#include <vector>
#include <algorithm>
#define Z80_COLD
constexpr unsigned DBG_TX_RING=4096;
char s_dbg_ring[DBG_TX_RING];
unsigned s_dbg_r, s_dbg_w;
bool s_dbg_uart=true, locked=false;
uint32_t dbg_lock() { assert(!locked); locked=true; return 42; }
void dbg_unlock(uint32_t s) { assert(locked && s==42); locked=false; }
bool dbg_uart_put(char c) {
 assert(locked);
 unsigned n=(s_dbg_w+1)&(DBG_TX_RING-1);
 if(n==s_dbg_r) return false;
 s_dbg_ring[s_dbg_w]=c; s_dbg_w=n; return true;
}
void dbg_uart_drain_fifo() { assert(!locked); }
struct Debug { static bool tryLog(const char*,...); };
namespace NextReg { uint8_t reg[256]; }
namespace NextInt { bool retnArmed; }
std::string drain() {
 std::string s;
 while(s_dbg_r!=s_dbg_w) { s+=s_dbg_ring[s_dbg_r]; s_dbg_r=(s_dbg_r+1)&4095; }
 return s;
}
'''
main = r'''
int main() {
 // Every available capacity, both contiguous and wrapped. Failure must leave
 // every byte/index unchanged; success must include translated CRLF and suffix.
 for(unsigned off: {0u,4090u}) for(unsigned avail=0;avail<4096;++avail) {
  s_dbg_w=off; s_dbg_r=(off+avail+1)&4095;
  std::fill_n(s_dbg_ring,4096,'!');
  bool ok=Debug::tryLog("a\nb");
  assert(ok==(avail>=6)); assert(!locked);
  if(!ok) { assert(s_dbg_w==off); for(char c:s_dbg_ring) assert(c=='!'); }
  else { s_dbg_r=off; assert(drain()=="a\r\nb\r\n"); }
 }
 s_dbg_r=s_dbg_w=0;
 assert(Debug::tryLog("%s",std::string(400,'x').c_str()));
 assert(drain()==std::string(255,'x')+"\r\n");
 s_dbg_uart=false; assert(Debug::tryLog("discard")); assert(s_dbg_w==s_dbg_r); s_dbg_uart=true;

 for(unsigned total: {0u,1u,255u,256u,257u,700u}) {
  nxJumpArmAfter(0); s_nx_jmp_w=total;
  const unsigned count=std::min(total,256u), first=total-count;
  for(unsigned i=first;i<total;++i) {
   s_nx_jmp[i&255]={}; s_nx_jmp[i&255].t='J'; s_nx_jmp[i&255].to=i;
  }
  NextReg::reg[0xC0]=0x12; NextReg::reg[0xC2]=0x34; NextReg::reg[0xC3]=0x56;
  NextInt::retnArmed=true;
  nxJumpDump("test"); assert(s_nx_jmp_done && s_dbg_w==s_dbg_r);
  NextReg::reg[0xC0]=0xFF; NextInt::retnArmed=false;
  std::vector<std::string> lines;
  for(unsigned i=0;i<=count;++i) {
   // Saturated ring: repeated pumping must preserve the same pending line.
   s_dbg_w=0; s_dbg_r=1;
   for(unsigned n=0;n<5;++n) { nxJumpPump(); assert(s_dbg_w==0); }
   s_dbg_r=s_dbg_w=0; nxJumpPump(); lines.push_back(drain());
  }
  assert(!s_nx_dump_why && lines.size()==count+1);
  assert(lines[0].find("C0=12 C2=34 C3=56 retnArmed=1")!=std::string::npos);
  for(unsigned i=0;i<count;++i) {
   char expected[40]; snprintf(expected,sizeof(expected),"[NXJMP] J %04X sp=",first+i);
   assert(lines[i+1].find(expected)==0);
  }
  nxJumpPump(); assert(drain().empty());
 }
 // Rearming cancels unsent old history instead of streaming overwritten data.
 nxJumpDump("old"); nxJumpArmAfter(192); nxJumpPump();
 assert(drain().empty() && !s_nx_jmp_done && s_nx_after==192);
 puts("PASS: atomic whole-line enqueue, ring wrap/full, captured metadata, ordered bounded pumping, rearm");
}
'''
with tempfile.TemporaryDirectory(prefix='next-jump-') as tmp:
    cpp, exe = Path(tmp)/'test.cpp', Path(tmp)/'test'
    cpp.write_text(harness + enqueue + decl + body + main)
    subprocess.run(['g++','-std=c++17','-O2','-Wall','-Wextra','-fsanitize=undefined',str(cpp),'-o',str(exe)],check=True)
    subprocess.run([str(exe)],check=True)
