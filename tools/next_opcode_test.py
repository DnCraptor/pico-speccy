#!/usr/bin/env python3
"""Compare direct Next opcode fetch with the boot6 pre/read/post hooks.

Run: python3 tools/next_opcode_test.py (requires g++).
The production hooks and slow fetch are extracted verbatim; mapping hardware
is stubbed with distinct pages. The baseline is frozen to detect filter drift.
"""
from pathlib import Path
import subprocess
import tempfile

ROOT = Path(__file__).resolve().parents[1]

def function(source, signature):
    start = source.index(signature)
    brace = source.index('{', start)
    depth = 1
    end = brace + 1
    while depth:
        depth += (source[end] == '{') - (source[end] == '}')
        end += 1
    return source[start:end]

STUB = r'''#include <initializer_list>

#include <cstdint>
#include <cstdio>
#include <cassert>
#include <cstring>
#define NEXT_TRACE 1
#define A_NEXT 1
#define DIVMMC_TRAP_0000 0x0000
#define DIVMMC_TRAP_0008 0x0008
#define DIVMMC_TRAP_0038 0x0038
#define DIVMMC_TRAP_0066 0x0066
#define DIVMMC_TRAP_04C6 0x04c6
#define DIVMMC_TRAP_0562 0x0562
#define DIVMMC_OFF_START 0x1ff8
#define DIVMMC_OFF_END 0x1fff
struct State {
 bool automap,conmem,nmi,armed,mf,boot,config,rom3;
 uint8_t pend,mask,l2,nr[256];
 unsigned maps,events,rst8,mapping;
 int arch;
 uint16_t trace;
} s;
namespace Config { int& arch=s.arch; }
namespace NextReg { uint8_t (&reg)[256]=s.nr; }
namespace MemESP {
 bool &nextBootRomActive=s.boot,&nextConfigMode=s.config,&next_rom3=s.rom3;
 uint8_t &next_rom_mask=s.mask,&l2_rd_mask=s.l2;
 uint8_t* ramCurrent[8];
}
bool &g_nxmf_mem=s.mf,&g_next_automap_armed=s.armed;
uint16_t& g_nmi_trace_left=s.trace;
void next_trace_rst8(){++s.rst8;}
void next_trace_ev(char,uint8_t,uint8_t){++s.events;}
uint8_t pages[10][8192];
void mapping(){
 for(unsigned i=0;i<8;i++)MemESP::ramCurrent[i]=pages[(i<2&&s.mapping)?8+i:i];
}
void apply(){s.mapping=s.automap||s.conmem;++s.maps;mapping();}
struct Legacy {
 static bool &automap,&conmem,&next_nmi_btn;
 static uint8_t& pend_after;
 static void applyMapping(){apply();}
 static inline void preOpcFetch(uint16_t);
 static inline void postOpcFetch();
};
struct DivMMC : Legacy {
 static inline bool opcodeTrapPage(uint16_t);
 static inline uint8_t nextOpcode(uint16_t);
 static uint8_t nextOpcodeSlow(uint16_t);
 static inline void preOpcFetch(uint16_t);
 static inline void postOpcFetch();
};
bool &Legacy::automap=s.automap,&Legacy::conmem=s.conmem,&Legacy::next_nmi_btn=s.nmi;
uint8_t& Legacy::pend_after=s.pend;
'''

REFERENCE = r'''__attribute__((always_inline)) inline void Legacy::preOpcFetch(uint16_t pc) {
    if (__builtin_expect(pc >= 0x4000, 1)) return;
    {   // 32-bit mask for pages 0x00-0x1F, one compare for 0x3D (a 64-bit mask
        const uint32_t tp = pc >> 8;
        static constexpr uint32_t kTrapPagesLo = (1u << 0x00) | (1u << 0x04) | (1u << 0x05) | (1u << 0x1F);
        if (tp != 0x3D && (tp >= 32 || !((kTrapPagesLo >> tp) & 1))) return;
    }
    pend_after = 0;

    if (Config::arch == A_NEXT) {
        if (MemESP::nextBootRomActive || MemESP::nextConfigMode) return;
        const uint8_t* nr = NextReg::reg;
        if (!(nr[0x0A] & 0x10) || !(nr[0x83] & 0x01)) {
            next_nmi_btn = false;
            if (automap) {
                automap = false;
                if (!conmem) applyMapping();
            }
            return;
        }
        { extern bool g_nxmf_mem; if (g_nxmf_mem) return; }
        extern bool g_next_automap_armed;
        if (!g_next_automap_armed) {
            if (pc != 0x0000) g_next_automap_armed = true;
        }
#if NEXT_TRACE
        { extern uint16_t g_nmi_trace_left;
          if (__builtin_expect(g_nmi_trace_left != 0, 0) && pc == 0x0008) next_trace_rst8(); }
#endif
        if (!automap) {
            bool rom3 = (MemESP::next_rom_mask & 1) && MemESP::next_rom3 &&
                        !((MemESP::l2_rd_mask >> (pc >> 13)) & 1);
            if (pc <= 0x0038 && (pc & 7) == 0) {
                uint8_t bit = (uint8_t)(1u << (pc >> 3));
                if (pc == 0x0000 && !g_next_automap_armed) bit = 0;
                if (bit && (nr[0xB8] & bit) && ((nr[0xB9] & bit) || rom3)) {
                    pend_after |= 1;
                }
            } else if (pc == 0x0066) {
                if (next_nmi_btn && (nr[0xBB] & 0x03)) {
                    next_nmi_btn = false;          // divmmc.vhd: cleared by automap_held
                    if (nr[0xBB] & 0x02) {
                        automap = true;
                        if (!conmem) applyMapping();
                    } else {
                        pend_after |= 1;
                    }
                }
            } else if (rom3) {
                if ((pc == 0x04C6 && (nr[0xBB] & 0x04)) ||
                    (pc == 0x0562 && (nr[0xBB] & 0x08)) ||
                    (pc == 0x04D7 && (nr[0xBB] & 0x10)) ||
                    (pc == 0x056A && (nr[0xBB] & 0x20))) {
                    pend_after |= 1;
                } else if ((pc & 0xFF00) == 0x3D00 && (nr[0xBB] & 0x80)) {
                    automap = true;
                    if (!conmem) applyMapping();
                }
            }
        }
        if (automap && pc >= DIVMMC_OFF_START && pc <= DIVMMC_OFF_END &&
            (nr[0xBB] & 0x40)) {
            pend_after |= 2;
        }
        return;
    }

#if NEXT_TRACE
    { extern uint16_t g_nmi_trace_left;
      if (__builtin_expect(g_nmi_trace_left != 0, 0) && pc == 0x0008) next_trace_rst8(); }
#endif
    if (!automap) {
        if (pc == DIVMMC_TRAP_0000 || pc == DIVMMC_TRAP_0008 ||
            pc == DIVMMC_TRAP_0038 || pc == DIVMMC_TRAP_0066 ||
            pc == DIVMMC_TRAP_04C6 || pc == DIVMMC_TRAP_0562) {
            pend_after |= 1;
        }
    }

    if (pc >= 0x3D00 && pc <= 0x3DFF) {
        if (!automap) {
            automap = true;
            if (!conmem) applyMapping();
        }
    }

    if (automap && pc >= DIVMMC_OFF_START && pc <= DIVMMC_OFF_END) {
        pend_after |= 2;
    }
}

__attribute__((always_inline)) inline void Legacy::postOpcFetch() {
    const uint8_t p = pend_after;
    if (__builtin_expect(p == 0, 1)) return;
    pend_after = 0;
    if (p & 1) {
        automap = true;
        if (!conmem) applyMapping();
#if NEXT_TRACE
        next_trace_ev('A', 0x01, 0);   // records PC itself
#endif
    }
    if (p & 2) {
        automap = false;
        if (!conmem) applyMapping();
#if NEXT_TRACE
        next_trace_ev('A', 0x00, 0);
#endif
    }
}'''

MAIN = r'''
uint8_t baseline(uint16_t pc){
 Legacy::preOpcFetch(pc);
 const uint8_t op=MemESP::ramCurrent[pc>>13][pc&8191];
 Legacy::postOpcFetch();return op;
}
uint32_t rng=1;
uint32_t rand32(){rng^=rng<<13;rng^=rng>>17;rng^=rng<<5;return rng;}
State randomState(){
 State q{};
 q.automap=rand32()&1;q.conmem=rand32()&1;q.nmi=rand32()&1;q.armed=rand32()&1;
 q.mf=!(rand32()%7);q.boot=!(rand32()%13);q.config=!(rand32()%11);q.rom3=rand32()&1;
 q.mask=rand32()&3;q.l2=rand32()&3;
 q.arch=(rand32()%4)?A_NEXT:0;q.trace=rand32()&1;
 for(unsigned r: {0x0au,0x83u,0xb8u,0xb9u,0xbbu})q.nr[r]=rand32();
 q.mapping=q.automap||q.conmem;return q;
}
void compare(uint16_t pc){
 const State initial=s;
 mapping();const uint8_t want=baseline(pc);const State expected=s;
 s=initial;mapping();const uint8_t got=DivMMC::nextOpcode(pc);
 assert(got==want);assert(!memcmp(&s,&expected,sizeof s));assert(!s.pend);
 for(unsigned i=0;i<8;i++)assert(MemESP::ramCurrent[i]==pages[(i<2&&s.mapping)?8+i:i]);
}
int main(){
 for(unsigned p=0;p<10;p++)for(unsigned i=0;i<8192;i++)pages[p][i]=(p*41+i*7+(i>>8))&255;
 unsigned count=0;
 for(unsigned pc=0;pc<65536;pc++){
   unsigned page=pc>>8;
   assert(DivMMC::opcodeTrapPage(pc)==(page==0||page==4||page==5||page==31||page==61));
   for(unsigned n=0;n<128;n++){s=randomState();compare(pc);++count;}
 }
 // Sequential M1s carry mapping changes into the next fetch, including RAM,
 // delayed entry, instantaneous entry, off-points, and 16-bit PC wrap.
 for(unsigned n=0;n<10000;n++){
   s=randomState();
   for(uint16_t pc: {0x0000,0x0001,0x0008,0x0038,0x0066,0x04c6,0x056a,0x3d00,0x4000,0x1ff8,0x1fff,0xffff,0x0000}){compare(pc);++count;}
 }
 printf("PASS: %u opcode/mapping comparisons, all 65536 addresses\n",count);
}
'''

header = (ROOT / 'src/speccy/devices/storage/DivMMC.h').read_text()
cpu = (ROOT / 'src/speccy/z80/CPU.cpp').read_text()
parts = [STUB, REFERENCE]
for signature in (
    '__attribute__((always_inline)) inline bool DivMMC::opcodeTrapPage',
    '__attribute__((always_inline)) inline uint8_t DivMMC::nextOpcode(',
    '__attribute__((always_inline)) inline void DivMMC::preOpcFetch',
    '__attribute__((always_inline)) inline void DivMMC::postOpcFetch',
):
    parts.append(function(header, signature))
parts.append(function(cpu, 'IRAM_ATTR __attribute__((noinline, optimize("O2"))) uint8_t DivMMC::nextOpcodeSlow').replace('IRAM_ATTR ', ''))
parts.append(MAIN)
with tempfile.TemporaryDirectory(prefix='next-opcode-test-') as work:
    cpp = Path(work) / 'test.cpp'
    binary = Path(work) / 'test'
    code = '\n'.join(parts)
    # Bind these production globals to the saved test state via references.
    for declaration in ('extern bool g_nxmf_mem;', 'extern bool g_next_automap_armed;',
                        'extern uint16_t g_nmi_trace_left;'):
        code = code.replace(declaration, '')
    cpp.write_text(code)
    subprocess.run(['g++', '-std=c++17', '-O2', '-Wall', '-Wextra',
                    '-fsanitize=undefined', str(cpp), '-o', str(binary)], check=True)
    subprocess.run([str(binary)], check=True)
