#!/usr/bin/env python3
"""Exercise native NEX launch setup and run its actual Z80 bootstrap against API stubs."""
from pathlib import Path
import subprocess
import tempfile
root = Path(__file__).resolve().parents[1]
source = (root / 'src/speccy/machines/Next/NexLaunch.cpp').read_text()
source = source[source.index('namespace NexLaunch {'):]
# The stubs below stand in for every header the file includes (the real ones
# pull fatfs/SDK types that would clash), so drop its includes.
source = "\n".join(l for l in source.splitlines() if not l.lstrip().startswith('#include'))
stub = r'''
#include <cassert>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <string>
#include <vector>
#include <cstdio>
#include "speccy/machines/Next/NexLaunchCode.h"
#include "speccy/machines/Next/NexLaunch.h"
constexpr int FR_OK=0, LEVEL_WARN=1;
struct FILINFO { char altname[13],fname[256]; };
bool missing=false, missingDot=false;
int f_stat(const char* p, FILINFO* f){
    if(missing || (missingDot && !strcmp(p,"/dot/NEXLOAD")))return 1;
    const char* base=strrchr(p,'/'); base=base?base+1:p;
    strcpy(f->fname,base); f->altname[0]=0;
    if(!strcmp(base,"Battle Tanks Long Name"))strcpy(f->altname,"BATTLE~2");
    if(!strcmp(base,"BattleTanks.nex"))strcpy(f->altname,"BATTLE~1.NEX");
    return FR_OK;
}
struct ScopedHeap {void* p;ScopedHeap(size_t n):p(malloc(n)){}~ScopedHeap(){free(p);}
    explicit operator bool()const{return p;}template<class T>T* as(){return (T*)p;}};
namespace Z80Ops { bool isNext=true; }
namespace MemESP {
 uint8_t memory[8][16384], rom[65536],*nextRomBase=rom;
 bool nextRamReady=true,nextBootRomActive=false,nextConfigMode=false;
 // The emulator's live 8K map (NextReg 0x50..0x57 / the legacy ports feed it).
 uint8_t mmu[8]={0xFF,0xFF,0x0A,0x0B,0x04,0x05,0x00,0x01};
 uint8_t pagingLock=0, romInUse=3, divmmc_mapped=0, altrom_rd=0, altrom_sel=0, *altRomBase=nullptr;
 uint8_t dbgPeek(uint16_t a){
   const uint8_t b=mmu[a>>13];
   const unsigned off=(b==0xFF)?(a&0x3FFF):((a&0x1FFF)+((b&1)<<13));
   return memory[(b==0xFF?0:(b>>1))&7][off&0x1FFF];
 }
 struct Page{uint8_t* p;uint8_t* direct(){return p;}}ram[8];
}
namespace NextReg {uint8_t reg[256],port1FFD,port7FFD,portDFFD;
 // +3/Next ROM select: rom = 1FFD.2:7FFD.4 (NextReg.cpp applyRomSelect); 0x7FFD
 // also drives the MMU 0xC000 pair (applyBankC000).
 void romSel(){MemESP::romInUse=(uint8_t)((((port1FFD>>2)&1)<<1)|((port7FFD>>4)&1));}
 void write(uint8_t r,uint8_t v){
   reg[r]=v;
   if(r>=0x50&&r<=0x57)MemESP::mmu[r-0x50]=v;
 }
 void writeLayer2Port(uint8_t){}
 void writeDFFD(uint8_t v){portDFFD=v;reg[0xdf]=v;}
 void write1FFD(uint8_t v){port1FFD=v;romSel();}
 void write7FFD(uint8_t v){port7FFD=v;uint8_t b=v&7;MemESP::mmu[6]=b*2;MemESP::mmu[7]=b*2+1;romSel();}}
namespace DivMMC {bool enabled=true,divsd_mode=true,automap=false,conmem=false;void applyMapping(){}}
namespace NEXTVID {void c1Drain(){}}
namespace ESPectrum {int resets=0;void reset(){++resets;}}
namespace OSD {int errors=0;std::string last;void osdCenteredMsg(const std::string& m,int,int){++errors;last=m;}}
namespace Debug {template<class...T>void log(const char*,T...){} }
struct Z80 {
 static inline uint16_t pc,sp,iy,hl,althl; static inline uint8_t a,d,i;static inline bool halted,iff,fz;
 enum{IM1=1};
 static void reset(){pc=0;halted=false;iff=false;i=0;fz=false;hl=althl=0;}
 static void setRegIY(uint16_t v){iy=v;} static void setRegPC(uint16_t v){pc=v;} static void setRegSP(uint16_t v){sp=v;}
 static void setRegI(uint8_t v){i=v;}static void setRegHLx(uint16_t v){althl=v;}
 static uint16_t getRegSP(){return sp;}static bool isIFF1(){return iff;}
 static void setIM(int){}static uint16_t getRegPC(){return pc;}static uint16_t getRegIY(){return iy;}
 static uint16_t getRegHL(){return hl;}
 static bool isHalted(){return halted;}static uint8_t getRegA(){return a;}static uint8_t getRegD(){return d;}
};
'''
tests = r'''
void fixture(){
 for(int i=0;i<8;++i){MemESP::ram[i].p=MemESP::memory[i];memset(MemESP::memory[i],0x5a,16384);}
 MemESP::nextBootRomActive=MemESP::nextConfigMode=false;
 MemESP::pagingLock=0;MemESP::romInUse=3;
 NextReg::port1FFD=NextReg::port7FFD=NextReg::portDFFD=0;
 ESPectrum::resets=OSD::errors=0;missing=missingDot=false;DivMMC::divsd_mode=true;
 NexLaunch::cancel();
}
using Format = NexLaunch::Format;
void run(int failStage, Format format=Format::NEX){
 const bool snx=format==Format::SNX, bas=format==Format::BAS;
 fixture();
 // BAS: the OS side of the launch. IdeBasic wants RAMTOP above the stack it is
 // given; the host writes RAMTOP/ERR_NR before it starts (the line's own CLEAR
 // then does the real workspace reset, in guest code).
 if(bas){uint8_t* s=MemESP::memory[5];s[0x1CB2]=0x57;s[0x1CB3]=0xFF;s[0x1C3A]=0xFF;s[0x1C59]=0xCC;s[0x1C5A]=0x5C;}
 assert(NexLaunch::start(bas ? "/games/Battle Tanks Long Name/TEST.BAS" : snx ? "/games/Battle Tanks Long Name/POGIE.SNX" : "/games/Battle Tanks Long Name/BattleTanks.nex", format)==1);
 assert(ESPectrum::resets==0 && Z80::iy==0x5c3a);
 // Two stacks, and the split is load-bearing: the tokeniser is a C=7 call that
 // pages RAM7 at $c000, so its stack sits below $bfe0 (bank 2), while IdeBasic
 // wants the BASIC one high in bank 0 (between STKEND and RAMTOP).
 assert(Z80::sp==(bas?NexLaunchCode::basEntryStack:NexLaunchCode::stack));
 assert(MemESP::memory[5][0x1B5C]==((bas||snx)?0x10:0x5a));   // BANKM for M_P3DOS
 assert(NextReg::port1FFD==4 && NextReg::port7FFD==0x10);     // = ROM3 until the loader sets up
 if(bas){
   // The host-tokenised line in E_LINE: LOAD "c:/path":RUN, #0D, #80, and the
   // workspace pointers just after it.
   const uint8_t* e=&MemESP::memory[5][0x1CCC];
   const std::string path="c:/games/BATTLE~2/TEST.BAS";
   assert(e[0]==0xEF&&e[1]=='"'&&!memcmp(e+2,path.data(),path.size()));
   const uint8_t* t=e+2+path.size();
   assert(t[0]=='"'&&t[1]==':'&&t[2]==0xF7&&t[3]==0x0D&&t[4]==0x80);
   const unsigned ws=0x5CCC+2+path.size()+5;
   auto w=[](unsigned a){return unsigned(MemESP::memory[5][a-0x4000]|(MemESP::memory[5][a-0x4000+1]<<8));};
   assert(w(0x5C61)==ws&&w(0x5C63)==ws&&w(0x5C65)==ws&&w(0x5C5B)==0x5CCC);
   assert(w(0x5B6A)==0x5BFF);                          // DOS-side stack = TSTACK
 }
 for(auto b:MemESP::memory[5])if(b!=0x5a)assert(bas||snx);        // only the BAS/SNX sysvars
 for(auto b:MemESP::memory[0])assert(b==0x5a);                   // banks loaded by the OS, not host
 uint16_t ix=0,de=0;uint8_t c=0;bool carry=false;std::vector<int> calls;
 auto byte=[](uint16_t a){assert(a>=0x8000&&a<0xc000);return MemESP::memory[2][a-0x8000];};
 for(int step=0;step<60;++step){
  auto op=byte(Z80::pc++);
  switch(op){
   case 0xf3:Z80::iff=false;break;
   case 0xfb:Z80::iff=true;break;
   case 0xaf:Z80::a=0;Z80::fz=true;break;
   case 0xd9:std::swap(Z80::hl,Z80::althl);break;
   case 0x21:Z80::hl=byte(Z80::pc++);Z80::hl|=byte(Z80::pc++)<<8;break;
   case 0x11:de=byte(Z80::pc++);de|=byte(Z80::pc++)<<8;break;
   case 0x01:{uint16_t bc=byte(Z80::pc++);bc|=byte(Z80::pc++)<<8;c=(uint8_t)bc;break;}
   case 0x0e:c=byte(Z80::pc++);break;
   case 0x16:Z80::d=byte(Z80::pc++);break;
   case 0x31:Z80::sp=byte(Z80::pc++);Z80::sp|=byte(Z80::pc++)<<8;
             if(bas)assert(Z80::sp==NexLaunchCode::basStack);
             break;
   case 0x32:{uint16_t ad=byte(Z80::pc++);ad|=byte(Z80::pc++)<<8;
              assert(bas&&ad==0x5B5C);MemESP::memory[5][ad-0x4000]=Z80::a;break;} // LD (BANKM),A
   case 0x2a:{uint16_t ad=byte(Z80::pc++);ad|=byte(Z80::pc++)<<8;               // LD HL,(E_LINE)
              assert(bas&&ad==0x5C59);Z80::hl=MemESP::memory[5][0x1C59]|(MemESP::memory[5][0x1C5A]<<8);break;}
   case 0x3a:{uint16_t ad=byte(Z80::pc++);ad|=byte(Z80::pc++)<<8;
              assert(bas&&ad==0x5C3A);Z80::a=MemESP::memory[5][ad-0x4000];break;} // LD A,(ERR_NR)
   case 0x3e:Z80::a=byte(Z80::pc++);break;
   case 0xfd:assert(bas&&byte(Z80::pc++)==0xcb&&byte(Z80::pc++)==0x01&&byte(Z80::pc++)==0xfe);
             MemESP::memory[5][0x1C3B]|=0x80;break;      // SET 7,(FLAGS): run mode
   case 0xdd:assert(byte(Z80::pc++)==0x21);ix=byte(Z80::pc++);ix|=byte(Z80::pc++)<<8;break;
   case 0x38:{int8_t delta=(int8_t)byte(Z80::pc++);if(carry)Z80::pc+=delta;break;}
   case 0x30:{int8_t delta=(int8_t)byte(Z80::pc++);if(!carry)Z80::pc+=delta;break;}
   case 0x28:{int8_t delta=(int8_t)byte(Z80::pc++);if(Z80::fz)Z80::pc+=delta;break;}
   case 0x18:{int8_t delta=(int8_t)byte(Z80::pc++);Z80::pc+=delta;break;}
   case 0xed:assert(byte(Z80::pc++)==0x79);            // OUT (C),A
             assert(bas&&c==0xFD);                     // ... to $7ffd: the ROM2 select
             assert(Z80::a==0 && Z80::d==4);
             NextReg::write7FFD(Z80::a);
             assert(MemESP::romInUse==2);              // 1FFD.2 alone: ROM2, not ROM0
             break;
   case 0xcd:{uint16_t ad=byte(Z80::pc++);ad|=byte(Z80::pc++)<<8;
              assert(bas&&ad==0x01C0);                 // IdeBasic, called directly
              assert(Z80::hl==0x5CCC);                 // E_LINE: the host-built line
              assert(Z80::sp==NexLaunchCode::basStack); // between STKEND and RAMTOP
              assert(Z80::iff&&Z80::iy==0x5c3a);
              assert(MemESP::memory[5][0x1C3B]&0x80);   // FLAGS: run, not syntax-check
              assert(NextReg::port1FFD==4&&NextReg::port7FFD==0);
              const uint8_t want[8]={0xFF,0xFF,0x0A,0x0B,0x04,0x05,0x00,0x01};
              for(int i=0;i<8;++i)assert(MemESP::mmu[i]==want[i]); // ROM2/RAM5/RAM2/RAM0
              assert(MemESP::memory[5][0x1B5C]==0&&MemESP::memory[5][0x1B67]==0x04);
              assert(NexLaunch::pending);              // the line owns the machine now
              return;
             }
   case 0xcf:{
    int api=byte(Z80::pc++);calls.push_back(api);
    if(api!=0x94)assert(Z80::d==calls.size());
    if(api==0x89)assert(Z80::a==0x11);
    else if(api==0xa9){assert(Z80::a=='*');assert(!strcmp((char*)&MemESP::memory[2][ix-0x8000],"/games/BATTLE~2"));}
    else if(snx){
     assert(api==0x94 && de==0xfd && c==7 && Z80::althl==0x8200);
     const unsigned char want[]={'P','O','G','I','E','.','S','N','X',255};
     assert(!memcmp(&MemESP::memory[2][Z80::althl-0x8000],want,sizeof(want)));
     assert(NextReg::reg[0xdf]==0);
    }
    else {assert(api==0x8f);assert(!strcmp((char*)&MemESP::memory[2][ix-0x8000],"nexload BATTLE~1.NEX"));}
    const bool failed=failStage==int(calls.size());
    carry=api==0x94 ? !failed : failed;Z80::a=failed?5:0;Z80::fz=false;
    if(!failed && (api==0x8f || (api==0x94&&snx))){ // the real loader replaces bank 2 and jumps
     memset(MemESP::memory[2],0,16384);Z80::pc=0xdc00;NexLaunch::poll();
     assert(!NexLaunch::pending && ESPectrum::resets==0);return;
    }
    break;
   }
   case 0x76:
    assert(Z80::pc==(snx ? NexLaunchCode::snxStop : bas ? NexLaunchCode::basStop : NexLaunchCode::stop)+1);Z80::halted=true;NexLaunch::poll();
    assert(calls.size()==unsigned(failStage));assert(OSD::errors==1&&ESPectrum::resets==1);return;
   default:assert(false);
  }
 }
 assert(false);
}
int main(){
 for(int fail=0;fail<=3;++fail){run(fail);run(fail,Format::SNX);if(fail<3)run(fail,Format::BAS);}
 fixture();MemESP::nextBootRomActive=true;assert(NexLaunch::start("/game.nex")==0);
 fixture();DivMMC::divsd_mode=false;assert(NexLaunch::start("/game.nex")==0);
 fixture();missing=true;assert(NexLaunch::start("/missing.nex")==-1);assert(ESPectrum::resets==0);
 fixture();assert(NexLaunch::start("/games/../file.nex")==-1);
 fixture();assert(NexLaunch::start("/game.nex")==1);
 assert(!strcmp((char*)MemESP::memory[2]+0x100,"/"));
 NexLaunch::cancel();assert(!NexLaunch::pending);
 fixture();missingDot=true;assert(NexLaunch::start("/game.snx",Format::SNX)==1);
 MemESP::memory[5][0x1C59]=0xCC;MemESP::memory[5][0x1C5A]=0x5C;
 NexLaunch::cancel();assert(NexLaunch::start("/game.bas",Format::BAS)==1);
 NexLaunch::cancel();assert(NexLaunch::start("/game.nex")==-1);
 fixture();MemESP::nextBootRomActive=true;assert(NexLaunch::start("/game.snx",Format::SNX)==0);
 fixture();DivMMC::divsd_mode=false;assert(NexLaunch::start("/game.snx",Format::SNX)==0);
 // Second launch while a program owns the machine: reset (= F11) first,
 // the launch itself only once NextZXOS sits idle (RAM7 at the top).
 fixture();assert(NexLaunch::start("/game.nex")==1);
 assert(NexLaunch::start("/game.snx",Format::SNX)==1);
 assert(ESPectrum::resets==1&&!NexLaunch::pending);
 MemESP::nextBootRomActive=MemESP::nextConfigMode=false;
 for(int f=0;f<70;++f)NexLaunch::poll();
 assert(!NexLaunch::pending);                       // still waiting (min boot time)
 NextReg::write7FFD(0x17);MemESP::mmu[0]=0xFF;
 for(int f=0;f<40;++f)NexLaunch::poll();
 assert(NexLaunch::pending&&!strcmp(NexLaunch::launchKind,"SNX"));
 puts("Native NEX/SNX/BAS launch: aliases, all API sequences, errors, OS workspace and fallback passed");
}
'''
with tempfile.TemporaryDirectory(prefix='nex-native-') as tmp:
    cpp = Path(tmp)/'test.cpp'; exe = Path(tmp)/'test'
    cpp.write_text(stub+source+tests)
    subprocess.run(['g++','-std=c++17','-Wall','-Wextra','-Werror','-I'+str(root/'src'),str(cpp),'-o',str(exe)],check=True)
    subprocess.run([str(exe)],check=True)

# Optional local reproductions: decode the actual bank stream independently.
# Both programs load their title/resources through the OS, not a NEX screen.
for name, resource in [("BattleTanks.nex", b"./res/title.bin"),
                       ("BombJack.nex", b"bombjack_ini.bmp")]:
    files = list((root / "debug/NEX").rglob(name))
    for path in files:
        data = path.read_bytes()
        assert data[:4] == b"Next" and data[10] == 0
        banks = {}
        at = 512
        for bank in [5, 2, 0, 1, 3, 4, 6, 7] + list(range(8, 112)):
            if data[18 + bank]:
                banks[bank] = data[at:at + 16384]
                assert len(banks[bank]) == 16384
                at += 16384
        assert len(banks) == data[9] and at == len(data)
        assert resource in b"".join(banks.values())
        assert (path.parent / resource.decode()).is_file()
        print(f"Fixture {path.parent.name}/{name}: {len(banks)} banks, external title confirmed")
