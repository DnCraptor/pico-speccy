#!/usr/bin/env python3
"""Exercise the production deferred UART init against a FIFO/time model."""
from pathlib import Path
import subprocess
import tempfile

root = Path(__file__).resolve().parents[1]
src = (root / 'src/speccy/devices/zifi/ZiFi.cpp').read_text()
a = src.index('static uint8_t g_init_phase')
b = src.index('// Switch the ESP-01S', a)
body = src[a:b]
at = (root / 'src/net/ZiFiAT.cpp').read_text()
a = at.index('    if (as_state == AS_INIT) {')
b = at.index('\n    if (as_state == AS_RETRY)', a)
at_step = at[a:b]
harness = r'''
#include <cstdint>
#include <cassert>
#include <cstring>
#include <string>
#include <cstdio>
unsigned credit, rx, switches, sleeps;
uint32_t now, switched_at, g_cur_baud;
bool irq;
std::string sent;
struct Data {
 void operator=(uint32_t b) { assert(credit); --credit; sent += char(b); }
 operator uint32_t() { assert(rx); --rx; return 0; }
};
struct Uart { Data dr; uint32_t fr; } hw;
Uart* g_uart = &hw;
unsigned g_uart_irq = 1;
constexpr uint32_t UART_UARTFR_BUSY_BITS = 8;
Uart* uart_get_hw(Uart* p) { return p; }
bool uart_is_writable(Uart*) { return credit != 0; }
bool uart_is_readable(Uart*) { return rx != 0; }
uint32_t time_us_32() { return now; }
void uart_set_baudrate(Uart*, uint32_t rate) {
 assert(rate == 230400); switched_at = now; ++switches;
}
void irq_set_enabled(unsigned n, bool on) { assert(n == 1); irq = on; }
void sleep_ms(unsigned ms) { assert(ms == 1); ++sleeps; now += 1000; credit = 32; hw.fr = 0; }
struct ZiFi { static bool initReady(); };
'''
at_harness = r'''
enum { AS_INIT, AS_CWMODE };
int as_state;
uint32_t as_deadline;
unsigned commands;
uint32_t as_now() { return now / 1000; }
void as_send(const char* cmd) {
 assert(irq && g_init_phase==0 && g_cur_baud==230400);
 assert(!strcmp(cmd,"AT+CWMODE=1")); ++commands;
}
'''
main = r'''
void reset(uint32_t t) {
 now=t; credit=0; rx=32; switches=sleeps=0; irq=false; hw.fr=0;
 sent.clear(); g_cur_baud=115200; g_init_target=230400;
 strcpy(g_init_cmd,"AT+UART_CUR=230400,8,1,0,0\r\n");
 g_init_len=strlen(g_init_cmd); g_init_pos=0; g_init_phase=1;
}
int main() {
 for (unsigned wrap=0;wrap<2;++wrap) for (unsigned chunk=1;chunk<=32;++chunk) {
  reset(wrap ? 0xFFFF0000u : 1000u);
  const uint32_t start=now;
  for(unsigned i=0;i<10;++i) assert(!ZiFi::initReady()); // FIFO full, no spin
  assert(now==start && sent.empty());
  while(g_init_phase==1) { credit=chunk; assert(!ZiFi::initReady()); }
  assert(sent==g_init_cmd && switches==0);
  hw.fr=UART_UARTFR_BUSY_BITS;
  for(unsigned i=0;i<10;++i) assert(!ZiFi::initReady());
  hw.fr=0; assert(!ZiFi::initReady()); // deadline starts after wire drained
  now += 79999; assert(!ZiFi::initReady() && switches==0);
  ++now; assert(!ZiFi::initReady() && switches==1 && g_cur_baud==230400);
  assert(!irq && rx==32);
  now += 19999; assert(!ZiFi::initReady());
  ++now; assert(ZiFi::initReady() && irq && rx==0 && sleeps==0);
  assert(uint32_t(now-switched_at)==20000);
  assert(ZiFi::initReady() && switches==1);
 }
 // Foreground takeover must complete every partially finished phase.
 for(unsigned phase=1;phase<=4;++phase) {
  reset(0xFFFF0000u);
  while(g_init_phase < phase) { credit=32; ZiFi::initReady(); now+=1000; }
  zifi_finish_init();
  assert(irq && sent==g_init_cmd && switches==1 && g_init_phase==0);
  assert(uint32_t(now-switched_at)>=20000);
 }
 // Continuous noise cannot make the final discard unbounded.
 reset(1000); g_init_phase=4; g_init_deadline=now; rx=1000;
 assert(ZiFi::initReady() && rx==968);
 // Actual AS_INIT branch must neither send commands nor start their response
 // timeout before the negotiated baud is ready.
 reset(1000); as_state=AS_INIT; as_deadline=0; commands=0;
 for(unsigned i=0;as_state==AS_INIT && i<200;++i) {
  credit=32; autoStep();
  if(as_state==AS_INIT) assert(commands==0 && as_deadline==0);
  else assert(as_deadline==as_now()+2000);
  now+=1000;
 }
 assert(as_state==AS_CWMODE && commands==1);
 puts("PASS: FIFO backpressure, TX busy, 80/20 ms timing, timer wrap, foreground takeover, bounded RX drain");
}
'''
with tempfile.TemporaryDirectory(prefix='zifi-init-') as tmp:
    cpp = Path(tmp) / 'test.cpp'
    exe = Path(tmp) / 'test'
    cpp.write_text(harness + body + at_harness + '\nvoid autoStep() {\n' + at_step + '\n}\n' + main)
    subprocess.run(['g++', '-std=c++17', '-O2', '-Wall', '-Wextra', '-fsanitize=undefined',
                    str(cpp), '-o', str(exe)], check=True)
    subprocess.run([str(exe)], check=True)
