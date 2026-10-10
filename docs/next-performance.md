# Spectrum Next performance follow-up (2026-09-25)

## 2026-10-10: boot13 deferred reset history dump

The trace-only NXJMP dump previously stopped CPU::loop for ~2.45 s by
printing 256 lines with uartFlushSync every eight lines and at the end. A
guest NR 02 reset is a normal boot transition, so the comment assuming the
machine was already dead did not apply. The trigger now freezes the existing
ring and captures the header's NextRegs/RETN state without printing. Once
per Next frame, nxJumpPump offers at most one line to Debug::tryLog; this
atomically enqueues a complete line or returns immediately if space is
insufficient. The producer advances only after success. No duplicate ring
allocation is needed, and no wait for the UART wire remains in this dump.

The full history takes at least 257 frames to enqueue (header + 256 rows),
longer under UART backpressure, while emulation continues. A new snapshot
launch supersedes unsent history from the previous dump, preventing output
of overwritten ring entries. Turning off the console discards pending lines.
The frame's reported trace time now includes nxJumpPump in addition to the
health dump. CPU execution/timing and no-trace firmware behavior are unchanged.

`tools/next_jump_dump_test.py` extracts the actual queue function and dump
producer. UBSan tests pass for every available TX-ring capacity at contiguous
and wrapped positions, newline expansion/truncation, disabled console,
empty/partial/full/wrapped history, repeated backpressure, frozen header
metadata, one-line pumping and rearming. Hardware confirmation of the removed
reset stall remains pending; loading's 27 ms CPU frames are a separate issue.

Both PICO_DV builds pass. Main SRAM normal/trace is 163,072 / 181,120 bytes
(normal unchanged, trace -32 bytes after layout changes). The Next overlay
is 34,128 / 34,816 bytes (trace +8, within the 35,840-byte window). Artifacts:
`debug/DVp2-next-boot13[-trace]-1.0.9.{elf,uf2}`. Use the trace build through
reset and allow at least several seconds for the deferred NXJMP output.

## 2026-10-10: boot12 hardware result

`logs/devttyACM0_2026_10_10.19.42.59.396.txt` no longer contains the
~103 ms background stall during ZiFi startup. The corresponding transition
interval's minimum IDL improves from -102,683 to -5,085 us; its worst frame
is now CPU 24,549 us, post 466 us, net/uart 31 us. The logged startup baud
115200 is expected before the deferred negotiation; the log alone does not
prove successful ESP connection or the final baud.

TBBLUE remains 50 FPS in stable intervals, zero negative IDL, but CPU time
is 17.4 ms versus boot11's 17.2 and minimum IDL is only +102/+122 us in the
trace build. Do not claim a menu throughput improvement from boot12.
Loading remains 43 FPS in the mixed-clock interval, 27/43 negative frames,
worst CPU 27,281 us (health trace 0). The synchronous NXJMP reset dump still
costs ~2.45 s. Settled NextZXOS intervals show 50 FPS, ~0.6 ms CPU and ample
positive IDL. Remaining work: CPU loading throughput and the reset trace
dump; the observed 100 ms GPIO-UART boot sleep has been removed.

## 2026-10-10: boot12 deferred GPIO-UART startup

Inspection of the function identified by boot11 finds the 100 ms wait:
`netBackgroundTick -> ZiFiAT::autoSyncBegin -> ZiFi::init -> zifi_set_baud`
sends AT+UART_CUR, waits for TX idle, sleeps 80 ms, changes Pico baud, then
sleeps 20 ms. The log shows GPIO UART0 ZiFi startup at the configured 230400
baud around the stall. These waits account for the observed ~103 ms duration;
the hardware confirmation of their removal is still pending.

`ZiFi::init(true)` now defers that GPIO-UART negotiation. `initReady` sends
only bytes the TX FIFO accepts, polls wire idle, preserves the 80 ms and
20 ms deadlines across calls, then discards at most one RX FIFO and enables
the RX interrupt. ZiFiAT's new AS_INIT phase sends CWMODE and starts its
response timeout only after negotiation completes. No emulated frame waits
through the settling delays. The normal guest/interactive startup contract
remains synchronous; foreground raw I/O, baud changes, probes and teardown
finish an in-flight negotiation before proceeding. USB-CDC negotiation is
unchanged. This is not a blanket conversion of every AT send/RX loop to
nonblocking I/O, nor a fix for the CPU loading bottleneck or NXJMP dump.

`python3 tools/zifi_init_test.py` passes with UBSan using the extracted
production negotiation and AS_INIT branch. It covers FIFO backpressure,
wire-busy polling, exact settling deadlines, 32-bit timer wrap, takeover
from every negotiation phase, bounded RX draining, and command/timeout
ordering. Normal and trace PICO_DV builds pass, as does diff whitespace
validation. Main SRAM is 163,072 / 181,152 bytes (+64 each over boot11).
The Next overlay remains 34,128 / 34,808 bytes. Artifacts:
`debug/DVp2-next-boot12[-trace]-1.0.9.{elf,uf2}`. Check the boot transition
and WiFi connection, and confirm `NEXT worst bg` no longer reports a ~103 ms
net/uart stall; FPS changes remain unmeasured.

## 2026-10-10: boot11 hardware result

`logs/devttyACM0_2026_10_10.19.36.28.658.txt` localizes the background stall
to `netBackgroundTick`: net/uart 103,193 us of background 103,222 us.
ZiFi tick, RTC, overlays, NVRAM24 and serial mouse each take 1–7 us in that
same frame. This is not an RTC/overlay/storage-watch stall. The function
contains UART pumping, HDMI diagnostic logging and ZiFiAT auto-connect/SNTP;
on-chip WiFi is compiled out. UART0 ZiFi initialization is visible shortly
before the stall. Auto-connect's as_send uses synchronous sendRaw and drains
RX without a work limit, so the nominally background state machine is a
candidate requiring inspection; this log does not identify the inner call.

No clear boot11 performance gain over boot10: stable TBBLUE stays 17.2 ms,
50 FPS, neg=0/50 with minimum IDL +349/+355 us including health trace. The
mixed-clock loading interval remains 43 FPS / 18.3 ms CPU, with a 27,245 us
worst CPU frame (previously 27,347 us; too small to claim an improvement).
The synchronous reset dump remains ~2.45 s. Unlike boot10, this capture also
shows settled NextZXOS: 50 FPS, CPU 0.5–0.6 ms, zero negatives in clean
intervals and minimum IDL about +17.3 ms. Remaining work is boot/loading and
the network-background stall, not settled-menu throughput.

## 2026-10-10: boot11 divMMC fetch and finer background timing

Build `DivMMC::nextOpcodeSlow` with O2 instead of the firmware's size
optimization. This is the RAM function used on automap trap pages, including
the low-ROM code reached during loading. The pre-fetch mapping, opcode read
and post-fetch mapping remain in the same order; no guest instructions or
cycles are skipped. The normal function grows from 888 to 932 bytes.
`tools/next_opcode_test.py` extracts the production implementation and passes
8,518,608 comparisons against frozen mapping hooks with UBSan, covering all
guest addresses and mapping transitions. A speed benefit is unmeasured.

Add `NEXT worst bg` to the trace's saved worst-frame report, splitting the
background section into ZiFi, RTC flush, overlay materialization, NVRAM24
flush, serial mouse and netBackgroundTick (which also pumps UART/HDMI
diagnostics). No persistence or storage behavior is changed; the measured
103 ms background stall still needs attribution. Measurements are absent
from the normal build. This is further instrumentation, not a claimed fix
for the background stall or the synchronous NXJMP dump.

Both PICO_DV builds and whitespace checks pass. Main SRAM normal/trace is
163,008 / 181,088 bytes (+64 / +32 over boot10); the Next overlay remains
34,128 / 34,808 bytes. Artifacts:
`debug/DVp2-next-boot11[-trace]-1.0.9.{elf,uf2}`.

## 2026-10-10: boot10 hardware result

Trace log `logs/devttyACM0_2026_10_10.19.31.21.989.txt` confirms another
menu improvement: stable TBBLUE CPU time is 17.2 ms/frame versus boot9's
18.1–18.2. Two stable intervals now show 50 FPS, neg=0/50 and minimum IDL
+403 / +407 us, including ~1.8 ms of health trace in their worst frames.
The preceding menu interval still includes a 20.382 ms CPU frame with
negative IDL; this is not a claim that every boot phase meets the deadline.

The post-CPU stall is localized: 105,269 us after CPU includes 103,219 us
in background, 58 us storage, 278 us audio, 30 us keyboard, 13 us tail and
1,663 us health trace. Thus storageTick and keyboard are not the cause of
this recorded stall. Background still groups frameEnd, ZiFi, RTC/NVRAM
flushes, overlay materialization, serial mouse and networking; finer
attribution is needed before changing persistence or I/O behavior.

The 2.45 s reset stall still samples dbg_uart_put_sync (487/512 samples).
The later mixed-clock loading interval improves from boot9's 41 to 43 FPS,
CPU 20.0 -> 18.3 ms/frame, worst CPU 28,560 -> 27,347 us (trace=0). These
are comparable intervals but not isolated 28 MHz benchmark phases; 27/43
frames still have negative IDL. Guest samples still place 18% in the
#0760/#0780 divMMC copy loop. This log ends during loading, with no settled
NextZXOS interval or new no-trace FPS measurement.

## 2026-10-10: boot10 Next executor and post-CPU attribution

Compile only `exec_nocheck_next` with O2, matching the existing TS fast
executor. Compared with boot9's size-optimized ARM code, the ordinary loop
keeps the line-limit, frame-limit, RAM-slot and opcode-table addresses in
registers, reuses the loaded cycle count at the loop back edge and removes
an unconditional branch. Guest operations and guards are unchanged. The
function grows from 228 to 252 bytes; there is no measured FPS claim yet.
The production-loop differential test (`tools/next_exec_test.py`, UBSan)
passes all 93,630 prefix/Q/EI, slice, MMU and gate-transition cases.

The ~105 ms post-CPU stall is still unattributed. `sdcard_alive` already
skips its probe after recent successful I/O; removing the storage watcher
without evidence would be unjustified. Trace now saves a stage breakdown
with the same worst frame: background (frameEnd, persistent state flushes,
overlays, networking), storage (including notifications), audio mixing,
keyboard, and tail (stats/remaining housekeeping). Health trace time stays
separate on the preceding line. Stage sums omit small gaps/other diagnostic
hooks and should not be treated as exact total accounting. These additions
are absent from normal builds; no measured delay is subtracted from IDL.

Both PICO_DV builds and diff whitespace checks pass. Main SRAM is
162,944 / 181,056 bytes (normal / trace); the normal total is unchanged
after alignment, trace +32 bytes over boot9. `.nxovl` is unchanged at
34,128 / 34,808 bytes. Artifacts:
`debug/DVp2-next-boot10[-trace]-1.0.9.{elf,uf2}`. Compare normal firmware for
performance and use trace's `NEXT worst post` to locate the remaining stall.

## 2026-10-10: boot9 hardware result

The owner sees further visual improvement but remaining FPS/IDL dips.
`logs/devttyACM0_2026_10_10.19.25.43.951.txt` is the trace run; the adjacent
19:24:48 normal-build log again contains startup only, not frame measurements.
Stable TBBLUE CPU time falls from boot8's 19.2–19.3 to 18.1–18.2 ms/frame.
FE reads fall from about 1.99 ms/frame to 0.797–0.805 ms/frame at about
2800 reads/frame. Menu intervals show 49–50 FPS and one negative-IDL frame
per interval. Two ordinary worst frames are now explicitly attributable to
the health trace: 18,276 us CPU + 59 us pre + 2,196 us post, including
1,786 us trace (IDL -539 us), and 18,271 + 61 + 2,183, including 1,780 us
trace (IDL -523 us). Subtracting just the measured dump cost leaves about
1.25 ms spare in those frames, but is not a no-trace hardware measurement.

Other stalls remain independent: 105,247 us after CPU with only 1,641 us
health trace; the synchronous NXJMP reset dump takes about 2.45 s with
509/512 host samples in dbg_uart_put_sync; genuine loading worst frames
take 28,560 and 27,254 us in CPU::loop with trace=0. Loading guest samples
include the #0760/#0780 divMMC byte-copy loop (10% + 9%). The final interval
reaches 50 FPS but still includes transition frames; this log ends before
a separate settled NextZXOS interval. Next targets: the ~104 ms non-trace
post-CPU stall and the divMMC/loading CPU paths, not another keyboard tweak.

## 2026-10-10: boot8 measurement and boot9 keyboard input

`logs/devttyACM0_2026_10_10.19.13.36.139.txt` confirms the owner's visual
improvement with boot8: the stable TBBLUE menu takes 19.2–19.3 ms CPU/frame,
49–50 FPS, typically 1 negative-IDL frame per 50 (one interval has 5/49).
Boot7 took 20.3–20.4 ms and usually 48 FPS. The worst ordinary menu frames
also spend about 2.2 ms after CPU::loop; that is not yet attributed to a
specific stage. The reset's 2.45 s stall samples `dbg_uart_put_sync` in
508/512 host samples, consistent with the synchronous NXJMP dump. Separate
loading frames still take 27–28 ms; the settled NextZXOS menu is 50 FPS with
positive IDL. Mixed-clock loading intervals do not establish a pure 28 MHz
average. The no-trace log has startup only, so no no-trace FPS claim yet.

Boot9 adds a short path for Next #xxFE reads with an empty, nonplaying tape.
The menu makes about 2800 FE reads/frame, previously about 2 ms total. The
path skips unrelated port decodes and TapePortRead (which has no effects in
this state), and visits only selected keyboard rows. The original early I/O
cycle, breakpoint and profiling hooks remain; the helper applies the same
three late cycles before reading rows and the Issue 2 MIC/EAR feedback.
Any inserted tape retains the original path, including autostart. Other
machines and other port addresses retain their decodes.

Trace firmware now reports `after ...us (trace ...us)` on the worst-frame
line: trace is the time spent in that frame's `next_trace_dump`, included in
after, not subtracted from FPS/IDL. This measures the health dump only, not
all trace overhead or the NXJMP dump inside CPU::loop. The synchronous
snapshot diagnostics are unchanged.

`python3 tools/next_keyboard_test.py` passes with UBSan: 34,603,008 keyboard,
EAR, issue-2, contention and register-change-during-Draw cases, plus 2,097,152
address/machine/tape guard cases. Both PICO_DV firmware builds pass.
Main SRAM is 162,944 / 181,024 bytes (normal / trace), +160 / +288 over boot8.
The Next overlay is unchanged at 34,128 / 34,808 bytes of 35,840.
Artifacts: `debug/DVp2-next-boot9[-trace]-1.0.9.{elf,uf2}`. Hardware performance
is still to be measured, starting with the normal build; this change targets
the TBBLUE keyboard loop, not all remaining loading bottlenecks.

## 2026-10-10: boot7 measurement and boot8 prefixes

The owner still sees FPS/IDL dips with boot7. The 19:10:23 trace log does
show a modest TBBLUE menu improvement: 21.8 ms/frame in boot6 becomes
20.3–20.4 ms/frame, ordinary intervals 45 -> 48 FPS (one 46 FPS interval).
That still exceeds the 20 ms frame budget. The settled NextZXOS menu reaches
50 FPS, CPU 0.6 ms/frame, positive IDL; the boot/reset transition includes
the synchronous NXJMP trace dump and mixed clock rates.

Disassembly of TBBLUE.FW #B980–#B9C0 shows repeated indexed loads/stores.
Boot8 adds an inline Next M1 reader for CB/DD/ED/FD second bytes. It retains
the divMMC mapping path and delegates the whole access at raster boundaries,
when the fast-memory gate is off, or under DEBUG. The DD/FD register/memory
load and ALU handlers also inline their five internal cycles, preserving the
existing deferred raster tick. No instruction is skipped and no key-wait
loop is recognized or fast-forwarded.

`tools/next_prefix_test.cpp` passes 1,310,720 cases in each DEBUG mode,
including every address, enabled/disabled gates, exact raster boundaries and
mapping-order sentinels. The extended `tools/next_memory_test.cpp` verifies
internal-cycle fallback and deferred ticking as well as existing operand
reads. Both run with UBSan. Normal and trace firmware builds pass; main SRAM
grows by 448 bytes relative to boot7, while the overlay size is unchanged.
Artifacts: `debug/DVp2-next-boot8[-trace]-1.0.9.{elf,uf2}`. The later boot8
trace measurement is recorded above; no-trace FPS remains unmeasured.

## 2026-10-10: boot7 opcode dispatch

The owner saw no useful visual improvement in boot6 at 28 MHz (14 MHz was
fine). The 18:58:59 trace log still has the TBBLUE menu at 21.8–21.9 ms/frame
and 43–45 FPS. Its post-reset intervals mix phases and clock rates, so they
do not establish a clean loading speedup from the SD-poll batch.

Boot7 extracts the unchecked Next executor from the multi-machine function.
The direct fetch skips both divMMC hooks outside their existing five trap
pages; on those pages a separate RAM function preserves pre-fetch mapping,
the byte read, then deferred mapping. The same fetch serves prefixed opcodes
below #4000. The page filter is shared with the original hooks, including
their non-trap-address side effects within each page. PC breakpoints retain
the generic executor; the fast loop still exits when its memory gate drops.

Validation: `python3 tools/next_opcode_test.py` compares 8,518,608 fetches and
mapping states with the boot6 hooks, including every address and sequential
mapping transitions. `python3 tools/next_exec_test.py` checks 93,630 executor
control-flow cases with synthetic handlers (prefixes, Q/EI, slice termination,
raster MMU changes, fast-path revocation). Both use UBSan. These are targeted
regressions, not a full Z80 instruction suite or a hardware speed measurement.

Both PICO_DV builds pass. The normal Next loop is 228 bytes; main SRAM falls
by 384 bytes without trace and 128 bytes with trace relative to boot6. The
rare fetch handler stays in main SRAM to avoid growing the Next overlay:
`.nxovl` remains 34,128 / 34,808 bytes (normal / trace), window 35,840 bytes.
Artifacts: `debug/DVp2-next-boot7[-trace]-1.0.9.{elf,uf2}`. Compare boot7
against boot6 without trace at 28 MHz; hardware FPS is still unmeasured.

## 2026-10-10: boot6 SD-response polling

The 18:45 and 18:48 dumps stopped in NextZXOS ROM 2's `#1928` SD
response loop (`IN A,(#EB); CP #FF; RET NZ; DJNZ #1928`). The latter
stack identifies CMD17's data-token wait. This is not the LDIR at a similar
address in the separately mapped enNxtmmc ROM.

`Z80::nextSdPollBatch` fuses up to 32 taken iterations while preserving every
port read. It verifies the code bytes and ROM/mapping, requires the unchecked
Next fast-read path with IFF1 clear, and declines PC/port-read/memory-read
breakpoints and RZX. B's terminal iteration and non-FF responses return to the
interpreter. Batches stop before raster boundaries and respect the CPU slice
limit, including scheduled DMA events. The existing early `#EB` path's 1-T I/O
charge is deliberately preserved; if that path is corrected to 4 T, update
the batch's IN bound and its regression model too.

`python3 tools/next_sd_poll_test.py` compares the production helper with an
instruction-stepping model: 1,689,600 cases covering all B values, early token
arrival, slice/raster boundaries, registers, flags, R, WZ and port-read times,
plus pattern and runtime guards (UBSan enabled). This does not model hardware
SD latency or prove a wall-clock speedup.

Hardware comparison: use `debug/DVp2-next-boot6-1.0.9.uf2` against boot5,
both without NEXT_TRACE. Measure the TBBLUE menu and NextZXOS load separately;
this optimization targets the latter's SD polling, not the former's key loop.
Hardware FPS for boot6 is still unmeasured. The AUTO Next overlay window grows
by 1 KiB to leave room for the helper and trace build.

The latest Claude session measured the TBBLUE menu at about 744 ns per Z80
instruction after inlining tsFastTick and the DivMMC opcode hooks (771 ns
before). DD/FD-heavy NextZXOS code still cannot sustain 14 MHz. The PC breakpoint
filter did not measurably improve performance; the 64-byte profile bucket also
contained opcode-fetch code. Do not attribute that bucket solely to breakpoints.

## Current change: inline operand reads in shared Z80 handlers

`src/Z80_JLS/next_memory.h` skips the CPU.cpp accessor call for direct Next reads
that finish before the next scanline boundary. It is used in the shared LD/ALU,
DD/FD/CB, INC/DEC, conditional branch and POP handlers. It adds **1,472 bytes of
static SRAM** in the current PICO_DV PERF_HIST build (RAM region 210,720 to
212,192 bytes). Inlining every operand read instead cost 5,696 bytes, so that
version was discarded.

The enable flag follows the existing Next fast-read gates and is cleared on
video reset. Memory breakpoints, NeoGS DMA overlays, fallback ROMs and other
machines retain the existing accessor. Slot pointers are looked up on each read,
including DivMMC/MMU mapping changes. The two halves of a word are read from
their own 8K slots, with uint16_t wrap at FFFF. At or past a scanline boundary,
the entire read delegates before modifying T-states, preserving copper/MMU
ordering. Opcode fetches and writes retain their existing paths.

Host regression, including ASan/UBSan, covers every address with the fast path
both enabled and disabled, discontiguous pages, wrap and MMU changes at a line
tick. ARM disassembly confirms the hot branch performs the load inline.
**The first hardware run is not a valid isolated speed comparison:** rendering
work differed substantially; see the follow-up below. Compatibility on other
machines is still untested.

## Hardware comparison

Both diagnostic ELFs use the same core flags (-Os) and profiler configuration:

- `debug/DVp2-next-inline-before-1.0.7.elf`: state at the start of this follow-up,
  including Claude's uncommitted fixes.
- `debug/DVp2-next-inline-1.0.7.elf`: the targeted inline readers above.

For each, leave the TBBLUE menu idle for five seconds, then boot NextZXOS.
For the FAT/save workload, issue `RUN AT 2` before `SAVE "t1.bas"` (use another
unused filename on the second run). Compare `NEXT cpu`, `NEXT z80`, CPU clock
and the opcode mix in equivalent phases. Check the file browser as well: an
earlier profiler using 8 KB caused an allocation panic; the current profiler
uses 2 KB. Before retaining this optimization, smoke-test TS-Conf and Pentagon
and confirm a useful measured gain. Do not infer a speedup from instruction
counts or the host tests alone.


## BIOS redraw follow-up

The user reported that the BIOS felt worse in the inline build. Compared logs:
`logs/devttyACM0_2026_09_25.21.37.25.904.txt` (before) and
`logs/devttyACM0_2026_09_25.21.56.42.073.txt` (inline).
Both report a 378 MHz host clock. In the stable TBBLUE menu, the instruction
mix and ~25,595 instructions/frame match:

| Measurement | Before | Inline run |
| --- | ---: | ---: |
| Emulated frames/s | 50 | 50 |
| CPU loop time, ms/frame | 19.0 | 16.8 |
| Estimated render time, ms/frame | 1.5 | 0.0 |
| Loop time/instruction, ns | 744 | 659 |

Do not claim an 11% core speedup: most of the difference is missing render work.
The original `NEXTVID::EndFrame` drew only one in 64 frames in Max speed mode.
At 50 emulated frames/s this gives only 0.78 screen updates/s, matching a BIOS
that looks stuck despite a healthy emulation FPS. The old log does not contain
the Max speed flag, so that mode's actual activation is inferred, not proven.
The later 28 MHz FAT phase runs at ~21 emulated frames/s; it cannot be compared
to the previous 3.5 MHz SAVE phase as an equal-clock benchmark.

The new `NextFramePacing` replaces the 1-in-64 rule with a 20 ms wall-clock
interval. Normal mode still draws every frame. In Max speed mode, a ~50 Hz
emulation draws at least every other frame; when frames take >=20 ms it draws
every frame. Copper and CPU timing are unchanged. Host tests cover fast/slow
cadences, reset, mode switching and 32-bit timer wrap. The NEXT cpu diagnostic
now includes `draw` (frames with composition enabled in the reporting interval)
and `max` (current Max speed flag).

Retest BIOS with `debug/DVp2-next-cadence-1.0.7.elf`. This build retains the
inline readers; the only functional follow-up is the redraw scheduling fix.
Hardware confirmation of the scheduling fix is pending.


## Cadence hardware log and cold-boot clock

`logs/devttyACM0_2026_09_25.22.06.42.658.txt` confirms `max=0` and
`fps=draw=50` in the BIOS, with normal renderer cost (1.5–1.6 ms/frame).
The user still reports image changes in strips: **that visual issue remains
unresolved** and cannot be attributed to frame skipping in this run.
With rendering enabled, the stable menu measures 18.3 ms/frame and 718 ns/instr
versus the earlier 19.0 ms and 744 ns (~3.5% lower time/instruction).

The same log exposed a separate cold-boot bug: `boot=1 spd=3/0`, 28 MHz,
93.1 ms for the first frame. Setup restored generic `Config::turbo` but reset
NextReg 07 to zero without resetting the live multiplier. Warm reset already
sets it to zero. Setup now also sets the live multiplier to zero before
CPU::reset/VIDEO::Reset derive their timing, preserving the saved generic turbo
preference for other machines. Hardware confirmation is pending.

## Session 2026-09-26: fetch loop, audio timer, NEXTVID on core1

Measured on DVp2 at 378 MHz, 14 MHz guest, plain (non-PERF) builds, ms of
core0 per frame (`NEXT cpu`):

| Scene | start | Next fetch loop | + audio timer | + NEXTVID on core1 |
| --- | ---: | ---: | ---: | ---: |
| TBBLUE menu | 22.1 | 17.1 | 16.6 | 15.4 |
| NextZXOS boot | 28.6 | 22.7 | 22.6 | 21.7 |
| Text viewer (Timex hi-res) | 25.9 | 19.9 | 19.3 | 17.4 |

- **Next fetch loop** (`Z80::exec_nocheck`, under `g_next_fastrd`): a loop of
  its own without the TS-Conf / Timex / overlay / beam-video tests; the shared
  fetch sequence was ~47% of core0 in the PC profile. Re-tests the flag per
  instruction, so the shared loop takes over when it drops mid-slice.
- **divMMC automap flags** merged into one byte (`DivMMC::pend_after`) and a
  trap-page filter in `preOpcFetch`.
- **Audio sample clock** (`pwm_audio.cpp`): own hardware alarm with a RAM ISR
  instead of the SDK repeating_timer, whose alarm_pool_irq_handler ran from
  flash 31250 times a second (~3% of core0). Every machine and driver.
- **NEXTVID on core1**: core0 runs the copper, snapshots the 48 bytes of line
  inputs (`NextLineState`) and posts `{row, state}`; core1's render_core loop
  draws (`NEXTVID::c1Pump`). Palette, sprite, Timex LUT/pair-map writers and
  every `VIDEO::tsRenderDrain()` site drain first; guest RAM is read live (the
  TS-Conf rule). ~4.6 KB heap, +480 B static SRAM (ScanlineWork is `noinline`:
  at -O3 it was inlined four times, +2.9 KB). Falls back to core0 when the
  heap block is missing or the queue sticks (`[NXC1]` log line).
- **Tried and dropped**: inlining ~44 hot handlers into a switch in the Next
  loop — within noise (viewer −1.5%, menu +1%) for +2.2 KB SRAM. The call
  itself is cheap; the cost is the handlers' global-register traffic, which
  only a full core rewrite would change.
