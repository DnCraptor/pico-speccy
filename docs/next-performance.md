# Spectrum Next performance follow-up (2026-09-25)

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
