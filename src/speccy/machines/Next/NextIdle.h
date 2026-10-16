#pragma once
// ZX Next: fast-forward of a memory-poll idle loop — the main loop of a game
// that waits for its interrupt handler to raise a flag instead of HALTing:
//
//     S:  [EI]  LD A,(nn)  OR A | AND A | CP n | AND n | BIT b,A  JP cc,S
//
// Pogie (60 Hz) spins there for 55% of the frame (80EB, hw 2026-10-10): each
// pass is real host time while the guest does nothing. Nothing but an
// interrupt (or a DMA / NMI, which end the running slice) can change (nn), so
// once the loop has run two passes of equal length the guest clock is advanced
// by whole passes to the end of the slice — the next interrupt event — with the
// lines rendered on the way, exactly as a HALT is (NEXTVID::idleAdvanceTo).
// The phase inside the pass is kept, so the interrupt lands where it would.
//
// A pass may take up to three jumps (NextBench, 2026-10-11, 100% of its
// desktop frame: S: LD DE,(nn) / LD HL,(nn) / CALL f / OR A / JP NZ,S with
// f: XOR A / SBC HL,DE / RET NZ / INC A / RET — targets f, S+7, S, f, ...):
// the hook keeps the last three targets and S is a candidate when it comes
// round again after one, two or three jumps.
//
// Hook: every taken JP / CALL / RET / JR (Z80::check_trdos and the Next opcode
// switch): note() is a few compares on the hot path; the pattern check runs
// only for a repeating target and is cached per address.
#include <cstdint>

extern bool z80_in_nocheck;    // Z80_JLS.cpp: inside exec_nocheck

namespace NextIdle {
    extern uint16_t tgt, tgt2, tgt3;   // the last three jump targets, newest first
    extern uint16_t rej;               // last target that is not an idle loop
    void jump();                       // flash: tgt came round again, inside exec_nocheck
    void jumpGate();                   // SRAM: the reject table in front of jump()

    void noteNew(uint16_t pc);         // .nxovl: a target other than the last one
    static inline __attribute__((always_inline)) void note(uint16_t pc) {
        if (pc == tgt) {
            if (z80_in_nocheck && pc != rej) jumpGate();
        } else noteNew(pc);
    }
#if NEXT_TRACE
    extern volatile uint32_t g_nx_idle_n, g_nx_idle_t;   // passes skipped, T skipped
#endif
}
