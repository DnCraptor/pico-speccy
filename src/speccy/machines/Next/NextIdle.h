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
// Hook: Z80::check_trdos (every taken JP / CALL / RET, Next branch): two
// compares on the hot path; the pattern check runs only for a target taken
// twice in a row and is cached per address.
#include <cstdint>

namespace NextIdle {
    extern uint16_t tgt;    // target of the last taken jump
    extern uint16_t rej;    // last target that is not an idle loop
    void jump();            // flash: tgt taken twice in a row, inside exec_nocheck
#if NEXT_TRACE
    extern volatile uint32_t g_nx_idle_n, g_nx_idle_t;   // passes skipped, T skipped
#endif
}
