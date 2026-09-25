// PC-sampling profiler for core0 (PERF_HIST builds only).
//
// SysTick fires PCPROF_HZ times a second on core0; the handler takes the
// interrupted PC out of the exception frame (word 6 in both the basic and the
// FP-extended frame) and counts it in a small open-addressing table keyed by
// 64-byte code bucket. pcprof_dump() logs the hottest buckets once a second;
// resolve them with addr2line against the same ELF. Nothing else in the
// firmware uses SysTick (the SDK's time base is the TIMER block).
#include <stdint.h>
#include <string.h>
#include <stdio.h>
#include "hardware/structs/systick.h"
#include "hardware/exception.h"
#include "hardware/clocks.h"
#include "Debug.h"

#if PERF_TRACE && PERF_HIST

#define PCPROF_HZ    8000u
#define PCPROF_SLOTS 256u        // power of two (2 KB with the counts)
#define PCPROF_SHIFT 6           // 64-byte buckets

static uint32_t s_key[PCPROF_SLOTS];
static uint32_t s_cnt[PCPROF_SLOTS];
static volatile uint32_t s_total, s_lost;

extern "C" void __not_in_flash_func(pcprof_sample)(const uint32_t* frame) {
    const uint32_t k = (frame[6] >> PCPROF_SHIFT) + 1u;   // + 1: 0 means empty
    uint32_t h = (k * 2654435761u) >> 24;                  // 8 bits
    for (uint32_t i = 0; i < 16; i++, h = (h + 1) & (PCPROF_SLOTS - 1)) {
        if (s_key[h] == k) { s_cnt[h]++; s_total++; return; }
        if (s_key[h] == 0) { s_key[h] = k; s_cnt[h] = 1; s_total++; return; }
    }
    s_lost++;
}

extern "C" void __attribute__((naked)) __not_in_flash_func(pcprof_isr)(void) {
    __asm volatile(
        "tst lr, #4      \n"
        "ite eq          \n"
        "mrseq r0, msp   \n"
        "mrsne r0, psp   \n"
        "b pcprof_sample \n");
}

void pcprof_start() {
    exception_set_exclusive_handler(SYSTICK_EXCEPTION, pcprof_isr);
    const uint32_t hz = clock_get_hz(clk_sys);
    systick_hw->csr = 0;
    systick_hw->rvr = hz / PCPROF_HZ - 1;
    systick_hw->cvr = 0;
    systick_hw->csr = 0x7;   // CLKSOURCE = processor, TICKINT, ENABLE
    Debug::log("PROF: SysTick PC sampling at %u Hz (clk_sys %u)", PCPROF_HZ, (unsigned)hz);
}

void pcprof_dump() {
    // Read the table in place with the tick masked (no second copy: RAM is
    // scarce on the Next). Samples missed while it is masked are ~1 ms/s.
    // The 42 hottest buckets over three lines ("PROF", "PROF+", "PROF+"):
    // the handlers are spread thinly, 14 buckets covered only ~60% of the time.
    systick_hw->csr &= ~0x2u;
    const uint32_t total = s_total, lost = s_lost;
    for (int part = 0; total && part < 3; part++) {
        char line[240];
        int n = part ? snprintf(line, sizeof line, "PROF+") :
                       snprintf(line, sizeof line, "PROF %u smp:", (unsigned)total);
        for (int k = 0; k < 14 && n < (int)sizeof line - 20; k++) {
            uint32_t best = 0, bi = 0;
            for (uint32_t i = 0; i < PCPROF_SLOTS; i++) if (s_cnt[i] > best) { best = s_cnt[i]; bi = i; }
            if (!best) break;
            n += snprintf(line + n, sizeof line - n, " %08X:%u",
                          (unsigned)((s_key[bi] - 1u) << PCPROF_SHIFT),
                          (unsigned)(best * 1000u / total));   // per mille
            s_cnt[bi] = 0;
        }
        if (part == 2 && lost) snprintf(line + n, sizeof line - n, " lost=%u", (unsigned)lost);
        Debug::log("%s", line);
    }
    memset(s_key, 0, sizeof s_key); memset(s_cnt, 0, sizeof s_cnt);
    s_total = s_lost = 0;
    systick_hw->csr |= 0x2u;
}

#else
void pcprof_start() {}
void pcprof_dump() {}
#endif
