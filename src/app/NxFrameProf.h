// Per-frame host PC profiler for core0 (NEXT_TRACE builds) — see NxFrameProf.cpp.
#pragma once
#include <stdint.h>
#if NEXT_TRACE
namespace NxFrameProf {
    void init();                  // core0, once (heap 4 KB + one hardware alarm)
    void frameBegin();
    void frameEnd();              // stop sampling
    void keep();                  // this frame is the worst so far: keep its samples
}
extern uint32_t* g_nxprof_worst;
extern volatile uint32_t g_nxprof_worst_n;
#endif
