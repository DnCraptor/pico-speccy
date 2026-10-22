#pragma once
#include <stddef.h>
#include <stdint.h>
#include <stdlib.h>
struct Buffer {
    enum : uint32_t { ALLOC_AUTO = 0, NEED_POINTER = 1, PREFER_PSRAM = 2 };
    static void* palloc(size_t n, uint32_t = 0) { return malloc(n); }
    static void  pfree(void* p) { free(p); }
};
