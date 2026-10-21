// The Yamaha YRW801-M GM sample ROM of the ZXM-MoonSound card (2 MB, CRC32 2A9D8D43,
// the same image MAME ships as yrw801.rom). Embedded only in the 16 MB firmware
// (MOONSOUND=1, cmake/board.cmake): it does not fit the 4 MB image. MoonPcm reads it
// straight from flash through XIP; only the card's 1 MB SRAM goes to PSRAM.
#if MOONSOUND
    .section .rodata.yrw801, "a"
    .balign 4
    .global gb_rom_yrw801
    .type   gb_rom_yrw801, %object
gb_rom_yrw801:
    .incbin YRW801_ROM_PATH
    .size   gb_rom_yrw801, . - gb_rom_yrw801
#endif
