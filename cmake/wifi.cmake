# On-chip WiFi (MURM_W / MURM2_W / PICO_DV_W / PICO_PC_W) and the W boards' PIO block assignment.

# On-chip WiFi (MURM_W / MURM2_W / PICO_DV_W: Raspberry Pi Radio Module 2 = CYW43439).
# This replaces a hook that had been dormant since it was written: it keyed off
# PICO_CYW43_SUPPORTED, which no board header here defined, and it used the
# keyword-less target_link_libraries() signature that CMake refuses to mix with
# the PRIVATE form used everywhere else on this target — so it would have failed
# the configure the first time it ever fired.
#
# Phase 1 links the bus driver only (pico_cyw43_arch_none): that brings up the
# radio and its PIO/DMA claims for ~0 heap, with no lwIP and no ~30 KB working
# set, which is what makes the pin/gpio_base bring-up cheap to test. lwIP arrives
# with WifiNet/WifiSock (pico_cyw43_arch_lwip_poll + pico_lwip_dns/sntp).
if (PICOSPECCY_WIFI)
    target_compile_definitions(${PROJECT_NAME} PRIVATE PICOSPECCY_WIFI=1
        # The SDK's fixed divider assumes 150 MHz; WifiNet::init() computes one
        # for the real clk_sys (378/504 MHz) — see the gSPI note there.
        CYW43_PIO_CLOCK_DIV_DYNAMIC=1
        CYW43_HOST_NAME="pico-speccy")
    # lwIP in POLL mode (NO_SYS, src/drivers/board/lwipopts.h): every stack callback runs inside
    # cyw43_arch_poll() from core0 — WifiNet::poll() once per frame plus the waits in
    # WifiNet/WifiSock — so the emulator, not a background IRQ context, decides when
    # the network runs. All lwIP memory is heap/net-arena via Buffer::palloc (see
    # lwipopts.h); the static footprint is a few hundred bytes.
    target_link_libraries(${PROJECT_NAME} PRIVATE pico_cyw43_arch_lwip_poll)
else()
    target_compile_definitions(${PROJECT_NAME} PRIVATE PICOSPECCY_WIFI=0)
endif()
# Board identity for the sources. MURM2 already defines MURM2=1 in its pin arm;
# MURM1 defines nothing and is the `#else` fallback everywhere, so MURM_W needs
# its own macro for src/drivers/board/BoardPins.cpp to tell the two Murmulator-1 modules apart.
if (MURM_W)
    target_compile_definitions(${PROJECT_NAME} PRIVATE MURM_W=1)
endif()
if (MURM2_W)
    target_compile_definitions(${PROJECT_NAME} PRIVATE MURM2_W=1)
endif()
# A third carrier, same module: the Pimoroni Pico DV. Nothing in the sources reads
# this macro — the PICO_DV arm in cmake/board-pins.cmake carries the module's pin
# moves and BoardPins.cpp takes the plain PICO_DV branch — it only labels the image,
# for a future `#if` that does need to tell DVp2w from DVp2.
if (PICO_DV_W)
    target_compile_definitions(${PROJECT_NAME} PRIVATE PICO_DV_W=1)
endif()
# Fourth carrier: the Olimex PICO-PC. Same story — the PICO_PC arm carries the
# module's pin moves, this macro only labels the image.
if (PICO_PC_W)
    target_compile_definitions(${PROJECT_NAME} PRIVATE PICO_PC_W=1)
endif()

# PIO block assignment on the W boards. An RP2350 PIO reaches 32 CONSECUTIVE
# GPIOs from its gpio_base (0 or 16), and the radio sits above GPIO31 — so CYW43
# needs a block at base 16, which pio1 (keyboard on GP0-3) and pio2 (HDMI on
# GP6-19) cannot be. That leaves pio0, and whatever else has moved above GPIO31
# has to join it there:
#
#   MURM_W   — audio is on GPIO40/41/42, so I2S moves pio1 -> pio0.
#              pio0: CYW43 (5-7) + I2S (9, or 17 for CS4334)   = 14..24 / 32
#              pio1: PS/2 (7) + NESPAD (7)                     = 14 / 32
#   MURM2_W  — audio stays on GP9/10/11 (below GP23, so unmoved), but the pad's
#              data pair is on GPIO40/41, so NESPAD moves pio1 -> pio0.
#              pio0: CYW43 (5-7) + NESPAD (7)                  = 12..14 / 32
#              pio1: PS/2 (7) + I2S (9, or 17)                 = 16..24 / 32
#   PICO_DV_W — audio is on the GP26/27/28 header positions, i.e. GPIO40/41/42
#              here, so I2S moves pio1 -> pio0 as on MURM_W; the pad is not used
#              on this carrier at all. pio1 keeps the PS/2 keyboard and — the one
#              thing unique to a Pico DV base — the SD card (SDCARD_PIO=pio1).
#              pio0: CYW43 (5-7) + I2S (9, or 17 for CS4334)   = 14..24 / 32
#              pio1: PS/2 (7) + SD SPI (4)                     = 11 / 32
#   PICO_PC_W — nothing PIO moves: the audio block that goes to GPIO40/41/42 is
#              PWM / beeper / MIDI UART (this carrier has no I2S), the pad (GP5/9/
#              20/21) and keyboard stay on pio1 at base 0. pio0 (or pio2 under
#              VGA) holds the radio alone. With HSTX — the AUTO default here, as
#              on PICO_PC/MURM2_W — the display claims NO PIO at all (TMDS
#              expander; VGA on HSTX likewise), and RAW keeps only the address
#              converter on pio2, so auxPio's pick stays free in every case.
#
# pio2 keeps HDMI (18 / 32) on all three. cyw43_arch_init() must therefore run
# AFTER graphics_init() and the keyboard have fixed their blocks at base 0, or the
# SDK's pio_claim_free_sm_and_add_program_for_gpio_range() can take the wrong one.
# (Only ONE display driver is initialised — graphics_init() picks VGA or HDMI —
# so under VGA the radio and I2S move to pio2 instead; BoardPins::auxPio is that
# decision and every one of them reads it.)
if (MURM_W OR PICO_DV_W)
    target_compile_definitions(${PROJECT_NAME} PRIVATE I2S_PIO=pio0)
endif()
if (MURM2_W)
    target_compile_definitions(${PROJECT_NAME} PRIVATE NESPAD_PIO=pio0)
endif()
