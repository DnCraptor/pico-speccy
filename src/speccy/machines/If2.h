// pico-speccy — ZX Interface 2 ROM cartridge on the 48K.
//
// A cartridge pulls ROMCS and replaces the whole 16 KB ROM at #0000-#3FFF; the
// machine starts it at reset. It is served exactly like an ALF cartridge — the file
// stays on SD, one 16 KB window on the heap (Alf::Cart, g_alfWindow), no flash write,
// no reboot — and the machine stays a stock 48K (A_48K, R_48K) with rom[0] pointed at
// the window. Guest writes into the window are dropped by MemESP::writebyte's existing
// g_alfWindow guard. FLASH-resident: nothing here is on a hot path.
#pragma once
#include <stdint.h>
#include <string>
#include "fatfs/ff.h"

namespace If2 {

// Largest image taken as an IF2 cartridge; anything bigger is an ALF cartridge.
constexpr uint32_t MAX_SIZE = 16u << 10;

// Plug `path` in and restart the machine on it (switches to the stock 48K first when
// needed). False with a message on screen when the file cannot be used.
bool insert(const std::string& path);

// Take the cartridge out. `restart` = put the 48K ROM back and reset the machine.
void eject(bool restart);

// A cartridge path is set (it may not be mounted yet, e.g. before boot binds it).
bool present();

// The cartridge is plugged into the running machine (48K, mounted).
bool live();

// Boot: mount Config::if2CartPath (after Alf::bindCart, before requestMachine). A
// missing SD file just leaves the slot empty.
void bindBoot();

// Config::requestMachine(A_48K, ...) tail: point rom[0] at the window when live.
void bindRom();

// CPU::reset: keep esxDOS off the bus while a cartridge is in (its automap fires on
// #0000/#0008/#0038/#0066, which are the cartridge's own vectors), and give it back
// once the cartridge is out.
void onReset();

// MachineSwitch::commit: leaving the stock 48K takes the cartridge out.
void onMachineSwitch(uint8_t arch, uint8_t romset);

// F1 Info for a .rom/.bin: what the image is and what Enter will do with it.
void describe(FIL* f, FSIZE_t size, std::string& info, int& lines);

}
