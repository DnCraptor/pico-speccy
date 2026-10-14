// pico-speccy — ZX Interface 2 ROM cartridge. See If2.h. FLASH-resident.
#include "If2.h"
#include "Alf.h"

#include "app/Config.h"
#include "app/Debug.h"
#include "app/ESPectrum.h"
#include "app/MachineSwitch.h"
#include "app/TryAlloc.h"
#include "app/messages.h"
#include "speccy/core/MemESP.h"
#include "speccy/devices/storage/DivMMC.h"
#include "ui/OSDMain.h"
#include <stdio.h>
#include <string.h>

namespace {
bool s_esxOff = false;   // esxDOS taken off the bus by a cartridge (given back at eject)

bool isStock48() { return Config::arch == A_48K && Config::romSet == R_48K; }

// Close the window only when it is ours: Alf::Cart is shared with ALF cartridges.
void unplug() {
    if (Alf::Cart::active() && Alf::Cart::path() == Config::if2CartPath)
        Alf::Cart::unmount();
    Config::if2CartPath = "";
}

uint32_t crc32(const uint8_t* p, uint32_t n, uint32_t c) {
    c = ~c;
    while (n--) {
        c ^= *p++;
        for (int k = 0; k < 8; k++) c = (c >> 1) ^ (0xEDB88320u & (0u - (c & 1)));
    }
    return ~c;
}
}

bool If2::present() { return !Config::if2CartPath.empty(); }

bool If2::live() {
    return isStock48() && present() && Alf::Cart::active() &&
           Alf::Cart::path() == Config::if2CartPath;
}

bool If2::insert(const std::string& path) {
    FIL* f = fopen2(path.c_str(), FA_READ);
    if (!f) { OSD::osdCenteredMsg(OSD_NOROMFILE_ERR, LEVEL_WARN, 2000); return false; }
    FSIZE_t size = f_size(f);
    fclose2(f);
    if (size == 0 || size > MAX_SIZE) {
        OSD::osdCenteredMsg(" Not an Interface 2 cartridge ", LEVEL_WARN, 2000);
        return false;
    }
    // Persist BEFORE the machine switch: MachineSwitch::commit() reboots on some
    // boards and never returns; the boot then mounts the cartridge itself.
    Config::if2CartPath = path;
    Config::save();
    if (!isStock48() && !MachineSwitch::commit(A_48K, R_48K)) {
        Config::if2CartPath = "";
        Config::save();
        return false;
    }
    // Mounted only now: leaving ALF inside commit() closes the shared window.
    if (!Alf::Cart::mount(path)) {
        Config::if2CartPath = "";
        Config::save();
        OSD::osdCenteredMsg(" Cartridge: cannot open or no memory ", LEVEL_WARN, 2000);
        return false;
    }
    Config::alfCartPath  = "";
    Config::alfCartBanks = 0;
    Config::requestMachine(A_48K, R_48K);    // -> bindRom()
    ESPectrum::reset();                      // -> onReset(); the cartridge starts at #0000
    Debug::log("[IF2] cartridge in: %s (%u B)", path.c_str(), (unsigned)size);
    return true;
}

void If2::eject(bool restart) {
    if (!present()) return;
    unplug();
    Config::save();
    if (Config::arch == A_48K) Config::requestMachine(A_48K, Config::romSet);   // ROM back
    if (restart) ESPectrum::reset();
    Debug::log("[IF2] cartridge out");
}

void If2::bindBoot() {
    if (!present()) return;
    if (!Config::alfCartPath.empty() || !Alf::Cart::mount(Config::if2CartPath)) {
        Debug::log("[IF2] cartridge %s not mounted, slot emptied", Config::if2CartPath.c_str());
        Config::if2CartPath = "";
    }
}

void If2::bindRom() {
    if (live()) MemESP::rom[0].assign_rom(g_alfWindow);
}

void If2::onReset() {
    if (live()) {
        if (!DivMMC::enabled) return;
        DivMMC::automap = DivMMC::conmem = DivMMC::mapram = false;
        DivMMC::applyMapping();
        DivMMC::enabled = false;
        s_esxOff = true;
        Debug::log("[IF2] esxDOS off while the cartridge is in");
    } else if (s_esxOff) {
        s_esxOff = false;
        DivMMC::enabled = (Config::esxdos != 0);
    }
}

void If2::onMachineSwitch(uint8_t arch, uint8_t romset) {
    if (present() && !(arch == A_48K && romset == R_48K)) {
        unplug();
        Debug::log("[IF2] cartridge out (machine switch)");
    }
}

void If2::describe(FIL* f, FSIZE_t size, std::string& info, int& lines) {
    auto line = [&](const char* s) { info += s; info += "\n"; lines++; };
    char buf[48];
    if (size > MAX_SIZE) {
        snprintf(buf, sizeof(buf), "ALF cartridge, %u x 16K banks",
                 (unsigned)((size + 0x3FFF) >> 14));
        line(buf);
        line("Enter: plug in, machine -> ALF");
        return;
    }
    if (size == 0) { line("Empty file"); return; }

    // Heap, not stack: F1 Info runs deep under the browser (FileInfo's own rule).
    ScopedHeap ch(512);
    if (!ch) { line("(no memory)"); return; }
    uint8_t* chunk = ch.as<uint8_t>();
    uint8_t head[8] = {0};
    uint8_t vec38 = 0xFF;
    uint32_t crc = 0, pos = 0;
    UINT br = 0;
    f_lseek(f, 0);
    while (pos < size && f_read(f, chunk, 512, &br) == FR_OK && br) {
        if (pos == 0) memcpy(head, chunk, br < 8 ? br : 8);
        if (pos <= 0x38 && pos + br > 0x38) vec38 = chunk[0x38 - pos];
        crc = crc32(chunk, br, crc);
        pos += br;
    }
    // Sinclair 48K BASIC and every ROM built on it open with DI / XOR A / LD DE,#FFFF.
    static const uint8_t kBasic[] = {0xF3, 0xAF, 0x11, 0xFF, 0xFF};
    const bool basic = size == MAX_SIZE && memcmp(head, kBasic, sizeof(kBasic)) == 0;
    line(basic ? "48K system ROM (BASIC)" : "ZX Interface 2 cartridge");
    snprintf(buf, sizeof(buf), "CRC32: %08lX", (unsigned long)crc);
    line(buf);
    if (!basic) {
        line(vec38 == 0xFF ? "IM 1 at #0038: none" : "IM 1 at #0038: own handler");
        line("Runs on: 48K with Interface 2");
    }
    line("Enter: plug in, restart as 48K");
}
