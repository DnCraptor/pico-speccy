// Spectrum Next Multiface — see NextMf.h. Flash code: every entry is a cold
// path (an NMI, a RETN, a port access to 0x3F/0xBF/0x1F/0x9F on the Next).
#include "NextMf.h"
#include "NextReg.h"
#include "NextInt.h"
#include "speccy/core/MemESP.h"
#include "speccy/video/Video.h"
#include "speccy/z80/z80.h"
#include "speccy/devices/storage/DivMMC.h"
#include "app/Debug.h"

void ts_fastrd_recalc();   // Video.cpp: the Next fast read level follows slots 0/1
#if NEXT_TRACE
void nxJumpArmAfter(int n);   // Z80_JLS.cpp: dump the [NXJMP] ring n transfers later
#endif

// Read by DivMMC::preOpcFetch (inline, RAM): the MF overlay mutes the divMMC
// entry points. Mirrors s_memOn.
bool g_nxmf_mem = false;

namespace {
bool s_nmiActive = false;   // multiface.vhd nmi_active  (mf_nmi_hold)
bool s_invisible = true;    // multiface.vhd invisible
bool s_memOn     = false;   // multiface.vhd mf_enable   (mf_mem_en)
bool s_pending   = false;   // MF NMI requested, not yet taken at 0x0066
bool s_nr02      = false;   // NR 0x02 bit 3 read-back (nr_02_generate_mf_nmi)
uint8_t s_logLeft = 16;     // first few transitions go to the log

inline uint8_t mode() { return (uint8_t)(NextReg::reg[0x0A] >> 6); }   // 00 +3, 01 128, 1x 48
inline bool modeP3()  { return mode() == 0; }
inline bool mode48()  { return mode() >= 2; }

void note(const char* what) {
    if (!s_logLeft) return;
    s_logLeft--;
    Debug::log("[NXMF] %s (mode %u, nmi=%d inv=%d mem=%d, pc=%04X)", what, mode(),
               (int)s_nmiActive, (int)s_invisible, (int)s_memOn, (unsigned)Z80::getRegPC());
}

void unplug() {
    // Hand slots 0/1 back to whatever owns them under the MF: the divMMC window
    // (which re-derives the MMU mapping itself when it is not paged) or the MMU.
    if (DivMMC::enabled) {
        DivMMC::applyMapping();
    } else {
        MemESP::divmmc_mapped = false;
        MemESP::applyMMU(0, MemESP::mmu[0]);
        MemESP::applyMMU(1, MemESP::mmu[1]);
        ts_fastrd_recalc();
    }
    NextReg::updateLayer2Window();
}

void setMem(bool on) {
    if (on == s_memOn) return;
    s_memOn = on;
    g_nxmf_mem = on;
    if (on) NextMf::plug(); else unplug();
}

// The enable port by mf_type; the disable port is the other one of the pair.
inline uint8_t enablePort()  { return mode48() ? 0x9F : (mode() == 1 ? 0xBF : 0x3F); }
inline uint8_t disablePort() { return mode48() ? 0x1F : (mode() == 1 ? 0x3F : 0xBF); }

// zxnext.vhd 4307-4321: what an enable read drives onto the bus.
uint8_t portData(uint16_t address) {
    if (!modeP3()) return (uint8_t)((NextReg::port7FFD & 0x08) << 4 | 0x7F);
    switch (address >> 12) {
        case 0x1: return NextReg::port1FFD & 0x0F;            // motor + paging bits
        case 0x7: return NextReg::port7FFD;
        case 0xD: return NextReg::portDFFD & 0x5F;
        case 0xE: return 0;                                   // EFF7 bits 3/2: not modelled
        default:  return VIDEO::borderColor & 0x07;
    }
}
} // namespace

void NextMf::plug() {
    if (!s_memOn || !MemESP::mfRomBase) return;
    MemESP::page0_lo = MemESP::mfRomBase;       // enNextMf.rom: read-only
    MemESP::divmmc_lo_rom = true;
    MemESP::page0_hi = MemESP::mfRamBase;       // MF RAM: writable
    MemESP::divmmc_lo_dirty = nullptr;
    MemESP::divmmc_hi_dirty = nullptr;
    MemESP::divmmc_mapped = true;
    MemESP::nextDivPlug();
    MemESP::ramContended[0] = MemESP::ramContended[1] = false;
    MemESP::wr_overlay_mask &= (uint8_t)~3;     // MF beats the Layer 2 write-over
    ts_fastrd_recalc();
}

void NextMf::reset() {
    const bool was = s_memOn;
    s_nmiActive = false;
    s_invisible = true;
    s_memOn = false;
    g_nxmf_mem = false;
    s_pending = false;
    s_nr02 = false;
    if (was) unplug();
}

void NextMf::nr02(bool gen) {
    if (!gen) { s_nr02 = false; return; }
    if (request()) s_nr02 = true;
}

uint8_t NextMf::nr02Bit() { return s_nr02 ? 0x08 : 0x00; }

// zxnext.vhd 2090-2112: nmi_assert_mf needs NR 06 bit 3, then it is taken only
// with no NMI in progress, port 0xE3 bit 7 (CONMEM) clear and no divMMC NMI
// hold; multiface.vhd button_pulse additionally needs nmi_active clear.
bool NextMf::request() {
    const char* why = nullptr;
    if (MemESP::nextConfigMode || MemESP::nextBootRomActive) why = "config mode";
    else if (!NextReg::portOn(NextReg::PE_MF)) why = "NR 83 bit 1 off";
    else if (!(NextReg::reg[0x06] & 0x08))     why = "NR 06 bit 3 off";
    else if (!MemESP::mfRomBase)                why = "no enNextMf.rom";
    else if (s_nmiActive || s_pending)          why = "MF NMI already active";
    else if (DivMMC::conmem)                    why = "port E3 bit 7 set";
    else if (DivMMC::next_nmi_hold && (DivMMC::next_nmi_btn || DivMMC::automap))
                                                why = "divMMC NMI in progress";
    if (why) {
        if (s_logLeft) { s_logLeft--; Debug::log("[NXMF] NMI refused: %s", why); }
        return false;
    }
    s_nmiActive = true;      // button_pulse
    s_invisible = false;
    s_pending = true;
    Z80::triggerNMI();
    // Take it NOW, not at the next frame: enNextMf's 0x0066 handler only runs
    // the snapshot launch when the interrupted PC is #0AB8/#0AB9 (the NOP right
    // after NextZXOS's OUT to NR 02) — anything later reads as the M1 button
    // and opens the Multiface debugger (hw 2026-10-08). wake() ends the
    // running slice after at most one more instruction; CPU::loop's Next
    // slice loop then accepts the NMI at that boundary.
    NextInt::wake();
    note("NMI requested");
    return true;
}

void NextMf::nmiTaken() {
    // fetch_66: the M1 at 0x0066 with nmi_active pages the overlay in, and it
    // is the overlay that supplies that very opcode.
    if (!s_pending) return;
    s_pending = false;
    setMem(true);
    note("paged in at 0066");
}

void NextMf::retn() {
    if (!s_nmiActive && !s_memOn) return;
    s_nmiActive = false;
    setMem(false);
    note("RETN: paged out");
#if NEXT_TRACE
    nxJumpArmAfter(0);   // re-arm the [NXJMP] ring for this launch
#endif
}

bool NextMf::active() { return s_memOn || s_nmiActive; }
bool NextMf::memOn()  { return s_memOn; }

int NextMf::portRead(uint16_t address) {
    if (!NextReg::portOn(NextReg::PE_MF)) return -1;
    const uint8_t lo = (uint8_t)address;
    if (lo == enablePort()) {
        const bool inv = s_invisible && !mode48();
        setMem(!inv);
        // mf_port_en: only a visible MF in 128/+3 mode drives the bus
        return (!inv && !mode48()) ? portData(address) : -1;
    }
    if (lo == disablePort()) {
        setMem(false);
        if (modeP3()) s_nmiActive = false;
        return -1;
    }
    return -1;
}

void NextMf::portWrite(uint16_t address) {
    if (!NextReg::portOn(NextReg::PE_MF)) return;
    const uint8_t lo = (uint8_t)address;
    const bool en = lo == enablePort(), dis = lo == disablePort();
    if (!en && !dis) return;
    s_nmiActive = false;
    if ((dis && !modeP3()) || (en && modeP3())) s_invisible = true;
}
