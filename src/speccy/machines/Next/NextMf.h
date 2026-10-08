#pragma once
#include <stdint.h>

// Spectrum Next Multiface (zxnext multiface.vhd + its wiring in zxnext.vhd).
//
// The Next's Multiface is not a card: it is 16K of the FPGA's SRAM (RAMPAGE 5 —
// enNextMf.rom at 0x0000-0x1FFF, read-only, and 8K of MF RAM at 0x2000-0x3FFF)
// paged over 0x0000-0x3FFF by an NMI. NextZXOS uses it to LAUNCH every snapshot
// from the Browser: bank 2 #0809 sets NR 06 bit 3, #0AB6 writes NR 02 = 08
// ("generate multiface NMI"), and the MF ROM's 0x0066 handler restores the
// snapshot and RETNs into it. Without it the OUT falls through into the next
// routine and the machine crashes on a stack the snapshot has already
// overwritten (hw 2026-10-08, DIHalt.sna from the Browser: black screen).
//
// State, as multiface.vhd keeps it:
//   nmiActive  NMI master: set by the MF NMI, cleared by RETN or a write to the
//              enable/disable port (+3 mode: also a disable READ)
//   invisible  software lockout: cleared by the NMI, set by a disable write
//              (enable write in +3 mode); ignored in MF 48 mode
//   memOn      the overlay itself: on at the 0x0066 fetch while nmiActive, off
//              on a disable read or RETN, on again on an enable read if visible
// Ports (NR 0x0A bits 7:6, full low-byte decode, NR 0x83 bit 1):
//   00 = MF +3   enable 0x3F  disable 0xBF
//   01 = MF 128  enable 0xBF  disable 0x3F
//   1x = MF 1/48 enable 0x9F  disable 0x1F
// An enable read in 128/+3 mode while visible returns machine state (+3: the
// paging latches by A15:12, the border otherwise; 128: 7FFD bit 3).
//
// Memory priority (zxnext.vhd ~3030): the MF overlay wins over the divMMC,
// Layer 2 write-over, Alt-ROM and the MMU. It is plugged through the divMMC
// window fields (MemESP::page0_lo/hi + divmmc_mapped) so the hot accessors
// need no new test; DivMMC::applyMapping re-plugs it whenever it runs.
namespace NextMf {
    void reset();                       // hard/soft reset, or NR 0x83 bit 1 off
    // NR 0x02 write bit 3 (gen = true) or clear (gen = false).
    void nr02(bool gen);
    uint8_t nr02Bit();                  // read-back of NR 0x02 bit 3
    bool request();                     // fire an MF NMI (false = refused)
    void nmiTaken();                    // Z80::nmi just set PC = 0x0066
    void retn();                        // RETN executed
    bool active();                      // mf_is_active: overlay on or NMI master
    bool memOn();                       // the overlay is paged in
    void plug();                        // (re)apply the overlay to slots 0/1
    // Port access hook (NextPorts). Read: returns -1 unless the MF drives the
    // data bus. Write: side effects only.
    int  portRead(uint16_t address);
    void portWrite(uint16_t address);
}
