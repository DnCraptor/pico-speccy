#pragma once
#include <stdint.h>

namespace NexLaunchCode {
constexpr uint16_t entry = 0x8000;
constexpr uint16_t directory = 0x8100;
constexpr uint16_t command = 0x8200;
constexpr uint16_t stack = 0xBFFE;
// APIs use IX from a normal application. C: is the system SD volume.
// The loader takes over on success. All returns stop at a polled HALT, with
// A = esxDOS error and D = stage (drive / directory / dot command).
constexpr uint8_t code[] = {
    0xF3,                         // DI
    0x16, 1,                      // LD D,1
    0x3E, 0x11,                   // LD A,C: (M_GETSETDRV encoded drive)
    0xCF, 0x89,                   // RST 8 / M_GETSETDRV
    0x38, 21,                     // JR C,stop
    0x16, 2,                      // LD D,2
    0x3E, '*',                    // LD A,'*'
    0xDD, 0x21, 0x00, 0x81,       // LD IX,directory
    0xCF, 0xA9,                   // RST 8 / F_CHDIR
    0x38, 9,                      // JR C,stop
    0x16, 3,                      // LD D,3
    0xDD, 0x21, 0x00, 0x82,       // LD IX,command
    0xCF, 0x8F,                   // RST 8 / M_EXECCMD
    0xF3,                         // DI
    0x76,                         // stop: HALT
    0x18, 0xFD,                   // JR stop
};
constexpr uint16_t stop = entry + 30;

// IDE_SNAPLOAD through M_P3DOS: HL' points to an FF-terminated filespec,
// DE = call ID, C = workspace bank 7. Success does not return; failure has
// carry CLEAR (unlike esxDOS). Any return is therefore handled by the host.
// NextZXOS_and_esxDOS_APIs.pdf: IDE_SNAPLOAD, M_P3DOS.
constexpr uint8_t snxCode[] = {
    0xF3,                         // DI
    0x16, 1,
    0x3E, 0x11,                   // select C:
    0xCF, 0x89,
    0x38, 25,                     // JR C,snxStop
    0x16, 2,
    0x3E, '*',
    0xDD, 0x21, 0x00, 0x81,       // IX = directory
    0xCF, 0xA9,
    0x38, 13,                     // JR C,snxStop
    0xD9,                         // EXX
    0x21, 0x00, 0x82,             // HL' = filespec
    0xD9,                         // EXX
    0x11, 0xFD, 0x00,             // DE = IDE_SNAPLOAD
    0x0E, 7,                      // C = RAM7
    0xCF, 0x94,                   // M_P3DOS
    0xF3,
    0x76,                         // snxStop: HALT
    0x18, 0xFD,
};
constexpr uint16_t snxStop = entry + 34;
// BASIC: LOAD the program through the NextBASIC interpreter, with IDE_BASIC
// ($01c0: HL = tokenised line ending $0d, ROM2/RAM5/RAM2/RAM0, SP between
// STKEND and RAMTOP; returns only when the line ends, ERR_NR = report-1).
// M_AUTOLOAD is not usable: it loads under 48K BASIC (ROM3), which cannot run
// NextBASIC. IDE_TOKENISER is not used either: it syntax-checks on the BASIC
// stack it derives from the OS's own frame, and from our context it derailed
// (RET chain through #FF58 / #E04F, SP in the attribute file, hw 2026-10-10,
// [NXJMP] ring: the crash is inside the tokeniser call, ROM1 #0A9C..#0AA8).
// So the HOST tokenises (LOAD/RUN are standard tokens) and writes the line
// into the E_LINE workspace; see NexLaunch::start.
// IDE_BASIC is CALLed directly with ROM2/RAM0 paged by hand (through M_P3DOS
// the LOAD would be an esxDOS call nested in the hook).
// Stops: D=1 drive, 2 chdir, 4 line returned (A=ERR_NR).
constexpr uint16_t basStack = 0xFF50;     // IDE_BASIC: under RAMTOP, bank 0
constexpr uint16_t basEntryStack = 0xBF00;// esxDOS calls (bank 2)
constexpr uint16_t basRamtop = 65367;
constexpr uint8_t basCode[] = {
    0xF3,                         // DI
    0x16, 1,
    0x3E, 0x11,                   // select C:
    0xCF, 0x89,
    0x38, 0x2B,                   // JR C,basStop
    0x16, 2,
    0x3E, '*',
    0xDD, 0x21, 0x00, 0x81,       // IX = directory
    0xCF, 0xA9,                   // F_CHDIR (the program loads its files from here)
    0x38, 0x1F,                   // JR C,basStop
    0x16, 4,
    0x31, basStack & 0xFF, basStack >> 8, // LD SP,basStack
    0xFD, 0xCB, 0x01, 0xFE,       // SET 7,(FLAGS): run mode
    0xAF,                         // XOR A: ROM2 (with BANK678 bit 2), RAM0
    0x32, 0x5C, 0x5B,             // LD (BANKM),A
    0x01, 0xFD, 0x7F,             // LD BC,#7FFD
    0xED, 0x79,                   // OUT (C),A
    0x2A, 0x59, 0x5C,             // LD HL,(E_LINE): the host-built line
    0xFB,                         // EI: BASIC runs with interrupts on
    0xCD, 0xC0, 0x01,             // CALL IDE_BASIC
    0xF3,
    0x3A, 0x3A, 0x5C,             // A = ERR_NR
    0x16, 4,
    0x76,                         // basStop: HALT
    0x18, 0xFD,
};
constexpr uint16_t basStop = entry + 52;
static_assert(sizeof(basCode) == 55 && basCode[52] == 0x76, "basStop must be the HALT");
static_assert(basStack < basRamtop && basStack >= 0xC000, "BASIC stack: bank 0, below RAMTOP");
}
