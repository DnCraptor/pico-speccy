#pragma once

// The guest ZiFi NIC on the on-chip-WiFi boards: an ESP8266 AT emulator
// (AtEmuCore) whose sockets are our lwIP. ZiFi::init() starts it when the NIC is
// on and Network -> Transport is "On-chip WiFi"; the guest's port windows
// (#xxEF 16550 / ZIFI-API, ZX-UNO #FC3B/#FD3B) and their FIFOs are unchanged —
// this replaces only the UART at the far end of them.
//
// Only built where there is a radio (PICOSPECCY_WIFI); every other board keeps
// its image, its RAM and its ZiFi code as they were.

#if PICOSPECCY_WIFI

namespace AtEmu {

bool start();      // allocate (~1.5 KB heap) and power the "ESP" up; false = no heap
void stop();       // drop every guest link, free everything
bool active();

// Once per emulated frame (ZiFi::tick): guest TX -> emulator, async work, then
// the guest's RX ring is refilled at the configured NIC baud (guest time).
void tick();

// Guest TX ring -> emulator, without the rest of tick(). Called from the port
// write path when the ring fills, so a long CIPSEND payload is not cut at 256 B.
void drainTx();

// Hardware Info: "mux 0, 1 link" etc. Empty when not running.
void status(char* out, int cap);

}  // namespace AtEmu

#endif  // PICOSPECCY_WIFI
