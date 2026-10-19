// Native NextZXOS launch for NEX/BASIC programs and SNX snapshots.
#pragma once
#include <string>
namespace NexLaunch {
// 1: guest launch scheduled; 0: OS unavailable (direct loader may be used);
// -1: preparation failed, with a user-visible error.
// TAP: a tape image the program loads through NextZXOS's own tape loader
// (/nextzxos/tapload.bas — what the Browser runs for .tap; the program's other
// files must sit beside it, it runs with that folder as the current one).
enum class Format { NEX, SNX, BAS, TAP };
int start(const std::string& path, Format format = Format::NEX);
// Launch through NextZXOS from anywhere: on a running Next now (or once the OS
// is up), on another machine after rebooting into the Next — the request is
// kept in a file across that reboot. True if the launch was taken on.
bool startWhenReady(const std::string& path, Format format);
void poll();
void cancel();
}
