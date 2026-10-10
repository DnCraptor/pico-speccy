// Native NextZXOS launch for NEX/BASIC programs and SNX snapshots.
#pragma once
#include <string>
namespace NexLaunch {
// 1: guest launch scheduled; 0: OS unavailable (direct loader may be used);
// -1: preparation failed, with a user-visible error.
enum class Format { NEX, SNX, BAS };
int start(const std::string& path, Format format = Format::NEX);
void poll();
void cancel();
}
