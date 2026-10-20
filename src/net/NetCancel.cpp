#include "NetCancel.h"

#include <pico/time.h>

namespace NetCancel {

static AskFn    s_ask  = nullptr;
static bool     s_req  = false;
static bool     s_hard = false;
static bool     s_in   = false;   // inside ask(): its dialog pumps the network too
static uint32_t s_last = 0;

static constexpr uint32_t POLL_US = 20000;

Scope::Scope(AskFn ask, bool hard) : prevAsk(s_ask), prevReq(s_req), prevHard(s_hard) {
    s_ask = ask;
    s_req = false;
    s_hard = hard;
    s_last = time_us_32();
}

Scope::~Scope() {
    // A nested operation the user cancelled cancels the one around it too.
    const bool inner = s_req;
    s_ask = prevAsk;
    s_hard = prevHard;
    s_req = prevAsk ? (prevReq || inner) : false;
}

bool poll() {
    if (s_req) return true;
    if (!s_ask || s_in) return false;
    const uint32_t now = time_us_32();
    if (now - s_last < POLL_US) return false;
    s_last = now;
    s_in = true;
    if (s_ask()) s_req = true;
    s_in = false;
    s_last = time_us_32();          // the dialog's own time does not count
    return s_req;
}

bool requested() { return s_req; }
bool abortIo()   { return s_req && s_hard; }

} // namespace NetCancel
