#pragma once

// Esc-to-cancel for a user-visible network operation (catalog listing, download,
// upload, connect). The UI arms a Scope with an `ask` function that reads the
// keyboard and, on Esc/F1, asks the user to confirm; the network layer calls
// poll() from its wait loops (socket pumps, progress callbacks) and tests
// requested() where it can give up cleanly (TLS handshake/read, HTTP header and
// body loops, the download resume loop, socket recv waits).
//
// The verdict is LATCHED for the whole scope: once confirmed, every later poll
// answers true, so a retry/resume layer (HttpsGet::getToFileFn, the chunked
// catalog reader) cannot start the operation again behind the user's back.
// Cleanup paths (close_notify, CIPCLOSE) do not test it and run normally.
//
// Outside a scope poll() is a single load and false: the boot SNTP join, the FTP
// server and every other network user are unaffected.

#include <stdint.h>

namespace NetCancel {

// Reads the keyboard; returns true when the user asked to cancel AND confirmed.
// Called rate-limited, never re-entered.
typedef bool (*AskFn)();

// `hard` also lets the socket layer give up its waits early (abortIo()): right
// for a connection that is thrown away on cancel (HTTPS catalog, a connect). An
// FTP control channel or an SSH stream must stay in step with its peer, so a
// soft scope cancels only where the protocol layer can stop cleanly (the
// progress callback) and lets every socket wait run its course.
class Scope {
public:
    explicit Scope(AskFn ask, bool hard = true);
    ~Scope();
    Scope(const Scope&) = delete;
    Scope& operator=(const Scope&) = delete;
private:
    AskFn prevAsk;
    bool  prevReq;
    bool  prevHard;
};

// Check the keyboard (at most every ~20 ms) and latch a confirmed cancel.
// Returns requested().
bool poll();

// The latched verdict of the current scope (false outside one).
bool requested();

// requested() in a hard scope: socket recv/connect waits may return early.
bool abortIo();

} // namespace NetCancel
