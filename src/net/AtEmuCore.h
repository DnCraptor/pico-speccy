#pragma once

// ESP8266 AT-command emulator — the protocol half.
//
// The guest-visible ZiFi NIC is a raw serial line to an ESP-01, and every guest
// network program (MRF, NedoOS esp-com and its apps, BridgeZX...) speaks ESP-AT
// over it itself. On the boards whose network is the on-chip CYW43 + our own lwIP
// there is no ESP: this class plays its part. It takes the guest's TX byte stream,
// answers in the ESP8266 NONOS AT 1.7 dialect, and runs the sockets through an
// AtTransport — lwIP on the device (AtEmu.cpp), a fake in tools/atemu_test.cpp.
//
// Depends on nothing from the firmware or the SDK, which is the whole reason the
// protocol can be tested on a host. Keep it that way.
//
// Output is PULLED (produce()), so the caller decides how fast the guest gets
// bytes: the ZiFi glue paces it at the configured NIC baud, the rate the guest
// drivers were tested against on a real ESP.

#include <stdint.h>
#include <stddef.h>

struct AtTransport {
    // Station (the host's own WiFi link — the guest never drives it).
    virtual bool        linkUp() = 0;                          // associated + has an IP
    virtual bool        ip(char* out, size_t cap) = 0;         // dotted station IP
    virtual bool        mac(char* out, size_t cap) = 0;        // "aa:bb:cc:dd:ee:ff"
    virtual const char* ssid() = 0;
    // Local time for AT+CIPSNTPTIME? (false = unknown).
    virtual bool        now(int& y, int& mo, int& d, int& h, int& mi, int& s) = 0;
    virtual uint32_t    ms() = 0;                              // monotonic milliseconds

    // Sockets, id 0..MAX_LINKS-1. All non-blocking.
    // open() starts DNS + connect; openState() then reports 0 = in progress,
    // 1 = connected, -1 = failed. Only TCP for now.
    virtual bool open(int id, const char* host, uint16_t port) = 0;
    virtual int  openState(int id) = 0;
    virtual int  send(int id, const uint8_t* p, int n) = 0;    // bytes accepted (may be 0), -1 = dead
    virtual int  avail(int id) = 0;                            // received, not yet taken
    virtual int  recv(int id, uint8_t* p, int n) = 0;          // take up to n
    virtual bool closed(int id) = 0;                           // peer FIN / error seen
    virtual void close(int id) = 0;                            // drop the link (no callback)
    virtual bool remote(int id, char* ip, size_t cap, uint16_t& port) = 0;

    virtual void log(const char* line) { (void)line; }
};

class AtEmuCore {
public:
    static constexpr int MAX_LINKS     = 5;
    static constexpr int LINE_MAX      = 256;  // longest command line we keep
    static constexpr int SEND_MAX      = 2048; // AT+CIPSEND limit (ESP8266)
    static constexpr int STAGE         = 256;  // CIPSEND payload staging
    static constexpr int TXT_SIZE      = 512;  // pending response text (power of 2)
    static constexpr int IPD_CHUNK     = 1460; // largest +IPD we emit
    static constexpr uint32_t CONNECT_TIMEOUT_MS = 10000;
    static constexpr uint32_t RST_DELAY_MS       = 300;

    explicit AtEmuCore(AtTransport& t) : T(t) { reset(false); }

    // Back to power-on state: echo on, mux 0, every link dropped silently.
    // `banner` queues the boot banner + "ready" (as an ESP does after a reset).
    void reset(bool banner);

    // One byte of the guest's TX stream. Call only while canAccept().
    void feed(uint8_t b);
    // False while a CIPSEND payload is waiting for the socket to take it: the
    // caller stops draining the guest's TX ring, which backs up into the guest.
    bool canAccept() const { return st != ST_SENDING; }

    // Advance async work (connects, pending sends, the RST banner, link events).
    void tick();

    // Next bytes for the guest, at most `max`. Returns the count (0 = nothing).
    int produce(uint8_t* out, int max);

    // Diagnostics.
    int  openLinks() const;
    bool mux() const { return muxMode; }

private:
    enum State : uint8_t { ST_CMD, ST_DATA, ST_SENDING };
    enum LinkSt : uint8_t { L_FREE, L_CONNECTING, L_OPEN };

    struct Link {
        LinkSt   st = L_FREE;
        uint32_t deadline = 0;
    };

    AtTransport& T;

    // Command line.
    char     line[LINE_MAX + 1];
    int      lineLen = 0;
    bool     lineOverflow = false;

    // Flags.
    bool     echo = true;
    bool     muxMode = false;
    bool     dinfo = false;
    bool     lastLink = false;    // WiFi state last reported to the guest

    // CIPSEND.
    State    st = ST_CMD;
    int      sendId = 0;
    int      sendLeft = 0;        // payload bytes the guest still owes
    int      sendTotal = 0;
    uint8_t  stage[STAGE];
    int      stageLen = 0;        // bytes staged, not yet taken by the socket
    int      stageOff = 0;

    // Async.
    bool     rstPending = false;
    uint32_t rstAt = 0;

    Link     L[MAX_LINKS];

    // +IPD streaming in progress: header, then exactly ipdLeft data bytes. Both go
    // out before any queued response text, which must never land inside the data.
    char     hdr[48];
    uint8_t  hdrLen = 0, hdrOff = 0;
    int      ipdId = -1;
    int      ipdLeft = 0;
    int      rr = 0;              // round-robin start for the next +IPD

    // Response text ring.
    uint8_t  txt[TXT_SIZE];
    uint16_t txtHead = 0, txtTail = 0;

    void put(const char* s);
    void putch(char c);
    void putf(const char* fmt, ...);
    int  txtFree() const { return TXT_SIZE - (uint16_t)(txtHead - txtTail); }
    void ok()    { put("\r\nOK\r\n"); }
    void error() { put("\r\nERROR\r\n"); }

    void execLine();
    void exec(const char* cmd);
    void cmdCipstart(const char* a);
    void cmdCipsend(const char* a);
    void cmdCipclose(const char* a);
    void cmdCipstatus();
    void cmdCifsr();
    void cmdCwjap(const char* a);
    void cmdSntpTime();
    void dataByte(uint8_t b);
    void flushStage();
    void startIpd();
    void linkEvents();
    void dropAll();
    bool anyOpen() const;
};
