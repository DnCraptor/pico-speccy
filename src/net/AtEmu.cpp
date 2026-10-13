#include "AtEmu.h"

#if PICOSPECCY_WIFI

#include "AtEmuCore.h"
#include "WifiNet.h"
#include "speccy/devices/zifi/ZiFi.h"
#include "speccy/devices/storage/RTC.h"
#include "speccy/z80/CPU.h"
#include "app/ESPectrum.h"
#include "app/Debug.h"
#include "app/TryAlloc.h"
#include "ui/LEDIndicators.h"

#include "pico/cyw43_arch.h"
#include "lwip/tcp.h"
#include "lwip/dns.h"
#include "lwip/pbuf.h"
#include "lwip/ip_addr.h"
#include <new>
#include <string.h>
#include <stdio.h>

// All lwIP callbacks run inside cyw43_arch_poll() on core0 (poll-mode NO_SYS), the
// same core and context as tick() — so nothing here needs a lock. Same model as
// WifiSock, with its own links: the guest's sockets never share a slot with the
// host's FTP/HTTPS/SSH sessions.

namespace {

// Receive window of a guest link: 2 x MSS, half of the host sessions' TCP_WND.
// The guest is fed at the NIC baud (~23 KB/s at 230400), so a bigger window only
// parks more pbufs in RAM. tcp_connect() sets TCP_WND itself, so this is applied
// right after it: the SYN has already gone out with TCP_WND, but the server only
// sends after the handshake, and our ACK to its SYN-ACK carries this value.
constexpr u16_t GUEST_WND = 2 * TCP_MSS;
// All guest links together may hold this much received-but-unread data; above it
// a link's window is left closed (tcp_recved deferred) until the guest catches up.
constexpr int   GUEST_RX_BUDGET = 6144;
// The pace never banks more than this — after an idle stretch the guest gets a
// normal burst, not seconds of backlog at once.
constexpr uint32_t CREDIT_CAP = 2048;

enum : uint8_t { S_FREE, S_DNS, S_CONNECTING, S_OPEN, S_FAILED };

struct Link {
    tcp_pcb*  pcb = nullptr;
    pbuf*     rxq = nullptr;
    uint16_t  rx_off = 0;
    uint16_t  port = 0;
    uint16_t  ackDue = 0;      // consumed by the guest, tcp_recved not yet sent
    uint8_t   st = S_FREE;
    bool      fin = false;     // peer FIN or error
    uint8_t   dnsGen = 0;
    ip_addr_t addr;
};

struct LwipTransport : AtTransport {
    Link L[AtEmuCore::MAX_LINKS];

    bool linkUp() override { return WifiNet::isConnected(); }
    bool ip(char* out, size_t cap) override { return WifiNet::ipString(out, cap); }
    bool mac(char* out, size_t cap) override {
        uint8_t m[6] = {};
        cyw43_wifi_get_mac(&cyw43_state, CYW43_ITF_STA, m);
        snprintf(out, cap, "%02x:%02x:%02x:%02x:%02x:%02x", m[0], m[1], m[2], m[3], m[4], m[5]);
        return true;
    }
    const char* ssid() override { return WifiNet::ssid(); }
    bool now(int& y, int& mo, int& d, int& h, int& mi, int& s) override {
        return RTC::now(y, mo, d, h, mi, s);
    }
    uint32_t ms() override { return to_ms_since_boot(get_absolute_time()); }

    bool open(int id, const char* host, uint16_t port) override;
    int  openState(int id) override {
        switch (L[id].st) {
            case S_OPEN:   return 1;
            case S_FAILED: return -1;
            case S_FREE:   return -1;
            default:       return 0;
        }
    }
    int  send(int id, const uint8_t* p, int n) override;
    int  avail(int id) override {
        const Link& l = L[id];
        return l.rxq ? (int)(l.rxq->tot_len - l.rx_off) : 0;
    }
    int  recv(int id, uint8_t* p, int n) override;
    bool closed(int id) override { return L[id].fin || !L[id].pcb; }
    void close(int id) override;
    bool remote(int id, char* ip, size_t cap, uint16_t& port) override {
        if (!ipaddr_ntoa_r(&L[id].addr, ip, (int)cap)) ip[0] = 0;
        port = L[id].port;
        return true;
    }
#if ZIFI_TRACE
    void log(const char* line) override { Debug::log("ZiFi %s", line); }
#endif

    void connectNow(int id);
    void flushAcks();
};

struct Ctx {
    LwipTransport T;
    AtEmuCore     E;
    uint64_t      lastT = 0;      // CPU::global_tstates at the previous tick
    uint64_t      acc = 0;        // pace remainder (byte-T-states)
    uint32_t      credit = 0;     // bytes the guest may receive now
    Ctx() : E(T) {}
};

Ctx* s = nullptr;
uint8_t s_dnsGen = 0;

// ── lwIP callbacks ───────────────────────────────────────────────────────────

err_t cbRecv(void* arg, tcp_pcb*, pbuf* p, err_t err) {
    Link* l = (Link*)arg;
    if (!p) { l->fin = true; return ERR_OK; }
    if (err != ERR_OK) { pbuf_free(p); return err; }
    if (l->rxq) pbuf_cat(l->rxq, p); else l->rxq = p;
    return ERR_OK;                  // tcp_recved() only as the guest takes bytes
}

void cbErr(void* arg, err_t) {
    Link* l = (Link*)arg;
    l->pcb = nullptr;               // lwIP has freed it
    l->fin = true;
    if (l->st != S_OPEN) l->st = S_FAILED;
}

err_t cbConnected(void* arg, tcp_pcb*, err_t err) {
    Link* l = (Link*)arg;
    l->st = err == ERR_OK ? S_OPEN : S_FAILED;
    return ERR_OK;
}

void detach(Link& l) {
    if (!l.pcb) return;
    tcp_arg(l.pcb, nullptr);
    tcp_recv(l.pcb, nullptr);
    tcp_err(l.pcb, nullptr);
}

void dnsCb(const char*, const ip_addr_t* ip, void* arg) {
    if (!s) return;
    uintptr_t v = (uintptr_t)arg;
    int id = (int)(v & 0x0F);
    uint8_t gen = (uint8_t)(v >> 4);
    if (id >= AtEmuCore::MAX_LINKS) return;
    Link& l = s->T.L[id];
    if (l.st != S_DNS || l.dnsGen != gen) return;   // the guest gave up on it
    if (!ip) { l.st = S_FAILED; return; }
    l.addr = *ip;
    s->T.connectNow(id);
}

// ── transport ────────────────────────────────────────────────────────────────

void LwipTransport::connectNow(int id) {
    Link& l = L[id];
    tcp_pcb* pcb = tcp_new_ip_type(IPADDR_TYPE_V4);
    if (!pcb) { l.st = S_FAILED; return; }
    l.pcb = pcb;
    tcp_arg(pcb, &l);
    tcp_recv(pcb, cbRecv);
    tcp_err(pcb, cbErr);
    tcp_nagle_disable(pcb);
    l.st = S_CONNECTING;
    if (tcp_connect(pcb, &l.addr, l.port, cbConnected) != ERR_OK) {
        detach(l); tcp_abort(pcb); l.pcb = nullptr;
        l.st = S_FAILED;
        return;
    }
    pcb->rcv_wnd = pcb->rcv_ann_wnd = GUEST_WND;
}

bool LwipTransport::open(int id, const char* host, uint16_t port) {
    close(id);
    Link& l = L[id];
    l.port = port;
    if (ipaddr_aton(host, &l.addr)) { connectNow(id); return l.st != S_FAILED; }
    l.st = S_DNS;
    l.dnsGen = ++s_dnsGen & 0x0F;
    err_t e = dns_gethostbyname(host, &l.addr, dnsCb,
                                (void*)(uintptr_t)((l.dnsGen << 4) | id));
    if (e == ERR_OK) { connectNow(id); return l.st != S_FAILED; }
    if (e != ERR_INPROGRESS) { l.st = S_FREE; return false; }
    return true;
}

int LwipTransport::send(int id, const uint8_t* p, int n) {
    Link& l = L[id];
    if (!l.pcb || l.st != S_OPEN) return -1;
    u16_t room = tcp_sndbuf(l.pcb);
    if (room == 0 || tcp_sndqueuelen(l.pcb) >= TCP_SND_QUEUELEN - 1) return 0;
    if (n > room) n = room;
    err_t e = tcp_write(l.pcb, p, (u16_t)n, TCP_WRITE_FLAG_COPY);
    if (e == ERR_MEM) return 0;
    if (e != ERR_OK) return -1;
    tcp_output(l.pcb);
    LED::touchW(LED::NET);
    return n;
}

int LwipTransport::recv(int id, uint8_t* p, int n) {
    Link& l = L[id];
    if (!l.rxq) return 0;
    int avail = (int)(l.rxq->tot_len - l.rx_off);
    if (n > avail) n = avail;
    pbuf_copy_partial(l.rxq, p, (u16_t)n, l.rx_off);
    l.rx_off += (uint16_t)n;
    while (l.rxq && l.rx_off >= l.rxq->len) {
        l.rx_off -= l.rxq->len;
        pbuf* next = l.rxq->next;
        if (next) pbuf_ref(next);
        pbuf_free(l.rxq);
        l.rxq = next;
    }
    l.ackDue += (uint16_t)n;
    return n;
}

// Reopen windows for what the guest has taken — while everything the guest
// links hold together stays under the budget. With one link (mux 0) the
// window alone keeps it at ~2.9 KB, so this only ever bites in mux 1.
void LwipTransport::flushAcks() {
    int held = 0;
    for (int i = 0; i < AtEmuCore::MAX_LINKS; i++) held += avail(i);
    for (int i = 0; i < AtEmuCore::MAX_LINKS && held < GUEST_RX_BUDGET; i++) {
        Link& l = L[i];
        if (!l.ackDue) continue;
        if (l.pcb) tcp_recved(l.pcb, l.ackDue);
        held += l.ackDue;               // what this window may now bring in
        l.ackDue = 0;
    }
}

void LwipTransport::close(int id) {
    Link& l = L[id];
    if (l.pcb) {
        detach(l);
        if (tcp_close(l.pcb) != ERR_OK) tcp_abort(l.pcb);
    }
    if (l.rxq) pbuf_free(l.rxq);
    l = Link{};
}

uint32_t paceBaud() {
    uint32_t b = ZiFi::nicBaud();
    return b ? b : 115200;
}

}  // namespace

namespace AtEmu {

bool start() {
    if (s) return true;
    void* mem = tryMalloc(sizeof(Ctx));
    if (!mem) { Debug::log("ZiFi: AT emulator: no heap (%u B)", (unsigned)sizeof(Ctx)); return false; }
    s = new (mem) Ctx();
    s->lastT = CPU::global_tstates;
    Debug::log("ZiFi: AT emulator up (%u B, pace %u baud, radio %s)",
               (unsigned)sizeof(Ctx), (unsigned)paceBaud(),
               WifiNet::isConnected() ? "connected" : "not connected");
    return true;
}

void stop() {
    if (!s) return;
    s->E.reset(false);                 // closes every link
    s->~Ctx();
    free(s);
    s = nullptr;
    cyw43_arch_poll();                 // push the FINs out
    Debug::log("ZiFi: AT emulator stopped");
}

bool active() { return s != nullptr; }

void drainTx() {
    if (!s) return;
    int b;
    while (s->E.canAccept() && (b = ZiFi::txPopByte()) >= 0) s->E.feed((uint8_t)b);
}

void tick() {
    if (!s) return;
    drainTx();
    s->E.tick();

    // Pace: bytes = guest time x baud / 10, in the CPU's own clock (turbo frames
    // carry more T-states for the same wall time).
    const uint64_t now = CPU::global_tstates;
    const uint64_t dt = now - s->lastT;
    s->lastT = now;
    const uint64_t clock = 3500000ull << ESPectrum::multiplicator;
    s->acc += dt * (paceBaud() / 10);
    s->credit += (uint32_t)(s->acc / clock);
    s->acc %= clock;
    if (s->credit > CREDIT_CAP) s->credit = CREDIT_CAP;

    uint32_t want = s->credit;
    uint16_t room = ZiFi::rxRoom();
    if (want > room) want = room;
    uint8_t buf[128];
    bool got = false;
    while (want) {
        int n = s->E.produce(buf, want > sizeof buf ? (int)sizeof buf : (int)want);
        if (n <= 0) break;
        ZiFi::rxPushBytes(buf, (uint16_t)n);
        want -= (uint32_t)n;
        s->credit -= (uint32_t)n;
        got = true;
    }
    if (got) LED::touchR(LED::NET);
    s->T.flushAcks();
}

void status(char* out, int cap) {
    if (!s) { if (cap) out[0] = 0; return; }
    snprintf(out, cap, "AT emu, mux %d, %d link%s", s->E.mux() ? 1 : 0,
             s->E.openLinks(), s->E.openLinks() == 1 ? "" : "s");
}

}  // namespace AtEmu

#endif  // PICOSPECCY_WIFI
