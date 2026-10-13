// Host test for the ESP-AT emulator core (src/net/AtEmuCore.*).
//
//   g++ -O2 -Wall -Wextra -DATEMU_HOST -Isrc -fsanitize=address,undefined
//       -o /tmp/atemu_test tools/atemu_test.cpp src/net/AtEmuCore.cpp && /tmp/atemu_test
//
// The fake transport stands in for lwIP. The guest side is played by C++ copies
// of the two parsers the emulator exists for:
//   - MRF (nihirash/moon-rabbit-zx drivers/wifi.asm): checkOkErr, tcpSendZ, getPacket;
//   - NedoOS kapps/common/esp-com.c: recvHead.
// They are byte-scanners, so they are what decides whether the framing is right.
// Output is drained in small random slices to model the paced feed.

#include "net/AtEmuCore.h"
#include <cstdio>
#include <cstring>
#include <cstdlib>
#include <string>
#include <vector>
#include <deque>

static int fails = 0;
#define CHECK(c) do { if (!(c)) { fails++; std::printf("FAIL %s:%d  %s\n", __FILE__, __LINE__, #c); } } while (0)

struct FakeLink {
    bool used = false, failNext = false;
    int  state = 0;          // 0 connecting, 1 up, -1 failed
    int  connectAfter = 0;   // ticks until state = 1
    std::deque<uint8_t> rx;  // server -> guest
    std::string sent;        // guest -> server
    bool fin = false;
    int  sendRoom = 1 << 30; // bytes the socket accepts per call
    std::string host; uint16_t port = 0;
};

struct Fake : AtTransport {
    bool up = true;
    uint32_t t = 0;
    FakeLink L[AtEmuCore::MAX_LINKS];
    bool failOpen = false;
    bool linkUp() override { return up; }
    bool ip(char* o, size_t c) override { snprintf(o, c, "192.168.1.42"); return up; }
    bool mac(char* o, size_t c) override { snprintf(o, c, "28:cd:c1:00:11:22"); return true; }
    const char* ssid() override { return "home"; }
    bool now(int& y, int& mo, int& d, int& h, int& mi, int& s) override {
        y = 2026; mo = 10; d = 12; h = 14; mi = 5; s = 9; return true;
    }
    uint32_t ms() override { return t; }
    bool open(int id, const char* host, uint16_t port) override {
        if (failOpen) return false;
        FakeLink& l = L[id];
        bool keepFail = l.failNext;
        std::deque<uint8_t> pre = l.rx;     // tests may preload the reply
        bool preFin = l.fin;
        l = FakeLink{};
        l.used = true; l.host = host; l.port = port; l.connectAfter = 2;
        l.state = keepFail ? -1 : 0;
        l.rx = pre; l.fin = preFin;
        return true;
    }
    int openState(int id) override {
        FakeLink& l = L[id];
        if (l.state == 0 && --l.connectAfter <= 0) l.state = 1;
        return l.state;
    }
    int send(int id, const uint8_t* p, int n) override {
        FakeLink& l = L[id];
        if (!l.used || l.state != 1) return -1;
        int k = n < l.sendRoom ? n : l.sendRoom;
        l.sent.append((const char*)p, k);
        return k;
    }
    int avail(int id) override { return (int)L[id].rx.size(); }
    int recv(int id, uint8_t* p, int n) override {
        int k = 0;
        while (k < n && !L[id].rx.empty()) { p[k++] = L[id].rx.front(); L[id].rx.pop_front(); }
        return k;
    }
    bool closed(int id) override { return L[id].fin; }
    void close(int id) override { L[id].used = false; L[id].rx.clear(); L[id].fin = false; }
    bool remote(int id, char* ip, size_t cap, uint16_t& port) override {
        snprintf(ip, cap, "10.0.0.%d", id + 1); port = L[id].port; return true;
    }
};

// ── harness: the guest's view ────────────────────────────────────────────────
struct Guest {
    Fake& F; AtEmuCore& E;
    std::deque<uint8_t> in;      // emulator -> guest, already produced
    std::string all;             // every byte the guest ever saw
    unsigned seed = 1;
    Guest(Fake& f, AtEmuCore& e) : F(f), E(e) {}

    void step() {
        F.t += 5;
        E.tick();
        seed = seed * 1103515245u + 12345u;
        int slice = 1 + (int)((seed >> 16) % 40);     // paced feed: small random slices
        uint8_t b[64];
        int n = E.produce(b, slice);
        for (int i = 0; i < n; i++) { in.push_back(b[i]); all.push_back((char)b[i]); }
    }
    void send(const std::string& s) {
        for (unsigned char c : s) {
            int guard = 0;
            while (!E.canAccept() && guard++ < 10000) step();
            E.feed(c);
        }
    }
    // Blocking read with a step budget (like the guest's own timeout).
    int read() {
        for (int k = 0; k < 20000; k++) {
            if (!in.empty()) { int c = in.front(); in.pop_front(); return c; }
            step();
        }
        return -1;
    }
    // MRF checkOkErr: true = OK, false = ERROR/FAIL, -1 timeout -> false + flag.
    int checkOkErr() {
        for (;;) {
            int c = read(); if (c < 0) return -1;
            if (c == 'O') { if (read() != 'K') continue; if (read() != 13) continue; while ((c = read()) != 10) if (c < 0) return -1; return 1; }
            if (c == 'E') { if (read()!='R'||read()!='R'||read()!='O'||read()!='R') continue; while ((c = read()) != 10) if (c < 0) return -1; return 0; }
            if (c == 'F') { if (read()!='A'||read()!='I'||read()!='L') continue; while ((c = read()) != 10) if (c < 0) return -1; return 0; }
        }
    }
    int cmd(const std::string& c) { send(c + "\r\n"); return checkOkErr(); }
    // MRF tcpSendZ
    int tcpSendZ(const std::string& s) {
        send("AT+CIPSEND=" + std::to_string(s.size() + 2) + "\r\n");
        int r = checkOkErr(); if (r != 1) return r;
        int c; while ((c = read()) != '>') if (c < 0) return -1;
        send(s + "\r\n");
        return checkOkErr();
    }
    // MRF getPacket: returns payload bytes appended to buf; true when CLOSED seen.
    // -1 = timeout.
    int getPacket(std::string& buf) {
        for (;;) {
            int c = read(); if (c < 0) return -1;
            if (c == 'O') {
                if (read()!='S'||read()!='E'||read()!='D') continue;
                if (read() != 13) continue;
                return 1;
            }
            if (c == '+') {
                if (read()!='I'||read()!='P'||read()!='D') continue;
                read();                                   // comma
                int len = 0;
                for (;;) { c = read(); if (c < 0) return -1; if (c == ':') break; len = len * 10 + (c - '0'); }
                for (int i = 0; i < len; i++) { c = read(); if (c < 0) return -1; buf.push_back((char)c); }
                return 0;
            }
        }
    }
    // esp-com recvHead: returns length, 0 on CLOSED/ERROR, -1 timeout.
    int recvHead() {
        const char* closed = "CLOSED"; const char* error = "ERROR";
        size_t cc = 0, ce = 0; std::string nb; int c;
        do {
            c = read(); if (c < 0) return -1;
            nb.push_back((char)c);
            if (c == closed[cc]) { if (++cc == 6) return 0; } else cc = 0;
            if (c == error[ce])  { if (++ce == 5) return 0; } else ce = 0;
        } while (c != ',');
        size_t toComa = nb.size();
        do { c = read(); if (c < 0) return -1; nb.push_back((char)c); } while (c != ':');
        return atoi(nb.c_str() + toComa);
    }
};

static std::string bytes(int n, unsigned seed) {
    std::string s;
    for (int i = 0; i < n; i++) { seed = seed * 1664525u + 1013904223u; s.push_back((char)(seed >> 24)); }
    return s;
}

// ── tests ────────────────────────────────────────────────────────────────────

static void testMrfSession() {
    Fake F; AtEmuCore E(F); Guest G(F, E);
    // Wifi.init
    CHECK(G.cmd("ATE0") == 1);
    CHECK(G.cmd("AT+CIPSERVER=0") == 1);
    CHECK(G.cmd("AT+CIPCLOSE") == 0);                 // nothing open -> ERROR (MRF ignores it)
    CHECK(G.cmd("AT+CIPMUX=0") == 1);
    CHECK(G.cmd("AT+CIPDINFO=0") == 1);
    // A 5000-byte gopher page, delivered and then FIN.
    std::string page = bytes(5000, 7);
    // openTCP
    G.cmd("AT+CIPCLOSE");
    CHECK(G.cmd("AT+CIPSTART=\"TCP\",\"gopher.example\",70") == 1);
    CHECK(F.L[0].host == "gopher.example" && F.L[0].port == 70);
    CHECK(G.tcpSendZ("/docs") == 1);
    CHECK(F.L[0].sent == "/docs\r\n");
    for (unsigned char c : page) F.L[0].rx.push_back(c);   // the server answers the request
    F.L[0].fin = true;
    std::string got; int r;
    while ((r = G.getPacket(got)) == 0) {}
    CHECK(r == 1);
    CHECK(got == page);
    CHECK(!F.L[0].used);                              // released after CLOSED
    // Reload: the same sequence must work again on the freed link.
    G.cmd("AT+CIPCLOSE");
    CHECK(G.cmd("AT+CIPSTART=\"TCP\",\"gopher.example\",70") == 1);
    CHECK(G.tcpSendZ("/") == 1);
    for (unsigned char c : std::string("again")) F.L[0].rx.push_back(c);
    F.L[0].fin = true;
    got.clear();
    while ((r = G.getPacket(got)) == 0) {}
    CHECK(r == 1 && got == "again");
}

static void testEspComSession() {
    Fake F; AtEmuCore E(F); Guest G(F, E);
    // espReBoot: AT+RST, then wait for "WIFI GOT IP".
    G.send("AT+RST\r\n");
    std::string s; int c; const char* want = "WIFI GOT IP"; size_t k = 0;
    while (k < strlen(want) && (c = G.read()) >= 0) { if (c == want[k]) k++; else k = (c == want[0]); }
    CHECK(k == strlen(want));
    G.send("ATE0\r\n");
    while ((c = G.read()) >= 0 && c != 'K') {}
    CHECK(c == 'K');
    CHECK(G.read() == 13 && G.read() == 10);
    CHECK(G.cmd("AT+CIPCLOSE") == 0);
    CHECK(G.cmd("AT+CIPDINFO=0") == 1);
    CHECK(G.cmd("AT+CIPMUX=0") == 1);
    CHECK(G.cmd("AT+CIPSERVER=0") == 1);
    CHECK(G.cmd("AT+CIPRECVMODE=0") == 1);
    std::string payload = bytes(3000, 3);
    CHECK(G.cmd("AT+CIPSTART=\"TCP\",\"1.2.3.4\",80") == 1);
    // send a request the esp-com way: CIPSEND, wait '>', data, then read heads.
    G.send("AT+CIPSEND=4\r\n");
    while ((c = G.read()) >= 0 && c != '>') {}
    G.send("GET\n");
    CHECK(G.checkOkErr() == 1);                      // SEND OK
    for (unsigned char ch : payload) F.L[0].rx.push_back(ch);
    std::string got;
    while (got.size() < payload.size()) {
        int len = G.recvHead();
        if (len <= 0) break;
        for (int i = 0; i < len; i++) got.push_back((char)G.read());
    }
    CHECK(got == payload);
    CHECK(F.L[0].sent == "GET\n");
    F.L[0].fin = true;
    CHECK(G.recvHead() == 0);                        // CLOSED ends the stream
}

static void testMux() {
    Fake F; AtEmuCore E(F); Guest G(F, E);
    CHECK(G.cmd("ATE0") == 1);
    CHECK(G.cmd("AT+CIPMUX=1") == 1);
    CHECK(G.cmd("AT+CIPSTART=2,\"TCP\",\"a\",1") == 1);
    CHECK(G.all.find("2,CONNECT\r\n") != std::string::npos);
    CHECK(G.cmd("AT+CIPMUX=0") == 0);                // links open -> refuse
    for (unsigned char c : std::string("xyz")) F.L[2].rx.push_back(c);
    std::string seen;
    for (int i = 0; i < 200 && seen.find("xyz") == std::string::npos; i++) { G.step(); while (!G.in.empty()) { seen.push_back((char)G.in.front()); G.in.pop_front(); } }
    CHECK(seen.find("+IPD,2,3:xyz") != std::string::npos);
    CHECK(G.cmd("AT+CIPSEND=2,3") == 1);
    int c; while ((c = G.read()) >= 0 && c != '>') {}
    G.send("abc");
    CHECK(G.checkOkErr() == 1);
    CHECK(F.L[2].sent == "abc");
    CHECK(G.cmd("AT+CIPCLOSE=2") == 1);
    CHECK(G.all.find("2,CLOSED") != std::string::npos);
}

static void testSendBackpressure() {
    Fake F; AtEmuCore E(F); Guest G(F, E);
    CHECK(G.cmd("ATE0") == 1);
    CHECK(G.cmd("AT+CIPSTART=\"TCP\",\"h\",9") == 1);
    F.L[0].sendRoom = 7;                              // the socket takes 7 bytes a call
    std::string big = bytes(2000, 9);
    G.send("AT+CIPSEND=2000\r\n");
    int c; while ((c = G.read()) >= 0 && c != '>') {}
    G.send(big);
    CHECK(G.checkOkErr() == 1);
    CHECK(F.L[0].sent == big);
    CHECK(G.all.find("Recv 2000 bytes") != std::string::npos);
    CHECK(G.cmd("AT+CIPSEND=2049") == 0);             // over the ESP limit
}

static void testErrors() {
    Fake F; AtEmuCore E(F); Guest G(F, E);
    CHECK(G.cmd("ATE0") == 1);
    CHECK(G.cmd("AT+BOGUS") == 0);
    CHECK(G.cmd("AT+CIPSEND=5") == 0);                // no link
    CHECK(G.all.find("link is not valid") != std::string::npos);
    CHECK(G.cmd("AT+CIPSTART=\"UDP\",\"h\",9") == 0); // UDP: not yet
    F.L[0].failNext = true;
    CHECK(G.cmd("AT+CIPSTART=\"TCP\",\"h\",9") == 0); // refused
    CHECK(G.all.rfind("CLOSED") != std::string::npos);
    CHECK(G.cmd("AT+CIPSTART=\"TCP\",\"h\",9") == 1);
    CHECK(G.cmd("AT+CIPSTART=\"TCP\",\"h\",9") == 0); // already
    CHECK(G.all.find("ALREADY CONNECTED") != std::string::npos);
    F.up = false;
    G.cmd("AT+CIPCLOSE");
    CHECK(G.cmd("AT+CIPSTART=\"TCP\",\"h\",9") == 0); // no WiFi
    CHECK(G.cmd("AT+CWJAP=\"other\",\"pw\"") == 0);   // FAIL with no host link
    F.up = true;
    CHECK(G.cmd("AT+CWJAP_DEF=\"other\",\"pw\"") == 1); // ignored, reports connected
    CHECK(G.cmd("AT+CWQAP") == 1);
    CHECK(F.up);
    std::string longline(300, 'A');
    CHECK(G.cmd(longline) == 0);
}

static void testEchoAndInfo() {
    Fake F; AtEmuCore E(F); Guest G(F, E);
    G.send("AT\r\n");
    CHECK(G.checkOkErr() == 1);
    CHECK(G.all.substr(0, 4) == "AT\r\n");             // echo on after power-up
    CHECK(G.cmd("ATE0") == 1);
    size_t mark = G.all.size();
    CHECK(G.cmd("AT+CIFSR") == 1);
    std::string r = G.all.substr(mark);
    CHECK(r.find("AT+CIFSR") == std::string::npos);    // no echo
    CHECK(r.find("+CIFSR:STAIP,\"192.168.1.42\"") != std::string::npos);
    CHECK(G.cmd("AT+CIPSNTPTIME?") == 1);
    CHECK(G.all.find("+CIPSNTPTIME:Mon Oct 12 14:05:09 2026") != std::string::npos);
    CHECK(G.cmd("AT+CWJAP?") == 1);
    CHECK(G.all.find("+CWJAP:\"home\"") != std::string::npos);
    CHECK(G.cmd("AT+CIPSTATUS") == 1);
    CHECK(G.all.find("STATUS:2") != std::string::npos);
    CHECK(G.cmd("AT+CIPDINFO=1") == 1);
    CHECK(G.cmd("AT+CIPSTART=\"TCP\",\"h\",80") == 1);
    for (unsigned char c : std::string("hi")) F.L[0].rx.push_back(c);
    std::string seen;
    for (int i = 0; i < 200 && seen.find("hi") == std::string::npos; i++) { G.step(); while (!G.in.empty()) { seen.push_back((char)G.in.front()); G.in.pop_front(); } }
    CHECK(seen.find("+IPD,2,\"10.0.0.1\",80:hi") != std::string::npos);
}

// A response generated while an +IPD streams must not land inside its data.
static void testNoInterleave() {
    Fake F; AtEmuCore E(F); Guest G(F, E);
    CHECK(G.cmd("ATE0") == 1);
    CHECK(G.cmd("AT+CIPMUX=1") == 1);
    CHECK(G.cmd("AT+CIPSTART=0,\"TCP\",\"h\",1") == 1);
    std::string data = bytes(1400, 11);
    for (unsigned char c : data) F.L[0].rx.push_back(c);
    // Start the +IPD, take a little of it, then fire a command mid-stream.
    for (int i = 0; i < 3; i++) G.step();
    G.send("AT\r\n");
    std::string s;
    for (int i = 0; i < 5000 && s.find("OK\r\n") == std::string::npos; i++) { G.step(); while (!G.in.empty()) { s.push_back((char)G.in.front()); G.in.pop_front(); } }
    s = G.all;
    size_t h = s.find("+IPD,0,1400:");
    CHECK(h != std::string::npos);
    if (h != std::string::npos) CHECK(s.compare(h + 12, data.size(), data) == 0);
}

int main() {
    testMrfSession();
    testEspComSession();
    testMux();
    testSendBackpressure();
    testErrors();
    testEchoAndInfo();
    testNoInterleave();
    std::printf("%s (%d failures)\n", fails ? "FAILED" : "OK", fails);
    return fails ? 1 : 0;
}
