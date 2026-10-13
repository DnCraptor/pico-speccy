#include "AtEmuCore.h"

// Only the on-chip-WiFi boards ship this (and the host test). The guard keeps
// every other board's image byte for byte what it was.
#if PICOSPECCY_WIFI || defined(ATEMU_HOST)

#include <string.h>
#include <stdio.h>
#include <stdarg.h>
#include <stdlib.h>

// Formats follow ESP8266 NONOS AT 1.7 (what MRF and the NedoOS clients were written
// against). The parsers on the other side are byte-scanners, so the exact framing
// matters more than the words:
//   - MRF's checkOkErr looks for 'O','K','\r' / "ERROR" / "FAIL" anywhere;
//   - its getPacket finds "+IPD," then reads the length up to ':', and takes a
//     bare "...OSED\r" as the end of the stream;
//   - esp-com's recvHead reads up to the first ',' and then the length to ':' -
//     so in mux 0 the length must follow "+IPD," directly.

void AtEmuCore::put(const char* s) { while (*s) putch(*s++); }

void AtEmuCore::putch(char c) {
    if (txtFree() <= 0) return;   // a response never outgrows the ring in practice
    txt[txtHead++ & (TXT_SIZE - 1)] = (uint8_t)c;
}

void AtEmuCore::putf(const char* fmt, ...) {
    char b[160];
    va_list ap; va_start(ap, fmt);
    vsnprintf(b, sizeof b, fmt, ap);
    va_end(ap);
    put(b);
}

bool AtEmuCore::anyOpen() const {
    for (int i = 0; i < MAX_LINKS; i++) if (L[i].st != L_FREE) return true;
    return false;
}

int AtEmuCore::openLinks() const {
    int n = 0;
    for (int i = 0; i < MAX_LINKS; i++) if (L[i].st != L_FREE) n++;
    return n;
}

void AtEmuCore::dropAll() {
    for (int i = 0; i < MAX_LINKS; i++) {
        if (L[i].st != L_FREE) T.close(i);
        L[i] = Link{};
    }
    ipdId = -1; ipdLeft = 0; hdrLen = hdrOff = 0;
}

void AtEmuCore::reset(bool banner) {
    dropAll();
    lineLen = 0; lineOverflow = false;
    echo = true; muxMode = false; dinfo = false;
    st = ST_CMD; sendLeft = sendTotal = stageLen = stageOff = 0;
    txtHead = txtTail = 0;
    lastLink = T.linkUp();
    rstPending = banner;
    rstAt = T.ms() + RST_DELAY_MS;
}

// ── input ────────────────────────────────────────────────────────────────────

void AtEmuCore::feed(uint8_t b) {
    if (st == ST_DATA) { dataByte(b); return; }
    if (st == ST_SENDING) return;               // caller must honour canAccept()
    if (echo) putch((char)b);
    if (b == '\n') {
        line[lineLen] = 0;
        if (lineLen && line[lineLen - 1] == '\r') line[--lineLen] = 0;
        execLine();
        lineLen = 0; lineOverflow = false;
        return;
    }
    if (lineLen < LINE_MAX) line[lineLen++] = (char)b;
    else lineOverflow = true;
}

void AtEmuCore::execLine() {
    // Strip leading CR/space noise (a guest that sends "\r\n" twice).
    const char* p = line;
    while (*p == '\r' || *p == ' ') p++;
    if (!*p) return;                            // empty line: an ESP stays silent
    if (lineOverflow) { error(); return; }
    // Busy with an async command: refuse, as the ESP does.
    if (rstPending) { put("busy p...\r\n"); return; }
    for (int i = 0; i < MAX_LINKS; i++)
        if (L[i].st == L_CONNECTING) { put("busy p...\r\n"); return; }
    exec(p);
}

static bool startsWith(const char* s, const char* pre, const char** rest) {
    size_t n = strlen(pre);
    if (strncmp(s, pre, n) != 0) return false;
    if (rest) *rest = s + n;
    return true;
}

// Parse a decimal int at *p, advancing past it. False if no digit.
static bool parseInt(const char*& p, long& v) {
    while (*p == ' ') p++;
    char* e;
    v = strtol(p, &e, 10);
    if (e == p) return false;
    p = e;
    return true;
}

static bool skipComma(const char*& p) {
    while (*p == ' ') p++;
    if (*p != ',') return false;
    p++;
    return true;
}

// Quoted string at *p into out (quotes optional), advancing past it.
static bool parseStr(const char*& p, char* out, size_t cap) {
    while (*p == ' ') p++;
    size_t n = 0;
    if (*p == '"') {
        p++;
        while (*p && *p != '"') {
            if (*p == '\\' && p[1]) p++;        // AT escapes \" \, \\ inside strings
            if (n + 1 < cap) out[n++] = *p;
            p++;
        }
        if (*p != '"') return false;
        p++;
    } else {
        while (*p && *p != ',') { if (n + 1 < cap) out[n++] = *p; p++; }
    }
    out[n] = 0;
    return true;
}

void AtEmuCore::exec(const char* c) {
    char log[96];
    snprintf(log, sizeof log, "AT< %s", c);
    T.log(log);

    const char* a = nullptr;
    if (!strcmp(c, "AT"))                                  { ok(); return; }
    if (!strcmp(c, "ATE0"))                                { echo = false; ok(); return; }
    if (!strcmp(c, "ATE1"))                                { echo = true;  ok(); return; }
    if (!strcmp(c, "AT+RST") || !strcmp(c, "AT+RESTORE")) {
        ok();
        // Links go silently, like a real reboot; the banner follows after a delay.
        dropAll();
        echo = true; muxMode = false; dinfo = false;
        rstPending = true; rstAt = T.ms() + RST_DELAY_MS;
        return;
    }
    if (!strcmp(c, "AT+GMR")) {
        put("\r\nAT version:1.7.4.0(pico-speccy AT emulator)\r\n"
            "SDK version:lwIP\r\ncompile time:" __DATE__ "\r\n");
        ok(); return;
    }
    // Station mode is the only one there is; accept every spelling of it.
    if (startsWith(c, "AT+CWMODE", &a)) {
        if (startsWith(a, "_CUR", &a) || startsWith(a, "_DEF", &a)) {}
        if (*a == '?') { put("\r\n+CWMODE:1\r\n"); ok(); return; }
        if (*a == '=') { ok(); return; }
        error(); return;
    }
    if (startsWith(c, "AT+CWJAP", &a)) {
        if (startsWith(a, "_CUR", &a) || startsWith(a, "_DEF", &a)) {}
        if (*a == '?') {
            if (T.linkUp()) putf("\r\n+CWJAP:\"%s\"\r\n", T.ssid());
            else            put("\r\nNo AP\r\n");
            ok(); return;
        }
        if (*a == '=') { cmdCwjap(a + 1); return; }
        error(); return;
    }
    if (startsWith(c, "AT+CWQAP", nullptr))                { ok(); return; } // never drop the host's WiFi
    if (startsWith(c, "AT+CWAUTOCONN", nullptr))           { ok(); return; }
    if (!strcmp(c, "AT+CIFSR"))                            { cmdCifsr(); return; }
    if (!strcmp(c, "AT+CIPSTATUS"))                        { cmdCipstatus(); return; }
    if (startsWith(c, "AT+CIPMUX", &a)) {
        if (*a == '?') { putf("\r\n+CIPMUX:%d\r\n", muxMode ? 1 : 0); ok(); return; }
        long v;
        if (*a == '=' && (++a, parseInt(a, v)) && (v == 0 || v == 1)) {
            if (anyOpen() && (bool)v != muxMode) { put("\r\nlink is builded\r\n"); error(); return; }
            muxMode = v != 0; ok(); return;
        }
        error(); return;
    }
    if (startsWith(c, "AT+CIPDINFO", &a)) {
        long v;
        if (*a == '=' && (++a, parseInt(a, v)) && (v == 0 || v == 1)) { dinfo = v != 0; ok(); return; }
        error(); return;
    }
    if (startsWith(c, "AT+CIPSERVER", &a)) {
        long v;
        if (*a == '=' && (++a, parseInt(a, v)) && v == 0) { ok(); return; }
        error(); return;          // listening sockets: not yet
    }
    if (startsWith(c, "AT+CIPRECVMODE", &a)) {
        long v;
        if (*a == '=' && (++a, parseInt(a, v)) && v == 0) { ok(); return; }
        error(); return;          // passive (pull) mode: not yet
    }
    if (startsWith(c, "AT+CIPMODE", &a)) {
        long v;
        if (*a == '=' && (++a, parseInt(a, v)) && v == 0) { ok(); return; }
        error(); return;          // transparent (pass-through) mode: not supported
    }
    if (startsWith(c, "AT+CIPSTO", nullptr))               { ok(); return; }
    if (startsWith(c, "AT+UART", nullptr))                 { ok(); return; } // pace stays the menu's
    if (startsWith(c, "AT+SYSRAM", nullptr))               { put("\r\n+SYSRAM:32768\r\n"); ok(); return; }
    if (startsWith(c, "AT+CIPSNTPCFG", nullptr))           { ok(); return; }
    if (!strcmp(c, "AT+CIPSNTPTIME?"))                     { cmdSntpTime(); return; }
    if (startsWith(c, "AT+CIPSTART=", &a))                 { cmdCipstart(a); return; }
    if (startsWith(c, "AT+CIPSEND=", &a))                  { cmdCipsend(a); return; }
    if (startsWith(c, "AT+CIPCLOSE", &a))                  { cmdCipclose(a); return; }
    error();
}

void AtEmuCore::cmdCwjap(const char* a) {
    // The guest does not get to move the host's WiFi (owner's rule): if it is up,
    // say it joined, whatever it asked for.
    char ssidReq[33];
    const char* p = a;
    if (!parseStr(p, ssidReq, sizeof ssidReq)) { error(); return; }
    if (T.linkUp()) {
        if (strcmp(ssidReq, T.ssid()) != 0) {
            char log[96];
            snprintf(log, sizeof log, "AT: CWJAP \"%s\" ignored, staying on \"%s\"", ssidReq, T.ssid());
            T.log(log);
        }
        put("WIFI CONNECTED\r\nWIFI GOT IP\r\n");
        lastLink = true;
        ok();
    } else {
        put("\r\n+CWJAP:3\r\n\r\nFAIL\r\n");
    }
}

void AtEmuCore::cmdCifsr() {
    char ip[20] = "0.0.0.0", mac[20] = "00:00:00:00:00:00";
    T.ip(ip, sizeof ip);
    T.mac(mac, sizeof mac);
    putf("+CIFSR:STAIP,\"%s\"\r\n+CIFSR:STAMAC,\"%s\"\r\n", ip, mac);
    ok();
}

void AtEmuCore::cmdCipstatus() {
    // 2 = got IP, 3 = connected, 5 = no WiFi.
    int s = !T.linkUp() ? 5 : (anyOpen() ? 3 : 2);
    putf("STATUS:%d\r\n", s);
    for (int i = 0; i < MAX_LINKS; i++) {
        if (L[i].st != L_OPEN) continue;
        char ip[48] = ""; uint16_t port = 0;
        T.remote(i, ip, sizeof ip, port);
        putf("+CIPSTATUS:%d,\"TCP\",\"%s\",%u,0,0\r\n", i, ip, (unsigned)port);
    }
    ok();
}

void AtEmuCore::cmdSntpTime() {
    static const char* const DOW[] = { "Sun","Mon","Tue","Wed","Thu","Fri","Sat" };
    static const char* const MON[] = { "Jan","Feb","Mar","Apr","May","Jun",
                                       "Jul","Aug","Sep","Oct","Nov","Dec" };
    int y, mo, d, h, mi, s;
    if (!T.now(y, mo, d, h, mi, s) || mo < 1 || mo > 12) {
        put("+CIPSNTPTIME:Thu Jan 01 00:00:00 1970\r\n"); ok(); return;
    }
    // Day of week (Sakamoto).
    static const int tt[] = { 0,3,2,5,0,3,5,1,4,6,2,4 };
    int yy = y - (mo < 3);
    int dow = (yy + yy / 4 - yy / 100 + yy / 400 + tt[mo - 1] + d) % 7;
    putf("+CIPSNTPTIME:%s %s %02d %02d:%02d:%02d %d\r\n", DOW[dow], MON[mo - 1], d, h, mi, s, y);
    ok();
}

void AtEmuCore::cmdCipstart(const char* a) {
    const char* p = a;
    long id = 0;
    if (muxMode) {
        if (!parseInt(p, id) || !skipComma(p) || id < 0 || id >= MAX_LINKS) { error(); return; }
    }
    char type[8], host[96];
    long port;
    if (!parseStr(p, type, sizeof type) || !skipComma(p) ||
        !parseStr(p, host, sizeof host) || !skipComma(p) || !parseInt(p, port) ||
        port <= 0 || port > 65535) { error(); return; }
    if (strcmp(type, "TCP") != 0) { error(); return; }      // UDP / SSL: not yet
    if (L[id].st != L_FREE) { put("ALREADY CONNECTED\r\n"); error(); return; }
    if (!T.linkUp()) { put("no ip\r\n"); error(); return; }
    if (!T.open((int)id, host, (uint16_t)port)) {
        put("ERROR\r\nCLOSED\r\n"); return;
    }
    L[id].st = L_CONNECTING;
    L[id].deadline = T.ms() + CONNECT_TIMEOUT_MS;
}

void AtEmuCore::cmdCipsend(const char* a) {
    const char* p = a;
    long id = 0, len;
    if (muxMode) {
        if (!parseInt(p, id) || !skipComma(p) || id < 0 || id >= MAX_LINKS) { error(); return; }
    }
    if (!parseInt(p, len) || len <= 0 || len > SEND_MAX) { error(); return; }
    if (L[id].st != L_OPEN || T.closed((int)id)) { put("link is not valid\r\n"); error(); return; }
    sendId = (int)id; sendLeft = sendTotal = (int)len;
    stageLen = stageOff = 0;
    st = ST_DATA;
    put("\r\nOK\r\n> ");
}

void AtEmuCore::cmdCipclose(const char* a) {
    long id = 0;
    if (*a == '=') {
        a++;
        if (!parseInt(a, id)) { error(); return; }
        if (id == 5) {                                       // 5 = all links
            bool any = anyOpen();
            for (int i = 0; i < MAX_LINKS; i++)
                if (L[i].st != L_FREE) { T.close(i); L[i] = Link{}; putf("%d,CLOSED\r\n", i); }
            if (any) ok(); else error();
            return;
        }
        if (id < 0 || id >= MAX_LINKS) { error(); return; }
    } else if (*a) { error(); return; }
    if (L[id].st == L_FREE) { error(); return; }
    // An +IPD already announced for this link still owes the guest its bytes:
    // produce() pads it out, so the guest's parser stays in step.
    T.close((int)id);
    L[id] = Link{};
    if (muxMode) putf("%d,CLOSED\r\n", (int)id); else put("CLOSED\r\n");
    ok();
}

// ── CIPSEND payload ──────────────────────────────────────────────────────────

void AtEmuCore::dataByte(uint8_t b) {
    stage[stageLen++] = b;
    sendLeft--;
    if (stageLen == STAGE || sendLeft == 0) {
        st = ST_SENDING;
        flushStage();
    }
}

void AtEmuCore::flushStage() {
    while (stageOff < stageLen) {
        int n = T.send(sendId, stage + stageOff, stageLen - stageOff);
        if (n < 0) {                                        // the link died under us
            stageLen = stageOff = 0;
            st = ST_CMD;
            put("\r\nSEND FAIL\r\n");
            return;
        }
        if (n == 0) return;                                 // socket full: retry on tick()
        stageOff += n;
    }
    stageLen = stageOff = 0;
    if (sendLeft > 0) { st = ST_DATA; return; }
    st = ST_CMD;
    putf("\r\nRecv %d bytes\r\n\r\nSEND OK\r\n", sendTotal);
}

// ── async ────────────────────────────────────────────────────────────────────

void AtEmuCore::tick() {
    const uint32_t now = T.ms();

    if (st == ST_SENDING) flushStage();

    if (rstPending && (int32_t)(now - rstAt) >= 0) {
        rstPending = false;
        put("\r\n ets Jan  8 2013,rst cause:2, boot mode:(3,7)\r\n\r\nready\r\n");
        lastLink = T.linkUp();
        if (lastLink) put("WIFI CONNECTED\r\nWIFI GOT IP\r\n");
    }

    for (int i = 0; i < MAX_LINKS; i++) {
        if (L[i].st != L_CONNECTING) continue;
        int s = T.openState(i);
        bool timedOut = s == 0 && (int32_t)(now - L[i].deadline) >= 0;
        if (s == 1) {
            L[i].st = L_OPEN;
            if (muxMode) putf("%d,CONNECT\r\n", i); else put("CONNECT\r\n");
            ok();
        } else if (s < 0 || timedOut) {
            T.close(i);
            L[i] = Link{};
            put("ERROR\r\nCLOSED\r\n");
        }
    }

    linkEvents();
}

// Peer-closed links and WiFi transitions become unsolicited lines. A link's
// CLOSED waits until every byte it received has been handed out — MRF ends the
// page on it.
void AtEmuCore::linkEvents() {
    if (txtHead != txtTail) return;      // never in the middle of a response
    for (int i = 0; i < MAX_LINKS; i++) {
        if (L[i].st != L_OPEN) continue;
        if (ipdId == i || T.avail(i) > 0 || !T.closed(i)) continue;
        if (st != ST_CMD && sendId == i) continue;
        T.close(i);
        L[i] = Link{};
        if (muxMode) putf("%d,CLOSED\r\n", i); else put("CLOSED\r\n");
    }
    bool up = T.linkUp();
    if (up != lastLink && !rstPending) {
        lastLink = up;
        put(up ? "WIFI CONNECTED\r\nWIFI GOT IP\r\n" : "WIFI DISCONNECT\r\n");
    }
}

void AtEmuCore::startIpd() {
    if (txtHead != txtTail || st == ST_DATA) return;
    for (int k = 0; k < MAX_LINKS; k++) {
        int i = (rr + k) % MAX_LINKS;
        if (L[i].st != L_OPEN) continue;
        int n = T.avail(i);
        if (n <= 0) continue;
        if (n > IPD_CHUNK) n = IPD_CHUNK;
        int h = muxMode ? snprintf(hdr, sizeof hdr, "\r\n+IPD,%d,%d", i, n)
                        : snprintf(hdr, sizeof hdr, "\r\n+IPD,%d", n);
        if (dinfo) {
            char ip[20] = ""; uint16_t port = 0;
            T.remote(i, ip, sizeof ip, port);
            h += snprintf(hdr + h, sizeof hdr - h, ",\"%s\",%u", ip, (unsigned)port);
        }
        if (h > (int)sizeof hdr - 2) h = (int)sizeof hdr - 2;
        hdr[h++] = ':';
        hdrLen = (uint8_t)h; hdrOff = 0;
        ipdId = i; ipdLeft = n;
        rr = (i + 1) % MAX_LINKS;
        return;
    }
}

// ── output ───────────────────────────────────────────────────────────────────

int AtEmuCore::produce(uint8_t* out, int max) {
    int n = 0;
    while (n < max) {
        // An +IPD in flight goes out whole first: header, then its data.
        if (hdrOff < hdrLen) { out[n++] = (uint8_t)hdr[hdrOff++]; continue; }
        if (ipdLeft > 0) {
            int want = max - n;
            if (want > ipdLeft) want = ipdLeft;
            int got = T.recv(ipdId, out + n, want);
            if (got <= 0) {
                // The bytes the header promised are gone (link dropped under us):
                // pad so the guest's parser stays in step.
                while (n < max && ipdLeft > 0) { out[n++] = 0; ipdLeft--; }
            } else {
                n += got; ipdLeft -= got;
            }
            if (ipdLeft == 0) ipdId = -1;
            continue;
        }
        // Then response text; a new +IPD only starts once that is out.
        if (txtHead != txtTail) {
            out[n++] = txt[txtTail++ & (TXT_SIZE - 1)];
            continue;
        }
        startIpd();
        if (hdrOff < hdrLen) continue;
        if (txtHead == txtTail) {
            linkEvents();
            if (txtHead == txtTail) break;
        }
    }
    return n;
}

#endif  // PICOSPECCY_WIFI || ATEMU_HOST
