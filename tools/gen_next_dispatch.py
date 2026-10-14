#!/usr/bin/env python3
# Generates src/speccy/z80/next_dispatch.inc: the opcode switch of
# Z80::exec_nocheck_next (the Spectrum Next loop, .nxovl window).
#
#   python3 tools/gen_next_dispatch.py
#
# Declares nothing itself: the includer provides `uint8_t nx_v; uint8_t* nx_p; bool nx_cc;`.
# Every unprefixed opcode either
#   - is SPECIALISED inline here (LD r,r' / ALU r / INC r / DEC r: the generic
#     decodeLD8 / decodeALU8 / decodeINCDEC8 bodies with the register picked at
#     compile time instead of through Z80::reg8[] and the stored opCode),
#   - calls its handler DIRECTLY (the handler is marked NX_DISP_INL, so GCC
#     inlines it here while dcOpcode[] keeps its out-of-line copy), or
#   - goes through dcOpcode[] as before (prefixes, HALT, the tape trap, RST,
#     IN/OUT, EI/DI: rare, or with hooks that read opCode / the slice) — and
#     returns from the loop if the handler dropped g_next_fastrd.
# The handlers' semantics are therefore the core's own; only the dispatch moves.
# A directly-called handler is in fact pasted in: its body is copied out of
# Z80_JLS.cpp with the memory accessors swapped for the gate-free Next-inline
# ones (Z80Ops::/NextMemory::peek8/peek16/poke8/addressOnBus[IR] ->
# NextMemory::..._nx, push -> nxPush, pop -> nxPop, check_trdos -> nxCheckTrdos:
# its Next branch inline, condMet((opCode >> 3) & 7) -> nx_cc).
# A body with `static` or a loop around a `return` keeps the call.
import re, os, sys
ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
src = open(os.path.join(ROOT, 'src/speccy/z80/Z80_JLS.cpp')).read()
m = re.search(r'void \(\*Z80::dcOpcode\[256\]\)\(\) = \{(.*?)\};', src, re.S)
names = re.findall(r'&(\w+)', m.group(1))
assert len(names) == 256, len(names)
def body_of(name):
    m = re.search(r'NX_DISP_INL (?:IRAM_ATTR )?void Z80::%s\(void\)|NX_DISP_INL (?:IRAM_ATTR )?void Z80::%s\(\)' % (name, name), src)
    assert m, name
    i = src.index('{', m.end())
    depth = 0
    j = i
    while True:
        c = src[j]
        if src.startswith('//', j): j = src.index('\n', j); continue
        if src.startswith('/*', j): j = src.index('*/', j) + 2; continue
        if c == '{': depth += 1
        elif c == '}':
            depth -= 1
            if depth == 0: break
        j += 1
    body = src[i + 1:j]
    body = re.sub(r'//[^\n]*', '', body)
    body = re.sub(r'/\*.*?\*/', '', body, flags=re.S)
    # `return;` leaves the handler; pasted into the switch, `break;` leaves the
    # case the same way — as long as no loop or switch of its own is around it.
    if re.search(r'\bstatic\b|\breturn\s+[^;]', body): return None
    if re.search(r'\breturn\b', body):
        if re.search(r'\b(for|while|switch|do)\b', body): return None
        body = re.sub(r'\breturn\s*;', 'break;', body)
    # The gate-free accessors (next_memory.h): inside this loop g_next_fastrd
    # is an invariant — see the TABLE cases below, which end the slice when a
    # handler dropped it. A body already written against NextMemory:: (the
    # shared handlers) is rewritten the same way.
    for a, b in (('Z80Ops::peek8(', 'NextMemory::peek8_nx('), ('Z80Ops::peek16(', 'NextMemory::peek16_nx('),
                 ('Z80Ops::poke8(', 'NextMemory::poke8_nx('), ('Z80Ops::poke16(', 'NextMemory::poke16_nx('), ('Z80Ops::addressOnBusIR(', 'NextMemory::addressOnBusIR_nx('),
                 ('Z80Ops::addressOnBus(', 'NextMemory::addressOnBus_nx('),
                 ('NextMemory::peek8(', 'NextMemory::peek8_nx('), ('NextMemory::peek16(', 'NextMemory::peek16_nx('),
                 ('NextMemory::poke8(', 'NextMemory::poke8_nx('), ('NextMemory::addressOnBusIR(', 'NextMemory::addressOnBusIR_nx('),
                 ('NextMemory::addressOnBus(', 'NextMemory::addressOnBus_nx(')):
        body = body.replace(a, b)
    body = re.sub(r'(?<![\w:.])push\(', 'nxPush(', body)
    body = re.sub(r'(?<![\w:.])pop\(\)', 'nxPop()', body)
    body = re.sub(r'(?<![\w:.])check_trdos\(\)', 'nxCheckTrdos()', body)
    body = re.sub(r'(?<![\w:.])nextIdleNote\(\)', 'nxIdleNote()', body)
    # The cc families: the condition is decided per case (nx_cc, below), not
    # by a condMet() call on the stored opcode.
    body = body.replace('condMet((opCode >> 3) & 7)', 'nx_cc')
    return ' '.join(body.split())

# cc = (opcode >> 3) & 7: NZ Z NC C PO PE P M — condMet()'s table, inline.
CC = ['(sz5h3pnFlags & ZERO_MASK) == 0', '(sz5h3pnFlags & ZERO_MASK) != 0',
      '!carryFlag', 'carryFlag',
      '(sz5h3pnFlags & PARITY_MASK) == 0', '(sz5h3pnFlags & PARITY_MASK) != 0',
      '(sz5h3pnFlags & SIGN_MASK) == 0', '(sz5h3pnFlags & SIGN_MASK) != 0']

def call(name):
    b = body_of(name)
    return ('{ %s }' % b) if b is not None else '%s();' % name

TABLE = {0xCB, 0xDD, 0xED, 0xFD,      # prefixes: own second fetch / big decoders
         0x76, 0xBF,                  # HALT, CP A (tape LOAD trap)
         0xD3, 0xDB,                  # OUT (n),A / IN A,(n): the port path dominates
         0xF3, 0xFB}                  # DI / EI: interrupt hooks
# The gate-free accessors (next_memory.h) take the loop's T-state register
# first: CPU::tstates lives in `nx_t` for the whole slice (exec_nocheck_next).
def nxt(line):
    line = re.sub(r'NextMemory::(peek8|peek16|poke8|poke16|addressOnBus|addressOnBusIR)_nx\(', r'NextMemory::\1_nx(nx_t, ', line)
    # Z80Ops::poke16 has a RegisterPair overload (LD (nn),HL); poke16_nx takes the word
    line = re.sub(r'(poke16_nx\(nx_t, [^,]+, reg[A-Z]{2})\)', r'\1.word)', line)
    return line
R8 = ['REG_B', 'REG_C', 'REG_D', 'REG_E', 'REG_H', 'REG_L', None, 'regA']
ALU = ['add', 'adc', 'sub', 'sbc', 'and_', 'xor_', 'or_', 'cp']
out = ['// GENERATED by tools/gen_next_dispatch.py - do not edit.',
       '// Included inside Z80::exec_nocheck_next (Z80_JLS.cpp).']
inl = set()
for op in range(256):
    n = names[op]
    if op == 0xED:
        # ED: the second M1 here (NextOpcode::fetch_nx, as decodeOpcodeed does)
        # and the ED / Z80N ops a sample mixer lives on run inline — the
        # NXModPlayer profile of 2026-10-14 (per mille of all instructions):
        # NEXTREG r,A 19, SWAPNIB 11, RETI 11, TEST n 7, SBC HL,DE 7,
        # LD DE,(nn) 6, NEXTREG n,n 4, ADD rr,A 4, LD BC,(nn) 3 (+ MUL, ADD rr,nn
        # from 2026-10-14 noon). Bodies are decodeED's / decodeZ80N's with the
        # gate-free accessors; NEXTREG and RETI call out (NextReg::writeHot may
        # touch the raster / MMU and drop the gate, NextInt::reti syncs its
        # clock), so they go through nxSync and NEXTREG re-tests the gate.
        # Everything else: decodeED as before. fetch_nx may fall back to the
        # generic fetch (a raster line / MMU step), which can drop
        # g_next_fastrd: then no gate-free accessor may run.
        ED = {
            0x23: 'regA = (uint8_t)((regA << 4) | (regA >> 4));',
            0x27: '{ uint8_t value = NextMemory::peek8_nx(REG_PC); REG_PC++; carryFlag = false; '
                  'sz5h3pnFlags = sz53pn_addTable[regA & value] | HALFCARRY_MASK; flagQ = true; }',
            0x30: 'REG_DE = (uint16_t)(REG_D * REG_E);',
            0x31: 'REG_HL += regA;',
            0x32: 'REG_DE += regA;',
            0x33: 'REG_BC += regA;',
            0x4B: '{ REG_WZ = NextMemory::peek16_nx(REG_PC); REG_BC = NextMemory::peek16_nx(REG_WZ); REG_WZ++; REG_PC = REG_PC + 2; }',
            0x4D: '{ ffIFF1 = ffIFF2; REG_PC = REG_WZ = nxPop(); nxCheckTrdos(); '
                  'if (NextInt::ex) nxSync(NextInt::reti()); }',
            0x52: '{ NextMemory::addressOnBusIR_nx(7); sbc16(REG_DE); }',
            0x5B: '{ REG_WZ = NextMemory::peek16_nx(REG_PC); REG_DE = NextMemory::peek16_nx(REG_WZ++); REG_PC = REG_PC + 2; }',
            0x91: '{ uint8_t r = NextMemory::peek8_nx(REG_PC); REG_PC++; uint8_t v = NextMemory::peek8_nx(REG_PC); REG_PC++; '
                  'NextMemory::addressOnBusIR_nx(6); nxSync(NextReg::writeHot(r, v)); '
                  'if (__builtin_expect(!g_next_fastrd, 0)) { lastFlagQ = flagQ; return; } }',
            0x92: '{ uint8_t r = NextMemory::peek8_nx(REG_PC); REG_PC++; '
                  'NextMemory::addressOnBusIR_nx(6); nxSync(NextReg::writeHot(r, regA)); '
                  'if (__builtin_expect(!g_next_fastrd, 0)) { lastFlagQ = flagQ; return; } }',
        }
        inner = ' '.join('case 0x%02X: %s break;' % (k, v) for k, v in sorted(ED.items()))
        inner += (' case 0x34: case 0x35: case 0x36: { '
                  'uint16_t nn = NextMemory::peek8_nx(REG_PC); REG_PC++; '
                  'nn |= NextMemory::peek8_nx(REG_PC) << 8; REG_PC++; '
                  'NextMemory::addressOnBusIR_nx(2); '
                  'if (nx_op == 0x34) REG_HL += nn; else if (nx_op == 0x35) REG_DE += nn; else REG_BC += nn; '
                  'break; }')
        body = ('{ nx_op = NextOpcode::fetch_nx(nx_t); REG_PC++; nx_r++; '
                'if (__builtin_expect(!g_next_fastrd, 0)) { opCode = nx_op; regR = nx_r; decodeED(); lastFlagQ = flagQ; return; } '
                'switch (nx_op) { ' + inner +
                ' default: opCode = nx_op; nxSync(decodeED()); if (__builtin_expect(!g_next_fastrd, 0)) { lastFlagQ = flagQ; return; } break; '
                '} }')
    elif op in TABLE or n == 'decodeRST':
        # The only handlers that can drop g_next_fastrd (a port write): leave
        # the loop at once, so the gate-free bodies never run without it.
        body = ('opCode = nx_op; nxSync(dcOpcode[0x%02X]()); if (__builtin_expect(!g_next_fastrd, 0)) { lastFlagQ = flagQ; return; }' % op)
    elif n == 'decodeLD8':
        d, s = (op >> 3) & 7, op & 7
        if s == 6:   body = '%s = NextMemory::peek8_nx(REG_HL);' % R8[d]
        elif d == 6: body = 'NextMemory::poke8_nx(REG_HL, %s);' % R8[s]
        else:        body = '%s = %s;' % (R8[d], R8[s])
    elif n == 'decodeALU8':
        # The operand per case, the operation once per group (shared labels at
        # the end): eight inlined copies of add/adc/.../cp, not sixty-three.
        s = op & 7
        v = 'NextMemory::peek8_nx(REG_HL)' if s == 6 else R8[s]
        body = 'nx_v = %s; goto nx_alu%d;' % (v, (op >> 3) & 7)
    elif n == 'decodeINCDEC8':
        r = (op >> 3) & 7
        f = 'dec8' if op & 1 else 'inc8'
        if r == 6:
            body = ('{ uint8_t work8 = NextMemory::peek8_nx(REG_HL); %s(work8); '
                    'NextMemory::addressOnBus_nx(REG_HL, 1); NextMemory::poke8_nx(REG_HL, work8); }' % f)
        else:
            body = 'nx_p = &%s; goto nx_%s;' % (R8[r], f)
    elif n in ('decodeRETcc', 'decodeJPcc', 'decodeCALLcc'):
        # The condition per case (two or three instructions, where the
        # handler's condMet() on the stored opcode was a call), the family's
        # body once behind a shared label.
        body = 'nx_cc = %s; goto nx_%s;' % (CC[(op >> 3) & 7], n)
        inl.add(n)
    else:
        body = call(n)
        inl.add(n)
    # No break after an unconditional trailing goto (the shared ALU / INC / DEC
    # tails); a conditional one (the DD/FD prefix check) still needs it.
    tail_goto = re.search(r'(^|;\s*)goto \w+;$', body) and not body.rstrip().endswith(') goto nx_pfx;')
    out.append(nxt('case 0x%02X: %s%s' % (op, body, '' if tail_goto else ' break;')))
for k, f in enumerate(ALU):
    out.append(nxt('nx_alu%d: %s(nx_v); break;' % (k, f)))
out.append('nx_inc8: inc8(*nx_p); break;')
for n in ('decodeRETcc', 'decodeJPcc', 'decodeCALLcc'):
    out.append(nxt('nx_%s: %s break;' % (n, call(n))))
out.append('nx_dec8: dec8(*nx_p); break;')
open(os.path.join(ROOT, 'src/speccy/z80/next_dispatch.inc'), 'w').write('\n'.join(out) + '\n')
# The handlers called directly must carry NX_DISP_INL at their definition.
missing = [n for n in sorted(inl) if not re.search(r'NX_DISP_INL void Z80::%s\(' % n, src)]
if missing:
    print('handlers without NX_DISP_INL:', ' '.join(missing))
    sys.exit(1 if '--check' in sys.argv else 0)
