#!/usr/bin/env python3
"""Offline regression for the LBJ/POCSAG decoder.

The reference recording is the real LBJ short block that SFMON decodes as
1234000 / "42138 018 03447".  This script mirrors the C decoder's BCH(31,21)
two-bit correction, sync tolerance and 0..31 bit-shift search, then checks
that the first codeword comes back as 4B515A86.

Usage:
    python tools/lbj_wav_test.py [recording.wav]

The WAV is expected to be a 1200-baud POCSAG discriminator/audio recording.
It is resampled to 19.2 kHz, DC-blocked and sliced at every bit phase; the
best phase/shift is the one with the most LBJ-shaped codewords.  This is a
decoder test, not an RF test: it proves the 2-bit BCH and the shift search,
not the BK4819 front end.
"""
import sys
import wave

import numpy as np

FS_ADC = 19200
BAUD = 1200
SYNC = 0x7CD215D8
SYNC_INV = (~SYNC) & 0xFFFFFFFF
IDLE = 0x7A89C197
LBJ_MIN, LBJ_MAX = 1233980, 1234020

# 1234000 / func 3, "42138 018 03447" (SFMON's decode)
EXPECT_ADDR = 0x4B515A86
EXPECT_MSG = [0x92460E68, 0x98409B43, 0x86117151]


def pc32(x):
    return bin(x & 0xFFFFFFFF).count("1")


def bch_syn(d31):
    r = 0
    for i in range(30, -1, -1):
        fb = (r >> 9) & 1
        r = ((r << 1) | ((d31 >> i) & 1)) & 0x3FF
        if fb:
            r ^= 873
    return r


BCH_SYN1 = [bch_syn(1 << p) for p in range(31)]


def bch_dec(cw):
    """Return (corrected, status): 0 clean, 1 repaired, 2 uncorrectable."""
    d31 = (cw >> 1) & 0x7FFFFFFF
    s = bch_syn(d31)

    if not s:
        if pc32(cw) & 1:
            return cw ^ 1, 1
        return cw, 0

    for p in range(31):
        if BCH_SYN1[p] != s:
            continue
        c = ((d31 ^ (1 << p)) << 1) | (cw & 1)
        if (pc32(c) & 1) == 0:
            return c, 1
        return c ^ 1, 1

    for p in range(31):
        for q in range(p + 1, 31):
            if (BCH_SYN1[p] ^ BCH_SYN1[q]) != s:
                continue
            c = ((d31 ^ (1 << p) ^ (1 << q)) << 1) | (cw & 1)
            if (pc32(c) & 1) == 0:
                return c, 1
            return cw, 2

    return cw, 2


def load_wav(path):
    w = wave.open(path, "rb")
    fs = w.getframerate()
    ch = w.getnchannels()
    n = w.getnframes()
    sw = w.getsampwidth()
    raw = w.readframes(n)
    w.close()
    if sw != 2:
        sys.exit("only 16-bit PCM is supported (got %d bytes/sample)" % sw)
    a = np.frombuffer(raw, dtype="<i2").astype(float)
    if ch > 1:
        a = a[::ch]
    # linear resample to 19.2 kHz
    t = np.arange(0, len(a) - 1, fs / FS_ADC)
    x = np.interp(t, np.arange(len(a)), a)
    # 20 Hz DC block
    k = max(1, int(FS_ADC / 20))
    x = x - np.convolve(x, np.ones(k) / k, mode="same")
    return x


def bits_at(x, phase):
    idx = (phase + np.arange(int((len(x) - phase) / 16)) * 16).astype(int)
    return (x[idx] > 0).astype(int)


def find_sync(bits):
    sr = 0
    for i, b in enumerate(bits):
        sr = ((sr << 1) | int(b)) & 0xFFFFFFFF
        if i < 31:
            continue
        if pc32(sr ^ SYNC) <= 3:
            return i - 31, False
        if pc32(sr ^ SYNC_INV) <= 3:
            return i - 31, True
    return None, False


def word_at(bits, bit, inv):
    cw = 0
    for b in range(32):
        i = bit + b
        cw = (cw << 1) | (int(bits[i]) if i < len(bits) else 0)
    if inv:
        cw = (~cw) & 0xFFFFFFFF
    return cw


ADDR_CW = [
    0x4B5122C4, 0x4B512C17, 0x4B5131B2, 0x4B513F61,
    0x4B514723, 0x4B5149F0, 0x4B515455, 0x4B515A86,
]


def measure_addr(bits, inv):
    """Minimum Hamming distance to a known LBJ address codeword over all
    shifts and all 16 codeword positions.  Returns (distance, shift)."""
    best_d = 32
    best_sh = 0
    for sh in range(32):
        for j in range(16):
            cw = word_at(bits, sh + j * 32, inv)
            for ref in ADDR_CW:
                d = pc32(cw ^ ref)
                if d < best_d:
                    best_d = d
                    best_sh = sh
                    if d == 0:
                        return best_d, best_sh
    return best_d, best_sh


def decode_bcd(cws):
    table = "0123456789*U -)("
    out = []
    for cw in cws:
        if not cw:
            out.append("?????")
            continue
        d20 = (cw >> 11) & 0xFFFFF
        for n in range(5):
            v = (d20 >> (16 - n * 4)) & 0xF
            rev = ((v & 1) << 3) | ((v & 2) << 1) | ((v & 4) >> 1) | ((v & 8) >> 3)
            out.append(table[rev])
    return "".join(out)


def main():
    path = sys.argv[1] if len(sys.argv) > 1 else "设备接收pocsag音频录音.wav"
    try:
        x = load_wav(path)
    except FileNotFoundError:
        sys.exit("WAV not found: %s" % path)

    best = None
    for phase in range(16):
        bits = bits_at(x, phase)
        pos, inv = find_sync(bits)
        if pos is None:
            continue
        payload = bits[pos + 32:]
        d, sh = measure_addr(payload, inv)
        if best is None or d < best[0]:
            best = (d, phase, pos, sh, inv, payload)

    if best is None:
        print("FAIL: no POCSAG sync found")
        return 1

    dist, phase, pos, sh, inv, payload = best
    print("sync: phase=%d bit=%d inverted=%s shift=%d D=%d" %
          (phase, pos, inv, sh, dist))
    if dist > 2:
        print("address not within 2 bits; decoding at shift 0")
        sh = 0

    words = []
    for j in range(16):
        cw = word_at(payload, sh + j * 32, inv)
        cor, st = bch_dec(cw)
        words.append(cor)
        print("  [%2d] %08X -> %08X %s" %
              (j, cw, cor, ("clean" if st == 0 else
                            "repaired" if st == 1 else "BAD")))

    addr = None
    for j, cor in enumerate(words):
        if not (cor >> 31):
            addr = ((cor >> 13) & 0x3FFFF) * 8 + (j >> 1)
            func = (cor >> 11) & 3
            print("address: %d func=%d" % (addr, func))
            break

    print("message: %r" % decode_bcd(words[1:]))
    ok = words[0] == EXPECT_ADDR
    print("first codeword: %s (expected %08X)" %
          ("PASS" if ok else "FAIL", EXPECT_ADDR))
    if not ok:
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
