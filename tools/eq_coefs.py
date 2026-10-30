#!/usr/bin/env python3
"""EQ coefficients for the ax-bridge biquad block (rtl/eq_biquad.v).

Fixed output rate 192 kHz. Coefficients: signed 32 bit Q3.29, |c| < 4,
order b0 b1 b2 a1 a2 (a0 = 1), y = b0 x + b1 x1 + b2 x2 - a1 y1 - a2 y2.

  Design (RBJ cookbook) and print the I2C write sequence / a C table:
    eq_coefs.py --peq L:PK:45:-6:4 --peq R:PK:45:-5:4 --peq B:HS:10000:-2:0.7
        filter spec  CH:TYPE:FREQ:GAIN_DB:Q
        CH   L, R or B (both)
        TYPE PK peaking, LS / HS low / high shelf (Q = shelf slope S),
             LP / HP 2nd-order low / high pass (gain ignored)
  Load a REW "Generic" biquad export (a1/a2 with REW's sign convention:
  REW lists a1, a2 already negated -> use --rew-negated):
    eq_coefs.py --rew-l left.txt --rew-r right.txt

Also used by the benches: quantise(), model() (bit-exact RTL model).
"""
import argparse
import math
import re
import sys

FS = 192000.0
NS = 10
QB = 29
ONE = 1 << QB

def rbj(kind, f0, gain_db, q, fs=FS):
    A = 10 ** (gain_db / 40)
    w = 2 * math.pi * f0 / fs
    cw, sw = math.cos(w), math.sin(w)
    if kind in ("LS", "HS"):
        S = q
        alpha = sw / 2 * math.sqrt((A + 1 / A) * (1 / S - 1) + 2)
    else:
        alpha = sw / (2 * q)
    if kind == "PK":
        b = [1 + alpha * A, -2 * cw, 1 - alpha * A]
        a = [1 + alpha / A, -2 * cw, 1 - alpha / A]
    elif kind == "LS":
        r = 2 * math.sqrt(A) * alpha
        b = [A * ((A + 1) - (A - 1) * cw + r), 2 * A * ((A - 1) - (A + 1) * cw),
             A * ((A + 1) - (A - 1) * cw - r)]
        a = [(A + 1) + (A - 1) * cw + r, -2 * ((A - 1) + (A + 1) * cw),
             (A + 1) + (A - 1) * cw - r]
    elif kind == "HS":
        r = 2 * math.sqrt(A) * alpha
        b = [A * ((A + 1) + (A - 1) * cw + r), -2 * A * ((A - 1) + (A + 1) * cw),
             A * ((A + 1) + (A - 1) * cw - r)]
        a = [(A + 1) - (A - 1) * cw + r, 2 * ((A - 1) - (A + 1) * cw),
             (A + 1) - (A - 1) * cw - r]
    elif kind == "LP":
        b = [(1 - cw) / 2, 1 - cw, (1 - cw) / 2]
        a = [1 + alpha, -2 * cw, 1 - alpha]
    elif kind == "HP":
        b = [(1 + cw) / 2, -(1 + cw), (1 + cw) / 2]
        a = [1 + alpha, -2 * cw, 1 - alpha]
    else:
        raise ValueError("unknown filter type " + kind)
    a0 = a[0]
    return [b[0] / a0, b[1] / a0, b[2] / a0, a[1] / a0, a[2] / a0]

IDENT = [1.0, 0.0, 0.0, 0.0, 0.0]

def quantise(c):
    out = []
    for v in c:
        q = int(round(v * ONE))
        if not -(1 << 31) <= q < (1 << 31):
            raise ValueError(f"coefficient {v} outside +-4 (Q3.29)")
        out.append(q)
    return out

def words(sec_l, sec_r):
    """Word list for one bank: index sec*10 + ch*5 + k."""
    n = max(len(sec_l), len(sec_r))
    if n > NS:
        raise ValueError(f"at most {NS} sections per channel")
    w = []
    for s in range(n):
        for sec in (sec_l, sec_r):
            w += quantise(sec[s] if s < len(sec) else IDENT)
    return n, w

def i2c_bytes(n, w):
    """Register writes: (reg, [data bytes]) in order."""
    data = []
    for v in w:
        u = v & 0xFFFFFFFF
        data += [(u >> 24) & 255, (u >> 16) & 255, (u >> 8) & 255, u & 255]
    return [(0x0B, [0]), (0x0C, data), (0x0A, [n]), (0x09, [1])]

# ---------------------------------------------------------------- model
class Model:
    """Bit-exact model of eq_biquad. run() keeps the filter states, so a
    coefficient swap (apply) between calls behaves like the RTL."""
    def __init__(self):
        self.st = [[[0, 0, 0, 0, 0, 0] for _ in range(2)] for _ in range(NS)]

    def run(self, xl, xr, n, w):
        def sat(v):
            return max(-(1 << 31), min((1 << 31) - 1, v))
        st = self.st
        yl, yr, clips = [], [], 0
        for a, b in zip(xl, xr):
            cur = [int(a), int(b)]
            for s in range(NS):
                for ch in range(2):
                    x1, x2, y1, y2, e1, e2 = st[s][ch]
                    x0 = cur[ch]
                    if s >= n:
                        y, e = x0, 0
                    else:
                        c = w[(s * 2 + ch) * 5:(s * 2 + ch) * 5 + 5]
                        acc = (2 * e1 - e2) + c[0] * x0 + c[1] * x1 + c[2] * x2 \
                            - c[3] * y1 - c[4] * y2
                        yq = acc >> QB
                        y = sat(yq)
                        clips += (y != yq)
                        e = 0 if y != yq else acc & (ONE - 1)
                    st[s][ch] = [x0, x1, y, y1, e, e1]
                    cur[ch] = y
            yl.append(cur[0]); yr.append(cur[1])
        return yl, yr, clips

def model(xl, xr, n, w):
    yl, yr, _ = Model().run(xl, xr, n, w)
    return yl, yr

def response(secs, f, fs=FS):
    h = 1.0
    for c in secs:
        z = complex(math.cos(2 * math.pi * f / fs), -math.sin(2 * math.pi * f / fs))
        h *= (c[0] + c[1] * z + c[2] * z * z) / (1 + c[3] * z + c[4] * z * z)
    return abs(h)

# ---------------------------------------------------------------- CLI
def parse_rew(path, negated):
    secs = []
    cur = {}
    for line in open(path):
        m = re.match(r"\s*(b0|b1|b2|a1|a2)\s*=\s*([-+0-9.eE]+)", line)
        if m:
            cur[m.group(1)] = float(m.group(2))
            if len(cur) == 5:
                a1, a2 = cur["a1"], cur["a2"]
                if negated:
                    a1, a2 = -a1, -a2
                secs.append([cur["b0"], cur["b1"], cur["b2"], a1, a2])
                cur = {}
    return secs

def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--peq", action="append", default=[])
    ap.add_argument("--rew-l")
    ap.add_argument("--rew-r")
    ap.add_argument("--rew-negated", action="store_true")
    ap.add_argument("--c", action="store_true", help="print a C array for the STM32")
    a = ap.parse_args()
    L, R = [], []
    for spec in a.peq:
        ch, kind, f, g, q = spec.split(":")
        c = rbj(kind.upper(), float(f), float(g), float(q))
        if ch.upper() in ("L", "B"): L.append(c)
        if ch.upper() in ("R", "B"): R.append(c)
    if a.rew_l: L += parse_rew(a.rew_l, a.rew_negated)
    if a.rew_r: R += parse_rew(a.rew_r, a.rew_negated)
    n, w = words(L, R)
    for name, secs in (("L", L), ("R", R)):
        pts = [20, 31.5, 45, 63, 100, 200, 500, 1000, 3000, 10000, 20000]
        print(f"# {name}: " + "  ".join(f"{p:g}Hz {20*math.log10(response(secs, p)):+.2f}"
                                       for p in pts), file=sys.stderr)
    if a.c:
        print(f"/* ax-bridge EQ: {n} sections, Q3.29, index sec*10+ch*5+k */")
        print(f"static const uint8_t eq_nsec = {n};")
        print(f"static const int32_t eq_coef[{len(w)}] = {{")
        for i in range(0, len(w), 5):
            print("    " + ", ".join(f"{v:d}" for v in w[i:i + 5]) + ",")
        print("};")
    else:
        for reg, data in i2c_bytes(n, w):
            print(f"reg 0x{reg:02X}: " + " ".join(f"{b:02X}" for b in data))

if __name__ == "__main__":
    try:
        main()
    except ValueError as e:
        sys.exit(f"error: {e} - split a big boost into two sections or lower its gain")
