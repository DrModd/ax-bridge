#!/usr/bin/env python3
"""Unit benches with bit-exact golden models: fir_stage (all 5 configs)
and cic5_dsd (all R). Exit code != 0 on any mismatch."""
import os
import subprocess
import sys
import numpy as np

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
B = os.path.join(ROOT, "build")
os.makedirs(B, exist_ok=True)
RTL = [os.path.join(ROOT, "rtl", f) for f in ("mem.v", "fir_stage.v", "cic5_dsd.v")]

def run(cmd):
    r = subprocess.run(cmd, cwd=ROOT, capture_output=True, text=True)
    if r.returncode:
        print(r.stdout, r.stderr)
        raise SystemExit("command failed: " + " ".join(cmd))
    return r.stdout

def s24(v):
    return int(v) & 0xFFFFFF

def load_coef(name):
    c = []
    for line in open(os.path.join(ROOT, "coef", name + ".hex")):
        v = int(line, 16)
        if v & (1 << 24):
            v -= 1 << 25
        c.append(v)
    return c

def sat24(v):
    return max(-(1 << 23), min((1 << 23) - 1, v))

def fir_ref(x, h, interp):
    N = len(h)
    out = []
    hist = []
    for n, s in enumerate(x):
        hist.insert(0, s)
        def mac(taps):
            acc = 0
            for t, k in enumerate(taps):
                xv = hist[t] if t < len(hist) else 0
                acc += h[k] * xv
            return sat24((acc + (1 << 22)) >> 23)
        if interp:
            out.append(mac(range(0, N, 2)))
            out.append(mac(range(1, N, 2)))
        elif n % 2 == 1:
            out.append(mac(range(N)))
    return out

def test_fir(name, N, interp, aw, caw, gap, vec=300, seed=0):
    rng = np.random.default_rng(seed)
    L = rng.integers(-(1 << 23), 1 << 23, vec)
    R = rng.integers(-(1 << 23), 1 << 23, vec)
    # full-scale stretches to hit saturation
    L[100:140] = (1 << 23) - 1
    R[100:140] = -(1 << 23)
    L[200:230:2] = (1 << 23) - 1
    L[201:230:2] = -(1 << 23)
    with open(os.path.join(B, "fir_in.hex"), "w") as f:
        for a, b in zip(L, R):
            f.write("%06x%06x\n" % (s24(a), s24(b)))
    exe = os.path.join(B, "fir.vvp")
    run(["iverilog", "-g2005", "-o", exe, "-s", "tb_fir",
         f"-Ptb_fir.N={N}", f"-Ptb_fir.INTERP={interp}", f"-Ptb_fir.AW={aw}",
         f"-Ptb_fir.CAW={caw}", f'-Ptb_fir.FILE="coef/{name}.hex"',
         f"-Ptb_fir.VEC={vec}", f"-Ptb_fir.GAP={gap}",
         os.path.join(ROOT, "tb", "tb_fir.v")] + RTL)
    log = run(["vvp", "-n", exe])
    if "FAIL" in log:
        print(log)
        return False
    got = [tuple(map(int, l.split())) for l in open(os.path.join(B, "fir_out.txt"))]
    h = load_coef(name)
    rl = fir_ref(list(L), h, interp)
    rr = fir_ref(list(R), h, interp)
    ref = list(zip(rl, rr))
    ok = (got == ref)
    if not ok:
        n = min(len(got), len(ref))
        bad = [i for i in range(n) if got[i] != ref[i]]
        print(f"  {name}: len got {len(got)} ref {len(ref)}; first mismatch "
              f"{bad[:3]} {[(got[i], ref[i]) for i in bad[:3]]}")
    print(f"{'PASS' if ok else 'FAIL'} fir_stage {name} ({len(got)} outputs)")
    return ok

def cic_ref(bl, br, rlog2):
    R = 1 << rlog2
    out = []
    def chan(bits):
        i = [0] * 5
        d = [0] * 5
        res = []
        for n, b in enumerate(bits):
            x = 1 if b else -1
            # pipelined integrators: each stage adds the previous value
            # of the stage before it
            prev = i[:]
            i[0] = prev[0] + x
            for k in range(1, 5):
                i[k] = prev[k] + prev[k - 1]
            if n % R == R - 1:
                c = i[4]
                cs = []
                for k in range(5):
                    nc = c - d[k]
                    d[k] = c
                    c = nc
                sh = 23 - 5 * rlog2
                v = c << sh if sh >= 0 else c >> (-sh)
                res.append(sat24(v))
        return res
    return list(zip(chan(bl), chan(br)))

def test_cic(rlog2, chunks=600, gap=20, seed=1):
    rng = np.random.default_rng(seed)
    # biased random bitstreams (DC +0.3 / -0.6) plus a long all-ones run
    bl = (rng.random(chunks * 16) < 0.65).astype(int)
    br = (rng.random(chunks * 16) < 0.20).astype(int)
    bl[3000:5000] = 1
    with open(os.path.join(B, "cic_in.hex"), "w") as f:
        for c in range(chunks):
            wl = int("".join(map(str, bl[c * 16:(c + 1) * 16])), 2)
            wr = int("".join(map(str, br[c * 16:(c + 1) * 16])), 2)
            f.write("%04x%04x\n" % (wl, wr))
    exe = os.path.join(B, "cic.vvp")
    run(["iverilog", "-g2005", "-o", exe, "-s", "tb_cic",
         f"-Ptb_cic.RLOG2={rlog2}", f"-Ptb_cic.VEC={chunks}", f"-Ptb_cic.GAP={gap}",
         os.path.join(ROOT, "tb", "tb_cic.v")] + RTL)
    log = run(["vvp", "-n", exe])
    got = [tuple(map(int, l.split())) for l in open(os.path.join(B, "cic_out.txt"))]
    ref = cic_ref(bl, br, rlog2)
    ok = (got == ref) and "FAIL" not in log
    if not ok:
        print(log)
        n = min(len(got), len(ref))
        bad = [i for i in range(n) if got[i] != ref[i]]
        print(f"  len got {len(got)} ref {len(ref)} first bad {bad[:3]}",
              [(got[i], ref[i]) for i in bad[:3]])
    print(f"{'PASS' if ok else 'FAIL'} cic5_dsd R={1 << rlog2} ({len(got)} outputs)")
    return ok

if __name__ == "__main__":
    ok = True
    ok &= test_fir("hd_b", 31, 0, 5, 5, 80)
    ok &= test_fir("hd_a", 23, 0, 5, 5, 40)
    ok &= test_fir("dsd", 95, 0, 7, 7, 120)
    ok &= test_fir("hb_b", 91, 1, 6, 7, 220)
    ok &= test_fir("hb_a", 175, 1, 7, 8, 400)
    for r in (3, 4, 5, 6):
        ok &= test_cic(r)
    print("PASS units" if ok else "FAIL units")
    sys.exit(0 if ok else 1)
