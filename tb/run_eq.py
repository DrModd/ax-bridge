#!/usr/bin/env python3
"""EQ block: RTL vs bit-exact model (incl. live bank swap and section
saturation), then precision of the fixed-point cascade vs float64."""
import os
import subprocess
import sys
import numpy as np
from scipy import signal

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
B = os.path.join(ROOT, "build")
os.makedirs(B, exist_ok=True)
sys.path.insert(0, os.path.join(ROOT, "tools"))
import eq_coefs as E

def run(cmd):
    r = subprocess.run(cmd, cwd=ROOT, capture_output=True, text=True)
    if r.returncode:
        print(r.stdout, r.stderr)
        raise SystemExit("command failed: " + " ".join(cmd))
    return r.stdout

# A demanding room-correction set: deep narrow LF cuts, 20 Hz boost,
# shelves, a subsonic high-pass and an ultrasonic low-pass.
SET_L = [E.rbj("HP", 15, 0, 0.707), E.rbj("PK", 20, 6, 3), E.rbj("PK", 38, -12, 8),
         E.rbj("PK", 55, -8, 6), E.rbj("PK", 87, 4, 5), E.rbj("LS", 120, 3, 0.7),
         E.rbj("PK", 230, -3, 2), E.rbj("PK", 1000, 2, 1), E.rbj("HS", 8000, -4, 0.7),
         E.rbj("LP", 40000, 0, 0.707)]
SET_R = [E.rbj("PK", 25, 8, 4), E.rbj("PK", 42, -10, 10), E.rbj("PK", 60, -6, 4),
         E.rbj("LS", 80, -3, 0.6), E.rbj("PK", 150, 3, 3), E.rbj("PK", 400, -2, 1.5),
         E.rbj("PK", 2500, 3, 2), E.rbj("HS", 12000, 2, 0.7), E.rbj("PK", 18000, -6, 3),
         E.rbj("HP", 10, 0, 0.5)]
SET_B_L = [E.rbj("PK", 30, 12, 2), E.rbj("PK", 3000, 6, 1)]
SET_B_R = [E.rbj("PK", 50, -6, 5), E.rbj("HS", 5000, 6, 0.7), E.rbj("PK", 100, 3, 1),
           E.rbj("LP", 20000, 0, 0.707)]

def bit_exact():
    vec, swap = 2000, 1000
    rng = np.random.default_rng(5)
    n = np.arange(vec)
    xl = (0.4 * np.sin(2 * np.pi * 37 * n / E.FS) + 0.2 * rng.standard_normal(vec)) * 2 ** 31
    xr = (0.3 * np.sin(2 * np.pi * 1234 * n / E.FS) + 0.1 * rng.standard_normal(vec)) * 2 ** 31
    # hot full-scale stretch after the swap: bank B boosts must saturate
    xl[1300:1500] = 0.95 * np.sign(np.sin(2 * np.pi * 3000 * n[1300:1500] / E.FS)) * 2 ** 31
    xr[1300:1500] = 0.9 * np.sin(2 * np.pi * 9000 * n[1300:1500] / E.FS) * 2 ** 31
    xl = np.clip(np.round(xl), -2 ** 31, 2 ** 31 - 1).astype(np.int64)
    xr = np.clip(np.round(xr), -2 ** 31, 2 ** 31 - 1).astype(np.int64)
    na, wa = E.words(SET_L, SET_R)
    nb, wb = E.words(SET_B_L, SET_B_R)
    wb = wb + [0x7FFFFFFF] * (100 - len(wb))       # junk beyond nsec: must be ignored
    with open(os.path.join(B, "eq_in.hex"), "w") as f:
        for a, b in zip(xl, xr):
            f.write("%08x%08x\n" % (int(a) & 0xFFFFFFFF, int(b) & 0xFFFFFFFF))
    for name, w in (("eq_wa.hex", wa), ("eq_wb.hex", wb)):
        with open(os.path.join(B, name), "w") as f:
            for v in w:
                f.write("%08x\n" % (v & 0xFFFFFFFF))
    exe = os.path.join(B, "eq.vvp")
    run(["iverilog", "-g2005", "-Wall", "-o", exe, "-s", "tb_eq",
         f"-Ptb_eq.VEC={vec}", f"-Ptb_eq.SWAP={swap}", f"-Ptb_eq.NA={na}", f"-Ptb_eq.NB={nb}",
         os.path.join(ROOT, "tb", "tb_eq.v"), os.path.join(ROOT, "rtl", "mem.v"),
         os.path.join(ROOT, "rtl", "eq_biquad.v")])
    log = run(["vvp", "-n", exe])
    print("  " + log.strip().splitlines()[-1])
    got = [tuple(map(int, l.split())) for l in open(os.path.join(B, "eq_out.txt"))]
    m = E.Model()
    l1, r1, c1 = m.run(xl[:swap], xr[:swap], na, wa)
    l2, r2, c2 = m.run(xl[swap:], xr[swap:], nb, wb)
    ref = list(zip(l1 + l2, r1 + r2))
    ok = got == ref
    if not ok:
        k = min(len(got), len(ref))
        bad = [i for i in range(k) if got[i] != ref[i]]
        print(f"  len got {len(got)} ref {len(ref)}, first mismatch {bad[:3]}",
              [(got[i], ref[i]) for i in bad[:3]])
    clips = int(log.split("clips=")[1].split()[0])
    busy = int(log.split("maxbusy=")[1].split()[0])
    ok &= (clips == c1 + c2) and c2 > 0 and busy < 250
    print(f"{'PASS' if ok else 'FAIL'} eq_biquad bit-exact: {len(got)} frames, live swap "
          f"{na}->{nb} sections, saturations {clips} (model {c1 + c2}), "
          f"busy {busy}/256 MCLK")
    return ok

def precision():
    """Fixed point vs float64 through SET_L / SET_R: error spectrum in
    20 Hz..20 kHz relative to full scale, and the response itself."""
    ok = True
    n = 1 << 17
    t = np.arange(n)
    rng = np.random.default_rng(9)
    tests = [("1 kHz -1 dBFS", 0.891 * np.sin(2 * np.pi * 997 * t / E.FS)),
             ("40 Hz -6 dBFS", 0.5 * np.sin(2 * np.pi * 40.3 * t / E.FS)),
             ("-60 dBFS 1 kHz", 1e-3 * np.sin(2 * np.pi * 997 * t / E.FS)),
             ("noise -20 dBFS", 0.1 * rng.standard_normal(n))]
    na, w = E.words(SET_L, SET_R)
    for name, x in tests:
        xi = np.round(x * 2 ** 31 * 0.5).astype(np.int64)     # -6 dB EQ headroom
        yl, yr, cl = E.Model().run(xi, xi, na, w)
        for ch, secs, y in (("L", SET_L, yl), ("R", SET_R, yr)):
            sos = np.array([[c[0], c[1], c[2], 1.0, c[3], c[4]] for c in
                            [[q / E.ONE for q in E.quantise(s)] for s in secs]])
            yf = signal.sosfilt(sos, xi.astype(float))
            err = (np.array(y, float) - yf)[n // 4:] / 2 ** 31
            win = np.hanning(len(err))
            S = np.abs(np.fft.rfft(err * win)) ** 2 / (win ** 2).sum()
            f = np.fft.rfftfreq(len(err), 1 / E.FS)
            band = (f > 20) & (f < 20000)
            e_db = 10 * np.log10(2 * S[band].sum() / len(err) * 1 + 1e-300)
            # full scale sine power = 0.5 -> dB re full scale
            e_db -= 10 * np.log10(0.5)
            c = e_db < -150 and cl == 0
            ok &= c
            print(f"{'PASS' if c else 'FAIL'} eq precision {ch} {name:15s}: "
                  f"fixed-point error {e_db:7.1f} dBFS (20 Hz-20 kHz)")
    # quantised coefficients vs ideal response. Q3.29 at 192 kHz: poles of
    # narrow LF sections sit ~1e-6 from z = 1; above 15 Hz the response
    # stays within 0.1 dB of the float design (the 10 Hz subsonic HP
    # deviates 0.15 dB at 10 Hz, outside the band that matters).
    worst = 0.0
    for secs in (SET_L, SET_R):
        q = [[v / E.ONE for v in E.quantise(s)] for s in secs]
        for f0 in np.geomspace(15, 40000, 400):
            d = abs(20 * np.log10(E.response(q, f0) / E.response(secs, f0)))
            worst = max(worst, d)
    c = worst < 0.1
    ok &= c
    print(f"{'PASS' if c else 'FAIL'} eq coefficient quantisation: worst response "
          f"deviation {worst:.5f} dB (15 Hz-40 kHz)")
    return ok

if __name__ == "__main__":
    ok = bit_exact()
    ok &= precision()
    print("PASS eq" if ok else "FAIL eq")
    sys.exit(0 if ok else 1)
