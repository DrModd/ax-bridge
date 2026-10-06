#!/usr/bin/env python3
"""System benches: every input format through top.v, output decoded from
the I2S pins, then frequency / level / THD+N checked in Python.

usage: run_sys.py [case-name ...]     (no args = all cases)
"""
import os
import subprocess
import sys
import time
import numpy as np

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
B = os.path.join(ROOT, "build")
os.makedirs(B, exist_ok=True)
RTL = sorted(os.path.join(ROOT, "rtl", f) for f in os.listdir(os.path.join(ROOT, "rtl"))
             if f.endswith(".v"))
FOUT = 192000.0

# ---------------------------------------------------------------- DSD source
def ntf_coeffs(order=5, hinf=1.5):
    """Highpass NTF (1-z^-1)^order / A(z) with max gain ~hinf, leading 1."""
    from scipy import signal
    lo, hi = 0.01, 0.45
    for _ in range(60):
        wc = (lo + hi) / 2
        b, a = signal.butter(order, wc, "high")
        b = b / b[0]
        w, H = signal.freqz(b, a, 4096)
        if np.abs(H).max() > hinf:
            hi = wc
        else:
            lo = wc
    b, a = signal.butter(order, lo, "high")
    return b / b[0], a

def make_dsd(fbit, nbits, fl, fr, amp=0.5):
    """5th-order 1-bit sigma-delta (error-feedback NTF), both channels."""
    b, a = ntf_coeffs()
    f = b - a                       # NTF - 1 = (B-A)/A, f[0] == 0
    t = np.arange(nbits) / fbit
    out = []
    for f0 in (fl, fr):
        u = amp * np.sin(2 * np.pi * f0 * t)
        e_hist = np.zeros(len(a))
        w_hist = np.zeros(len(a))
        y = np.empty(nbits, dtype=np.uint8)
        order = len(a) - 1
        fb = f[1:]
        aa = a[1:]
        eh = [0.0] * order
        wh = [0.0] * order
        for n in range(nbits):
            # w[n] = sum f[k] e[n-k] - sum a[k] w[n-k]
            wn = 0.0
            for k in range(order):
                wn += fb[k] * eh[k] - aa[k] * wh[k]
            v = u[n] + wn
            yb = 1.0 if v >= 0 else -1.0
            en = yb - v
            eh = [en] + eh[:-1]
            wh = [wn] + wh[:-1]
            y[n] = 1 if yb > 0 else 0
        out.append(y)
    return out

def write_dsd_hex(bl, br, path):
    n = len(bl) // 16
    with open(path, "w") as f:
        for i in range(n):
            wl = int("".join(map(str, bl[i * 16:(i + 1) * 16])), 2)
            wr = int("".join(map(str, br[i * 16:(i + 1) * 16])), 2)
            f.write("%04x%04x\n" % (wl, wr))
    return n

# ---------------------------------------------------------------- analysis
def analyse(y, f_exp, fs=FOUT):
    n = len(y)
    w = np.kaiser(n, 38.0)
    Y = np.abs(np.fft.rfft(y * w)) ** 2
    f = np.fft.rfftfreq(n, 1 / fs)
    k = int(np.argmax(Y[1:])) + 1
    # parabolic interpolation on log magnitude
    lm = np.log(Y[k - 1:k + 2] + 1e-300)
    d = 0.5 * (lm[0] - lm[2]) / (lm[0] - 2 * lm[1] + lm[2])
    fpk = (k + d) * fs / n
    win = 30
    sig = Y[max(k - win, 1):k + win + 1].sum()
    band = (f > 20) & (f < 20000)
    band[max(k - win, 1):k + win + 1] = False
    noise = Y[band].sum()
    amp = np.sqrt(4 * sig / (n * (w ** 2).sum()))     # sine peak amplitude
    return fpk, 20 * np.log10(amp + 1e-30), 10 * np.log10(noise / sig)

CASES = {
    #  name        mode  rate        ppm     fl      fr     amp(dBFS expected)
    "pcm44":   (0, 44100.0,    37.0,  997.0, 3001.0, -1.0),
    "pcm48":   (0, 48000.0,   -23.0,  997.0, 3001.0, -1.0),
    "pcm88":   (0, 88200.0,    11.0, 1499.0, 15013.0, -1.0),
    "pcm96":   (0, 96000.0,    52.0,  997.0, 7003.0, -1.0),
    "pcm176":  (0, 176400.0,  -41.0,  997.0, 3001.0, -1.0),
    "pcm192":  (0, 192000.0,   13.0, 1999.0, 17011.0, -1.0),
    "pcm352":  (0, 352800.0,   29.0,  997.0, 3001.0, -1.0),
    "pcm384":  (0, 384000.0,  -17.0,  997.0, 3001.0, -1.0),
    "pcm705":  (0, 705600.0,    7.0,  997.0, 3001.0, -1.0),
    "pcm768":  (0, 768000.0,   -9.0,  997.0, 11003.0, -1.0),
    "dsd64":   (1, 2822400.0,  31.0,  997.0, 3001.0, -6.02),
    "dsd128":  (1, 5644800.0, -27.0,  997.0, 3001.0, -6.02),
    "dsd256":  (1, 11289600.0, 19.0,  997.0, 3001.0, -6.02),
    "dsd512":  (1, 22579200.0, -8.0,  997.0, 3001.0, -6.02),
    "dop64":   (2, 176400.0,   21.0,  997.0, 3001.0, -6.02),
    "dop128":  (2, 352800.0,  -33.0,  997.0, 3001.0, -6.02),
    "dop256":  (2, 705600.0,   15.0,  997.0, 3001.0, -6.02),
}

def run_case(name, nout=8192, thdn_limit=-110.0, extra=()):
    mode, rate, ppm, fl, fr, lvl = CASES[name]
    params = [f"-Ptb_sys.MODE={mode}", f"-Ptb_sys.FS={rate}", f"-Ptb_sys.PPM={ppm}",
              f"-Ptb_sys.FL={fl}", f"-Ptb_sys.FR={fr}", f"-Ptb_sys.NOUT={nout}"] + list(extra)
    if mode != 0:
        fbit = rate if mode == 1 else rate * 16
        secs = 0.20
        nb = int(fbit * secs) // 16 * 16
        hexp = os.path.join(B, "dsd.hex")
        cache = os.path.join(B, f"dsd_{int(fbit)}.hex")
        if not os.path.exists(cache):
            t0 = time.time()
            bl, br = make_dsd(fbit, nb, fl, fr)
            write_dsd_hex(bl, br, cache)
            print(f"  generated DSD {fbit/1e6:.4f} MHz in {time.time()-t0:.0f}s")
        n = sum(1 for _ in open(cache))
        import shutil
        shutil.copy(cache, hexp)
        params.append(f"-Ptb_sys.DSDN={n}")
    exe = os.path.join(B, "sys.vvp")
    r = subprocess.run(["iverilog", "-g2005", "-o", exe, "-s", "tb_sys"] + params +
                       [os.path.join(ROOT, "tb", "tb_sys.v")] + RTL,
                       cwd=ROOT, capture_output=True, text=True)
    if r.returncode:
        print(r.stdout, r.stderr)
        return False
    t0 = time.time()
    r = subprocess.run(["vvp", "-n", exe], cwd=ROOT, capture_output=True, text=True)
    dt = time.time() - t0
    log = r.stdout
    data = np.loadtxt(os.path.join(B, "sys_out.txt"), ndmin=2)
    if len(data) < nout:
        print(log)
        print(f"FAIL {name}: only {len(data)} samples captured ({dt:.0f}s)")
        return False
    y = data[-nout:] / 2.0 ** 31
    ok = True
    res = []
    for ch, f0 in ((0, fl), (1, fr)):
        fexp = f0 * (1 + ppm * 1e-6)
        fpk, amp, thdn = analyse(y[:, ch], fexp)
        cok = abs(fpk - fexp) < 0.5 and abs(amp - lvl) < 0.1 and thdn < thdn_limit
        ok &= cok
        res.append(f"{'LR'[ch]}: f {fpk:9.3f} (exp {fexp:9.3f}) lvl {amp:6.2f} dBFS "
                   f"THD+N {thdn:7.1f} dB")
    on = [l for l in log.splitlines() if "audio_on" in l or "audio_off" in l]
    print(f"{'PASS' if ok else 'FAIL'} {name:7s} | " + " | ".join(res) +
          f" | {on[0] if on else 'never on'}; {len([l for l in on if 'off' in l])} drop(s), {dt:.0f}s")
    if any("audio_off" in l for l in on):
        ok = False
    return ok

def run_switch():
    import shutil
    cache = os.path.join(B, "dsd_2822400.hex")
    if not os.path.exists(cache):
        nb = int(2822400 * 0.2) // 16 * 16
        bl, br = make_dsd(2822400.0, nb, 997.0, 3001.0)
        write_dsd_hex(bl, br, cache)
    shutil.copy(cache, os.path.join(B, "dsd.hex"))
    n = sum(1 for _ in open(cache))
    exe = os.path.join(B, "switch.vvp")
    r = subprocess.run(["iverilog", "-g2005", "-o", exe, "-s", "tb_switch",
                        f"-Ptb_switch.DSDN={n}", os.path.join(ROOT, "tb", "tb_switch.v")] + RTL,
                       cwd=ROOT, capture_output=True, text=True)
    if r.returncode:
        print(r.stdout, r.stderr)
        return False
    r = subprocess.run(["vvp", "-n", exe], cwd=ROOT, capture_output=True, text=True)
    print(r.stdout)
    return "PASS tb_switch" in r.stdout

if __name__ == "__main__":
    if sys.argv[1:2] == ["--switch"]:
        sys.exit(0 if run_switch() else 1)
    if sys.argv[1:2] == ["--silicon"]:
        # real rate_mon window and real DPLL gear lengths (slow: ~30 min)
        ok = run_case(sys.argv[2] if len(sys.argv) > 2 else "pcm44",
                      extra=["-Ptb_sys.TB_WBITS=18", "-Ptb_sys.TB_GSH=0",
                             "-Ptb_sys.TMAX_MS=800.0"])
        sys.exit(0 if ok else 1)
    names = sys.argv[1:] or list(CASES)
    ok = True
    for n in names:
        lim = -110.0 if CASES[n][0] == 0 else -100.0
        ok &= run_case(n, thdn_limit=lim)
    print("PASS system" if ok else "FAIL system")
    sys.exit(0 if ok else 1)
