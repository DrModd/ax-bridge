#!/usr/bin/env python3
"""Audio check of jitter_fifo: sine through the buffer with a clock offset.
THD+N (20 Hz .. 20 kHz) is measured separately on stretches of bit-exact
playback and of interpolation (flag_low / flag_high), each long enough for
an FFT. Pass: interpolated stretches within the table's error budget."""
import os
import subprocess
import sys
import numpy as np

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
B = os.path.join(ROOT, "build")

def thdn(y, fs, f0):
    n = len(y)
    w = np.kaiser(n, 38.0)
    Y = np.abs(np.fft.rfft(y * w)) ** 2
    f = np.fft.rfftfreq(n, 1 / fs)
    k = int(np.argmax(Y[1:])) + 1
    sig = Y[max(k - 30, 1):k + 31].sum()
    band = (f > 20) & (f < 20000)
    band[max(k - 30, 1):k + 31] = False
    return 10 * np.log10(Y[band].sum() / sig), f[k]

def run(name, fs, ppm, f0, aw, step_sh, nt, mph, coef, n_out, limit):
    exe = os.path.join(B, "jf_audio.vvp")
    os.makedirs(B, exist_ok=True)
    cmd = ["iverilog", "-g2005", "-Wall", "-o", exe, "-s", "tb_jf_audio",
           f"-Ptb_jf_audio.FS={fs}", f"-Ptb_jf_audio.PPM={ppm}", f"-Ptb_jf_audio.F0={f0}",
           f"-Ptb_jf_audio.AW={aw}", f"-Ptb_jf_audio.STEP_SH={step_sh}",
           f"-Ptb_jf_audio.NT={nt}", f"-Ptb_jf_audio.MPH_SH={mph}",
           f'-Ptb_jf_audio.COEF_FILE="{coef}"', f"-Ptb_jf_audio.N_OUT={n_out}",
           "jitter_fifo/rtl/jitter_fifo.v", "jitter_fifo/tb/tb_jf_audio.v"]
    r = subprocess.run(cmd, cwd=ROOT, capture_output=True, text=True)
    if r.returncode:
        print(r.stdout, r.stderr)
        return False
    r = subprocess.run(["vvp", "-n", exe], cwd=ROOT, capture_output=True, text=True)
    done = [l for l in r.stdout.splitlines() if l.startswith("DONE")]
    log = done[-1] if done else "no DONE line"
    d = np.loadtxt(os.path.join(B, "jf_audio.txt"), ndmin=2)
    interp = (d[:, 2] + d[:, 3]) > 0
    # stretches of constant mode, longest of each kind, trimmed by NT
    best = {False: None, True: None}
    i = 0
    while i < len(d):
        j = i
        while j < len(d) and interp[j] == interp[i]:
            j += 1
        a, b = i + nt + 4, j - nt - 4
        if b - a > 2048 and (best[interp[i]] is None or b - a > best[interp[i]][1] - best[interp[i]][0]):
            best[interp[i]] = (a, b)
        i = j
    ok = "xrun 0 late 0" in log
    res = []
    for mode in (False, True):
        if best[mode] is None:
            res.append(f"{'interp' if mode else 'exact'}: no stretch")
            ok = False
            continue
        a, b = best[mode]
        n = min(b - a, 16384)
        y = d[b - n:b, 0] / 2.0 ** 31
        t, fpk = thdn(y, fs, f0)
        ok &= t < limit
        res.append(f"{'interp' if mode else 'exact '}: {n:5d} samples THD+N {t:7.1f} dB (f {fpk:8.2f})")
    share = 100.0 * interp.mean()
    print(f"{'PASS' if ok else 'FAIL'} {name}: interpolating {share:4.1f} % of the time | "
          + " | ".join(res) + f" | {log}")
    return ok

if __name__ == "__main__":
    LS64 = "jitter_fifo/rtl/fir_ls64_m128.hex"
    CR4 = "jitter_fifo/rtl/fir_cr4.hex"
    ok = True
    ok &= run("LS64 192k +2500ppm 1k ", 192000.0, 2500.0, 997.0, 8, 8, 64, 7, LS64, 70000, -125)
    ok &= run("LS64 192k -2500ppm 15k", 192000.0, -2500.0, 15013.0, 8, 8, 64, 7, LS64, 70000, -105)
    ok &= run("LS64  48k +2500ppm 10k", 48000.0, 2500.0, 10007.0, 8, 8, 64, 7, LS64, 60000, -90)
    ok &= run("CR4  192k +2500ppm 1k ", 192000.0, 2500.0, 997.0, 6, 8, 4, 10, CR4, 40000, -110)
    print("PASS jf_audio" if ok else "FAIL jf_audio")
    sys.exit(0 if ok else 1)
