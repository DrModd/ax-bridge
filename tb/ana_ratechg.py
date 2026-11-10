#!/usr/bin/env python3
"""Analyse build/ratechg_out.txt (tb_ratechg): per stretch of audio_on,
the largest high-pass residual (4th difference, dBFS) and the pitch.
A clean -1 dBFS 997 / 3001 Hz sine has a 4th difference below -75 dBFS;
a slipped, repeated or corrupted sample shows up far above it."""
import sys
import numpy as np

B = sys.argv[1] if len(sys.argv) > 1 else "build"
d = np.loadtxt(f"{B}/ratechg_out.txt")
ev = np.loadtxt(f"{B}/ratechg_ev.txt", ndmin=2)
t, on, l, r = d[:, 0], d[:, 1] > 0, d[:, 2] / 2.0 ** 31, d[:, 3] / 2.0 ** 31
LIM = -65.0
ok = True

def rate_at(tt):
    k = np.searchsorted(ev[:, 0], tt, side="right") - 1
    return ev[max(k, 0), 1]

edges = np.flatnonzero(np.diff(np.r_[0, on.astype(int), 0]))
runs = list(zip(edges[::2], edges[1::2]))
print(f"{len(runs)} stretches of audio_on")
for a, b in runs:
    n = b - a
    seg_t = t[a:b]
    fs_out = 1e6 / np.median(np.diff(seg_t)) if n > 10 else 0
    worst = -200.0
    wt = 0.0
    for x in (l[a:b], r[a:b]):
        if n < 16:
            continue
        h = np.abs(np.diff(x, 4))
        k = int(np.argmax(h))
        v = 20 * np.log10(h[k] + 1e-12)
        if v > worst:
            worst, wt = v, seg_t[k]
    pitch = 0.0
    if n > 4096:
        y = l[b - 4096:b] * np.hanning(4096)
        Y = np.abs(np.fft.rfft(y, 65536))
        pitch = np.argmax(Y) * fs_out / 65536
    peak = 20 * np.log10(np.max(np.abs(l[a:b])) + 1e-12)
    bad = worst > LIM or (pitch and abs(pitch - 997.0) > 3.0)
    ok &= not bad
    print(f"{'BAD ' if bad else 'ok  '} {seg_t[0] / 1000:8.2f}..{seg_t[-1] / 1000:8.2f} ms  src {rate_at(seg_t[0]):6.0f} -> "
          f"out {fs_out / 1000:6.1f} k  peak {peak:6.2f} dBFS  HP max {worst:7.1f} dBFS at {wt / 1000:8.2f} ms"
          + (f"  pitch {pitch:7.2f} Hz" if pitch else ""))
for tt, f in ev:
    print(f"   source -> {f:6.0f} Hz at {tt / 1000:8.2f} ms")
print("PASS ratechg" if ok else "FAIL ratechg")
