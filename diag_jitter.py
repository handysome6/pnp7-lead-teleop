"""Quantify command dither while the operator is holding still.

Motor whine with the dead-man held but the lead arm stationary means the
commanded target is not actually constant. The lead arm's encoder is quantised
at 4096 counts/rev (0.0015 rad/count), and at scale 1.0 a single count of
electrical noise becomes 1.5 mrad of Franka command. Whether that matters
depends on the amplitude and the frequency it survives the filter at.

This finds windows where the dead-man was held and the lead arm was still, and
reports what the command was doing in them.

  python diag_jitter.py /tmp/demo_run.csv --conf demo.conf
"""
from __future__ import annotations

import argparse
import csv
import math

import numpy as np

NJ = 7
TICKS_TO_RAD = 2.0 * math.pi / 4096.0


def load_conf(path):
    conf = {}
    try:
        with open(path) as fh:
            for line in fh:
                line = line.split("#", 1)[0].strip()
                if "=" in line:
                    k, v = line.split("=", 1)
                    conf[k.strip()] = v.strip()
    except OSError:
        pass
    return conf


def lead_noise(seconds: float) -> int:
    """Measure how much the stationary lead arm's encoders wander.

    This is the root input to the whole chain: at scale 1.0 one count of noise
    becomes 1.5 mrad of Franka command, so the servo's own quantisation and
    electrical noise set the floor for how still the robot can be held.
    """
    import time

    from pnp7_lead import ALL_IDS, PNP7Lead

    lead = PNP7Lead()
    lead.open()
    lead.assert_torque_disabled()
    print(f"sampling the lead arm for {seconds:.0f}s -- do not touch it\n")

    samples, times = [], []
    t_end = time.time() + seconds
    while time.time() < t_end:
        s = lead.read()
        if s is not None:
            samples.append(list(s.ticks_raw))
            times.append(s.t_monotonic_ns / 1e9)
    lead.close()

    if len(samples) < 50:
        print("too few samples")
        return 1

    arr = np.array(samples, dtype=float)
    dt = float(np.median(np.diff(times)))
    print(f"{len(samples)} samples at {1/dt:.0f} Hz\n")
    print(f"{'servo':<8}{'p-p':>8}{'std':>9}{'changes/s':>12}"
          f"{'-> mrad p-p @1.0':>19}")
    for i, sid in enumerate(ALL_IDS):
        col = arr[:, i]
        pp = col.max() - col.min()
        sd = float(np.std(col))
        ch = int(np.sum(np.diff(col) != 0)) / seconds
        print(f"{sid:<8}{pp:>6.0f} ct{sd:>8.2f} ct{ch:>12.0f}"
              f"{pp * TICKS_TO_RAD * 1000:>16.2f} mr")

    worst = int(np.argmax(arr.max(axis=0) - arr.min(axis=0)))
    pp = arr[:, worst].max() - arr[:, worst].min()
    print(f"\nworst: servo {ALL_IDS[worst]} wanders {pp:.0f} counts at rest")
    print(f"at scale 1.0 that is {pp * TICKS_TO_RAD * 1000:.2f} mrad "
          f"({math.degrees(pp * TICKS_TO_RAD):.3f} deg) of commanded dither")
    print("\nA Franka joint asked to move a fraction of a milliradian back and")
    print("forth at hundreds of Hz will buzz audibly without actually going")
    print("anywhere. That is the noise this measures.")
    return 0


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("csv", nargs="?", default="")
    ap.add_argument("--conf", default="demo.conf")
    ap.add_argument("--still-ticks", type=float, default=2.0,
                    help="lead motion below this (peak-to-peak ticks) over the "
                         "window counts as holding still")
    ap.add_argument("--window", type=float, default=1.0, help="seconds")
    ap.add_argument("--lead-only", type=float, default=0.0, metavar="SECONDS",
                    help="skip the CSV and sample the stationary lead arm "
                         "directly for this long. Measures the encoder noise "
                         "at the source, with no robot involved.")
    args = ap.parse_args()

    if args.lead_only:
        return lead_noise(args.lead_only)

    conf = load_conf(args.conf)
    lowpass = float(conf.get("lowpass_hz", 6.0))
    scales = [float(x) for x in conf.get("scale", "1.0").split()]
    if len(scales) == 1:
        scales *= NJ

    rows = list(csv.DictReader(open(args.csv)))
    if len(rows) < 100:
        print(f"{args.csv} has only {len(rows)} rows -- not a usable "
              f"recording. Record a longer one, or use --lead-only.")
        return 1
    t = np.array([int(r["t_ns"]) for r in rows], dtype=np.int64) / 1e9
    held = np.array([r["deadman"] == "1" for r in rows])
    dt = np.median(np.diff(t))
    n_win = max(int(args.window / dt), 10)

    ld = np.array([[float(r[f"lead_delta{j}"]) for j in range(NJ)]
                   for r in rows])
    qt = np.array([[float(r[f"q_target{j}"]) for j in range(NJ)]
                   for r in rows])
    qr = np.array([[float(r[f"q_robot{j}"]) for j in range(NJ)]
                   for r in rows])

    print(f"rows={len(rows)}  dt={dt*1000:.3f} ms  "
          f"lowpass={lowpass} Hz  scale={scales[0]}")
    print(f"encoder resolution: 1 count = {TICKS_TO_RAD*1000:.3f} mrad "
          f"= {math.degrees(TICKS_TO_RAD):.4f} deg\n")

    # Windows where the dead-man was held and the lead arm was essentially still
    found = []
    i = 0
    while i + n_win < len(rows):
        if not held[i:i + n_win].all():
            i += n_win // 2
            continue
        seg_ld = ld[i:i + n_win]
        pp_ticks = (seg_ld.max(axis=0) - seg_ld.min(axis=0)) / TICKS_TO_RAD
        if pp_ticks.max() <= args.still_ticks:
            found.append(i)
            i += n_win
        else:
            i += n_win // 4

    if not found:
        print("no still-and-held windows found; hold the dead-man without "
              "moving the lead arm for a second or two and re-record")
        return 1

    print(f"still-and-held windows: {len(found)} of {args.window:.1f}s each\n")
    print(f"{'joint':<7}{'lead p-p':>12}{'cmd p-p':>14}{'cmd rms':>13}"
          f"{'reversals/s':>13}{'robot p-p':>13}")

    worst = []
    for j in range(NJ):
        pp_l, pp_c, rms_c, rev, pp_r = [], [], [], [], []
        for i in found:
            seg_l = ld[i:i + n_win, j] / TICKS_TO_RAD
            seg_c = qt[i:i + n_win, j]
            seg_r = qr[i:i + n_win, j]
            pp_l.append(seg_l.max() - seg_l.min())
            pp_c.append(seg_c.max() - seg_c.min())
            rms_c.append(float(np.std(seg_c)))
            pp_r.append(seg_r.max() - seg_r.min())
            d = np.diff(seg_c)
            nz = d[np.abs(d) > 1e-9]
            rev.append(int(np.sum(np.diff(np.sign(nz)) != 0)) / args.window
                       if len(nz) > 2 else 0.0)

        m_pp_c = float(np.mean(pp_c))
        worst.append((m_pp_c, j))
        print(f"{'J'+str(j+1):<7}{np.mean(pp_l):>9.2f} ct"
              f"{m_pp_c*1000:>11.3f} mr"
              f"{np.mean(rms_c)*1000:>10.3f} mr"
              f"{np.mean(rev):>13.0f}"
              f"{np.mean(pp_r)*1000:>10.3f} mr")

    print("\n(ct = encoder counts, mr = milliradians)")

    # Frequency content of the worst joint's command dither
    worst.sort(reverse=True)
    j = worst[0][1]
    seg = np.concatenate([qt[i:i + n_win, j] - qt[i:i + n_win, j].mean()
                          for i in found])
    if len(seg) > 64:
        spec = np.abs(np.fft.rfft(seg * np.hanning(len(seg))))
        freqs = np.fft.rfftfreq(len(seg), dt)
        band = (freqs > 0.5)
        peak = freqs[band][np.argmax(spec[band])]
        print(f"\nJ{j+1} dither spectrum peaks at {peak:.1f} Hz "
              f"(low-pass is set to {lowpass} Hz)")
        for lo, hi, name in ((0.5, 5, "below cutoff"), (5, 20, "near cutoff"),
                             (20, 100, "above cutoff"), (100, 1e9, "high")):
            m = (freqs >= lo) & (freqs < hi)
            if m.any():
                frac = float(spec[m].sum() / spec[band].sum() * 100)
                print(f"  {name:<14} {lo:>5.0f}-{hi if hi < 1e9 else 500:>4.0f} Hz"
                      f"  {frac:5.1f}% of energy")

    j_w = worst[0][1]
    mrad = worst[0][0] * 1000
    lead_pp_ct = float(np.mean([
        (ld[i:i + n_win, j_w].max() - ld[i:i + n_win, j_w].min()) / TICKS_TO_RAD
        for i in found]))
    explained = lead_pp_ct * TICKS_TO_RAD * 1000 * scales[j_w]
    print(f"\nworst command motion: J{j_w+1} at {mrad:.3f} mrad p-p")
    print(f"  lead arm moved {lead_pp_ct:.2f} counts = {explained:.3f} mrad "
          f"at scale {scales[j_w]}")
    if mrad > explained * 2.5:
        print("  The command moved much further than the lead did, so this "
              "window is\n  the safety chain still SETTLING toward an earlier "
              "target, not dither.\n  Re-record with the lead arm genuinely "
              "untouched to isolate jitter.")
    else:
        print("  Command motion is explained by real lead motion; no excess "
              "dither here.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
