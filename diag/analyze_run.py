"""Analyse a pnp7_teleop CSV (dry or live) before trusting it.

Checks the things that must hold for the run to be safe to repeat on the robot:
per-joint command velocity and acceleration stayed inside the configured
limits, no command discontinuities, and the session clamp behaved as intended.

  python analyze_run.py /tmp/dry_j7.csv --conf pnp7_teleop.conf
"""
from __future__ import annotations

import argparse
import csv
import math
import sys

NJ = 7
STATE_NAMES = {0: "READY", 1: "TELEOP", 2: "PAUSED"}


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


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("csv")
    ap.add_argument("--conf", default="conf/pnp7_teleop.conf")
    args = ap.parse_args()

    conf = load_conf(args.conf)
    def per_joint(key, default):
        vals = [float(x) for x in conf.get(key, str(default)).split()]
        return vals * NJ if len(vals) == 1 else vals

    v_lims = per_joint("max_joint_velocity", 0.30)
    a_lims = per_joint("max_joint_acceleration", 1.50)
    sess = float(conf.get("max_session_delta", 0.50))
    scale = float(conf.get("scale", "0.25").split()[0])
    mask = conf.get("enabled_joints", "0000001")
    enabled = [i for i in range(NJ) if i < len(mask) and mask[i] == "1"]

    rows = list(csv.DictReader(open(args.csv)))
    if len(rows) < 3:
        print("not enough rows")
        return 1

    t = [int(r["t_ns"]) / 1e9 for r in rows]
    dur = t[-1] - t[0]
    held = [r for r in rows if r["deadman"] == "1"]

    # The wall-clock stamp is taken inside the callback and jitters with
    # scheduling; libfranka's period is authoritative. Prefer it when logged,
    # otherwise fall back to the median, never to per-sample wall-clock deltas.
    if "dt_s" in rows[0]:
        dts = [float(r["dt_s"]) for r in rows[1:]]
        dt_source = "logged period"
    else:
        dts = [t[k] - t[k - 1] for k in range(1, len(t))]
        dt_source = "wall clock (median)"
    dt_nom = sorted(dts)[len(dts) // 2]

    print(f"rows={len(rows)}  duration={dur:.2f}s  "
          f"mean_rate={len(rows)/dur:.0f} Hz")
    wall = sorted(t[k] - t[k - 1] for k in range(1, len(t)))
    print(f"cycle period: nominal {dt_nom*1000:.4f} ms from {dt_source}; "
          f"wall-clock jitter {wall[0]*1000:.3f}..{wall[-1]*1000:.3f} ms")
    print(f"deadman held: {len(held)} rows "
          f"({100.0*len(held)/len(rows):.1f}%)")

    transitions = 0
    prev = rows[0]["state"]
    for r in rows[1:]:
        if r["state"] != prev:
            transitions += 1
            prev = r["state"]
    states = sorted({STATE_NAMES.get(int(r["state"]), r["state"]) for r in rows})
    print(f"state transitions: {transitions}  states seen: {states}")
    print(f"enabled joints: {[f'J{i+1}' for i in enabled]}  "
          f"scale={scale}  session_clamp={sess} rad\n")

    ok = True
    warnings_sat: list[str] = []
    held_mask = [r["deadman"] == "1" for r in rows]
    print(f"{'joint':<7}{'lead_delta range':>26}{'q_target range':>26}"
          f"{'max|v|':>10}{'max|a|':>10}{'vsat':>7}")
    for j in range(NJ):
        qt = [float(r[f"q_target{j}"]) for r in rows]
        ld = [float(r[f"lead_delta{j}"]) for r in rows]

        # Evaluate against the nominal period. Dividing by per-sample wall
        # clock deltas inflates both figures purely from scheduling jitter.
        vs, accs = [], []
        pv = 0.0
        for k in range(1, len(qt)):
            v = (qt[k] - qt[k - 1]) / dt_nom
            vs.append(v)
            accs.append((v - pv) / dt_nom)
            pv = v

        mv = max((abs(v) for v in vs), default=0.0)
        ma = max((abs(a) for a in accs), default=0.0)
        v_lim, a_lim = v_lims[j], a_lims[j]
        step_lim = v_lim * dt_nom
        over = sum(1 for k in range(1, len(qt))
                   if abs(qt[k] - qt[k - 1]) > step_lim * 1.001)
        flag = ""
        if over:
            flag += f" VEL! ({over} cycles)"
            ok = False
        if ma > a_lim * 1.5:
            flag += " ACC!"
            ok = False

        # How often the operator was throttled by the velocity cap. Saturation
        # is not a data fault -- the action is q_command, recorded after
        # limiting -- but it means demonstrations are slower than intended.
        held_steps = [abs(qt[k] - qt[k - 1]) for k in range(1, len(qt))
                      if held_mask[k]]
        sat = sum(1 for st in held_steps if st > step_lim * 0.98)
        sat_pct = 100.0 * sat / max(len(held_steps), 1)
        if sat_pct > 15.0:
            warnings_sat.append(f"J{j+1} at the velocity cap "
                                f"{sat_pct:.0f}% of held cycles")
        print(f"{'J'+str(j+1):<7}{f'{min(ld):+.4f} .. {max(ld):+.4f}':>26}"
              f"{f'{min(qt):+.4f} .. {max(qt):+.4f}':>26}"
              f"{mv:>10.4f}{ma:>10.3f}{sat_pct:>6.1f}%{flag}")

    print()
    for j in enabled:
        qt = [float(r[f"q_target{j}"]) for r in rows]
        ld = [float(r[f"lead_delta{j}"]) for r in rows]
        span_cmd = max(qt) - min(qt)
        want = max(abs(min(ld)), abs(max(ld))) * scale
        print(f"J{j+1}: lead swept {max(ld)-min(ld):.4f} rad "
              f"({math.degrees(max(ld)-min(ld)):.1f} deg)")
        print(f"    unclamped that would ask for {want:.4f} rad of Franka motion")
        print(f"    command actually spanned {span_cmd:.4f} rad "
              f"({math.degrees(span_cmd):.1f} deg)")
        if want > sess + 1e-6:
            print(f"    -> session clamp engaged, as designed "
                  f"(capped at {sess} rad)")
            print(f"    -> usable lead travel per clutch is {sess/scale:.2f} rad "
                  f"({math.degrees(sess/scale):.0f} deg); release and re-clutch "
                  f"to continue past it")

    # gripper, when the hand was enabled for the run
    if "gripper_width" in rows[0]:
        gw = [float(r["gripper_width"]) for r in rows]
        gt = [float(r["gripper_target"]) for r in rows]
        gk = [int(r["gripper_ticks"]) for r in rows]
        if max(gw) >= 0:
            print("gripper:")
            print(f"  trigger ticks    {min(gk)} .. {max(gk)} "
                  f"(span {max(gk)-min(gk)})")
            print(f"  commanded width  {min(gt)*1000:.1f} .. {max(gt)*1000:.1f} mm")
            print(f"  measured width   {min(gw)*1000:.1f} .. {max(gw)*1000:.1f} mm")
            # A move issued before the operator engaged would show as width
            # changing while the dead-man was never held.
            first_held = next((k for k, r in enumerate(rows)
                               if r["deadman"] == "1"), None)
            # Quantify how far the hand trails the trigger. The Franka Hand
            # cannot servo, so some lag is inherent; this makes it a number.
            span = max(gt) - min(gt)
            if span > 0.01:
                n = len(gt)
                best_lag, best_score = 0, None
                for lag in range(0, 4000, 10):
                    idx = range(lag, n, 23)
                    sc = sum(abs(gw[k] - gt[k - lag]) for k in idx)
                    cnt = max(len(range(lag, n, 23)), 1)
                    sc /= cnt
                    if best_score is None or sc < best_score:
                        best_score, best_lag = sc, lag
                ms = best_lag * (dur / n) * 1000
                ceiling = " (AT SEARCH CEILING -- true lag may exceed this)" \
                    if best_lag >= 3990 else ""
                print(f"  command->motion lag ~{best_lag} cycles "
                      f"({ms:.0f} ms){ceiling}")
            else:
                print(f"  lag not estimated: trigger spanned only "
                      f"{span*1000:.1f} mm")

            if first_held is not None and first_held > 0:
                moved_before = max(gw[:first_held]) - min(gw[:first_held])
                flag = "  UNCOMMANDED MOTION" if moved_before > 0.003 else ""
                print(f"  width motion before first dead-man press: "
                      f"{moved_before*1000:.1f} mm{flag}")
                if moved_before > 0.003:
                    ok = False
            print()

    # tracking fidelity: how well the arm followed the command. Meaningful only
    # on a live log, where q_robot is measured rather than echoed.
    live = any(
        abs(float(r["q_robot0"]) - float(r["q_target0"])) > 1e-9 for r in rows
    ) or any(
        abs(float(r[f"q_robot{j}"]) - float(r[f"q_target{j}"])) > 1e-9
        for j in enabled for r in rows[:200]
    )
    if live:
        print("tracking (measured vs commanded):")
        for j in enabled:
            qt = [float(r[f"q_target{j}"]) for r in rows]
            qr = [float(r[f"q_robot{j}"]) for r in rows]
            err = [qr[k] - qt[k] for k in range(len(qt))]
            mean_err = sum(err) / len(err)
            rms = math.sqrt(sum(e * e for e in err) / len(err))
            peak = max(abs(e) for e in err)

            print(f"  J{j+1}: mean={mean_err:+.5f} rad  rms={rms:.5f} rad  "
                  f"peak={peak:.5f} rad ({math.degrees(peak):.3f} deg)")

            # Cross-correlating two nearly-flat signals returns noise, not lag.
            # Require real motion before quoting a number.
            span = max(qt) - min(qt)
            if span < 0.01:
                print(f"       lag not estimated: joint moved only "
                      f"{span:.4f} rad")
                continue
            best_lag, best_score = 0, None
            for lag in range(0, 61):
                sc = sum(abs(qr[k] - qt[k - lag])
                         for k in range(lag, len(qt), 7))
                if best_score is None or sc < best_score:
                    best_score, best_lag = sc, lag
            print(f"       best-fit lag ~{best_lag} cycles "
                  f"({best_lag * dur / len(rows) * 1000:.1f} ms)")
        print()

    # command continuity
    worst_step, worst_j, worst_k = 0.0, -1, -1
    for j in range(NJ):
        qt = [float(r[f"q_target{j}"]) for r in rows]
        for k in range(1, len(qt)):
            d = abs(qt[k] - qt[k - 1])
            if d > worst_step:
                worst_step, worst_j, worst_k = d, j, k
    step_lim = v_lims[worst_j] * dt_nom
    print(f"\nlargest single-cycle command step: {worst_step:.8f} rad "
          f"on J{worst_j+1} at row {worst_k}")
    print(f"  velocity limit allows            {step_lim:.8f} rad per cycle")
    for w in warnings_sat:
        print(f"  NOTE: {w} -- raise that joint's limit for faster demos")
    if worst_step > step_lim * 1.001:
        print("  WARNING: exceeds the per-cycle velocity limit")
        ok = False

    print(f"\noverall: {'PASS' if ok else 'CHECK'}")
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
