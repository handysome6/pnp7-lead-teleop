"""Prove the signed-decode fix on the two joints that rest near tick 0/4095.

Measured behaviour: the XL330 in Position Control Mode does NOT wrap modularly
in this range. It reports a continuous SIGNED int32 that runs straight past the
0..4095 window -- J6 was observed at -397 and J5 at 4104. The original fault was
therefore a negative value decoded as unsigned (-3 reading as 2^32-3), not a
wrap.

So the proof this test looks for is a raw reading OUTSIDE 0..4095 that decodes
with the correct sign while the continuous signal stays smooth. A true modular
wrap is also accepted if the hardware ever produces one. Any continuous step
larger than --max-step fails, because that is what would be scaled into a
Franka command.

Read-only. The Franka is never contacted.

  python verify_wrap.py --seconds 40
"""
from __future__ import annotations

import argparse
import sys
import time

from pnp7.lead import ALL_IDS, PNP7Lead

WATCH = [5, 6]          # servo ids resting on the wrap boundary


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", default="/dev/pnp7_lead")
    ap.add_argument("--baud", type=int, default=1000000)
    ap.add_argument("--seconds", type=float, default=40.0)
    ap.add_argument("--max-step", type=int, default=200,
                    help="largest tolerated continuous step, in ticks")
    args = ap.parse_args()

    lead = PNP7Lead(args.port, args.baud)
    lead.open()
    lead.assert_torque_disabled()

    idx = {sid: ALL_IDS.index(sid) for sid in WATCH}
    prev_raw = {sid: None for sid in WATCH}
    prev_cont = {sid: None for sid in WATCH}
    raw_wraps = {sid: 0 for sid in WATCH}
    out_of_range = {sid: 0 for sid in WATCH}
    raw_min = {sid: None for sid in WATCH}
    raw_max = {sid: None for sid in WATCH}
    max_cont_step = {sid: 0 for sid in WATCH}
    violations: list[str] = []
    n = 0

    print(f"Slowly rotate lead-arm joints {WATCH} back and forth THROUGH their")
    print("rest position, so each crosses the encoder boundary a few times.")
    print(f"Watching for {args.seconds:.0f}s ...\n")
    print(f"{'t':>6}  " + "  ".join(
        f"{'id'+str(s)+' raw':>10}{'cont':>10}" for s in WATCH))

    t0 = time.time()
    last_print = 0.0
    while time.time() - t0 < args.seconds:
        s = lead.read()
        if s is None:
            continue
        n += 1
        for sid in WATCH:
            i = idx[sid]
            raw, cont = s.ticks_raw[i], s.ticks_cont[i]
            if raw < 0 or raw > 4095:
                out_of_range[sid] += 1
            raw_min[sid] = raw if raw_min[sid] is None else min(raw_min[sid], raw)
            raw_max[sid] = raw if raw_max[sid] is None else max(raw_max[sid], raw)
            if prev_raw[sid] is not None:
                if abs(raw - prev_raw[sid]) > 2048:
                    raw_wraps[sid] += 1
                step = abs(cont - prev_cont[sid])
                max_cont_step[sid] = max(max_cont_step[sid], step)
                if step > args.max_step:
                    violations.append(
                        f"id {sid}: continuous step of {step} ticks "
                        f"({step * 360.0 / 4096.0:.1f} deg) at t={time.time()-t0:.2f}s"
                    )
            prev_raw[sid], prev_cont[sid] = raw, cont

        now = time.time() - t0
        if now - last_print > 0.25:
            last_print = now
            cols = "  ".join(
                f"{s.ticks_raw[idx[sid]]:>10}{s.ticks_cont[idx[sid]]:>10}"
                for sid in WATCH)
            sys.stdout.write(f"\r{now:>6.1f}  {cols}")
            sys.stdout.flush()

    lead.close()
    print("\n")
    print(f"samples={n}  read_failures={lead.read_failures}  "
          f"rejected_jumps={lead.rejected_jumps}  "
          f"serial_errors={lead.serial_errors}  reopens={lead.reopens}\n")

    for sid in WATCH:
        print(f"id {sid}: raw range {raw_min[sid]}..{raw_max[sid]}, "
              f"{out_of_range[sid]} sample(s) outside 0..4095, "
              f"{raw_wraps[sid]} modular wrap(s), "
              f"largest continuous step = {max_cont_step[sid]} ticks "
              f"({max_cont_step[sid] * 360.0 / 4096.0:.1f} deg)")

    evidence = sum(out_of_range.values()) + sum(raw_wraps.values())
    if violations:
        print(f"\nFAIL: {len(violations)} discontinuity(ies)")
        for v in violations[:10]:
            print(f"  {v}")
        return 1
    if evidence == 0:
        print("\nINCONCLUSIVE: every reading stayed inside 0..4095, so the "
              "signed decode was never exercised.")
        print("Re-run and rotate those joints further past their rest point.")
        return 2
    print(f"\nPASS: {evidence} out-of-range/wrapped reading(s) decoded correctly, "
          f"continuous signal stayed smooth.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
