#!/usr/bin/env python3
"""Characterize a geared joint by sweeping it BY HAND, with the axis idle.

    joint_hand_sweep.py --position 6-3

Reads both encoders while you move the joint by hand and reports, at ZERO
motor current:

  - is the motor actually COUPLED to the joint (leg6 coxa was not)
  - the real gearbox RATIO, and whether it is constant over the travel
    (leg1's knee varies 5.6:1 to 7.4:1 with angle)
  - the SIGN: does +joint motion correspond to +motor motion
  - the mechanical TRAVEL range, in joint degrees from the current zero

This is the step that belongs BEFORE adding amps. A motor-driven ratio probe
on a stiff joint returns breakaway and backlash garbage, and a decoupled or
seized joint looks like a tuning problem until you turn it by hand.

The axis is never armed: the script refuses to run unless it is IDLE, and it
only ever READS. Nothing is written to the board and nothing is saved.

NOTE: strips cwd from sys.path so ./odrive does not shadow the installed pkg.
"""
import argparse
import os
import sys
import time

sys.path = [p for p in sys.path if p not in ("", ".", os.getcwd())]

import odrive

AXIS_STATE_IDLE = 1
JOINT_NAMES = {1: "coxa", 2: "femur", 3: "knee"}

# Joint motion below this is treated as "not moving" when estimating the ratio:
# dividing by a near-zero joint delta manufactures enormous ratios.
MIN_JOINT_TURN = 0.02      # ~7 deg
# A motor that moves less than this while the joint sweeps is DECOUPLED.
DECOUPLED_MOTOR_TURN = 0.05


def parse_args():
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--position", help="Joint label <leg>-<joint>, e.g. 6-3 (for logging).")
    p.add_argument("--serial-number")
    p.add_argument("--seconds", type=float, default=40.0,
                   help="Sampling window (default 40).")
    p.add_argument("--rate", type=float, default=20.0,
                   help="Samples per second (default 20). Keep modest: USB polling "
                        "competes with the control loop.")
    p.add_argument("--segments", type=int, default=6,
                   help="Split the sweep into N equal joint-angle bands and report "
                        "the ratio in each, to expose a varying linkage (default 6).")
    return p.parse_args()


def main():
    args = parse_args()
    label = args.position or "joint"

    dev = odrive.find_any(serial_number=args.serial_number.upper()
                          if args.serial_number else None, timeout=30)
    print(f"board {format(dev.serial_number, 'x')}  "
          f"fw {dev.fw_version_major}.{dev.fw_version_minor}.{dev.fw_version_revision}  "
          f"node {dev.axis0.config.can_node_id}")

    if dev.axis0.current_state != AXIS_STATE_IDLE:
        raise SystemExit(f"axis0 is in state {dev.axis0.current_state}, not IDLE. "
                         "This check must run de-energised -- disarm first.")

    a0, a1 = dev.axis0.encoder, dev.axis1.encoder
    if a1.config.mode == 0:
        raise SystemExit("axis1 (joint encoder) is disabled -- nothing to compare against.")
    a0.error = 0
    a1.error = 0

    print(f"\n{label}: move the joint SLOWLY by hand over its whole travel,")
    print(f"end to end, for {args.seconds:.0f} s. Sampling ...\n")

    samples = []
    dt = 1.0 / args.rate
    deadline = time.monotonic() + args.seconds
    next_print = time.monotonic() + 2.0
    while time.monotonic() < deadline:
        m, j = a0.pos_estimate, a1.pos_estimate
        samples.append((m, j))
        now = time.monotonic()
        if now >= next_print:
            next_print = now + 2.0
            print(f"   joint {j * 360.0:+8.2f} deg   motor {m:+9.3f} turn", flush=True)
        time.sleep(dt)

    motors = [s[0] for s in samples]
    joints = [s[1] for s in samples]
    jmin, jmax = min(joints), max(joints)
    mmin, mmax = min(motors), max(motors)
    jspan, mspan = jmax - jmin, mmax - mmin

    print(f"\n--- {label} -----------------------------------------------")
    print(f"  samples          {len(samples)}")
    print(f"  joint travel     {jspan * 360.0:.2f} deg "
          f"({jmin * 360.0:+.2f} .. {jmax * 360.0:+.2f} from current zero)")
    print(f"  motor travel     {mspan:.3f} turn ({mmin:+.3f} .. {mmax:+.3f})")
    print(f"  encoder errors   axis0 {hex(a0.error)}  axis1 {hex(a1.error)}")

    if jspan < MIN_JOINT_TURN:
        raise SystemExit(f"\n  the joint barely moved ({jspan * 360.0:.2f} deg). "
                         "Sweep it end to end and re-run.")
    if mspan < DECOUPLED_MOTOR_TURN:
        print(f"\n  !! DECOUPLED: the joint swept {jspan * 360.0:.1f} deg but the motor")
        print(f"     shaft moved only {mspan:.4f} turn. The motor is not driving this")
        print("     joint -- broken coupling, stripped gear, or a slipping output.")
        print("     No amount of tuning fixes this; nothing else here is meaningful.")
        return 1

    # Overall ratio and sign, by least squares through (joint, motor). Using the
    # fitted slope rather than span/span keeps the SIGN, which span ratios lose.
    n = len(samples)
    jbar = sum(joints) / n
    mbar = sum(motors) / n
    sjj = sum((j - jbar) ** 2 for j in joints)
    sjm = sum((j - jbar) * (m - mbar) for j, m in samples)
    slope = sjm / sjj if sjj else 0.0
    resid = sum((m - mbar - slope * (j - jbar)) ** 2 for j, m in samples)
    r2 = 1.0 - resid / sum((m - mbar) ** 2 for m in motors) if n > 2 else 0.0

    print(f"\n  RATIO            {abs(slope):.2f} : 1  (motor turns per joint turn)")
    print(f"  SIGN             +joint -> {'+' if slope > 0 else '-'}motor  "
          f"-> position_direction {'+1' if slope > 0 else '-1'}")
    print(f"  linearity        R^2 = {r2:.5f}")

    # Per-band ratios: a constant-ratio gearbox holds its slope everywhere; a
    # linkage does not, and a fixed conversion factor would then be wrong at the
    # ends of travel.
    print(f"\n  ratio across the travel ({args.segments} bands):")
    edges = [jmin + (jmax - jmin) * k / args.segments for k in range(args.segments + 1)]
    band_ratios = []
    for k in range(args.segments):
        lo, hi = edges[k], edges[k + 1]
        band = [(j, m) for j, m in samples if lo <= j <= hi]
        if len(band) < 5:
            print(f"    {lo * 360:+7.1f} .. {hi * 360:+7.1f} deg   (too few samples)")
            continue
        bj = [j for j, _ in band]
        bm = [m for _, m in band]
        if max(bj) - min(bj) < 1e-4:
            print(f"    {lo * 360:+7.1f} .. {hi * 360:+7.1f} deg   (joint static here)")
            continue
        bjbar, bmbar = sum(bj) / len(bj), sum(bm) / len(bm)
        bsjj = sum((j - bjbar) ** 2 for j in bj)
        bslope = sum((j - bjbar) * (m - bmbar) for j, m in band) / bsjj
        band_ratios.append(abs(bslope))
        print(f"    {lo * 360:+7.1f} .. {hi * 360:+7.1f} deg   {abs(bslope):7.2f} : 1")

    if band_ratios:
        lo_r, hi_r = min(band_ratios), max(band_ratios)
        spread_pct = 100.0 * (hi_r - lo_r) / (sum(band_ratios) / len(band_ratios))
        print(f"\n  band spread      {lo_r:.2f} .. {hi_r:.2f} ({spread_pct:.0f}% of mean)")
        if spread_pct > 20.0:
            print("  !! the ratio VARIES with angle -- this is a linkage, not a fixed")
            print("     gearbox. Never convert motor turns to joint angle with one")
            print("     factor, and re-check the position loop at BOTH ends of travel.")
        else:
            print("  ratio is constant over the travel: a fixed gearbox.")

    print(f"\n  Derived position gain: {23.0 * abs(slope):.0f} "
          f"(23 x ratio, joint-side units)")
    print("  Nothing was written. Sign and travel still need confirming with a")
    print("  small guarded step before this joint is trusted under power.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
