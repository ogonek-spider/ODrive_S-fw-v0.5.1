#!/usr/bin/env python3
"""Run encoder offset calibration N times and compare the resulting offset.

For an absolute SPI encoder (AS5047P, cpr 16384), AXIS_STATE_ENCODER_OFFSET_
CALIBRATION spins the rotor in lockin and measures the offset between the
encoder's absolute reading and the motor's electrical angle. A healthy
mount/magnet gives a REPEATABLE offset run-to-run (a few counts of scatter).
Large scatter -> slipping magnet, loose encoder, or intermittent SPI -- exactly
the kind of fault that leaves commutation 90 deg off and the current pinned at
the limit with no rotation.

Commutation only cares about offset MOD (cpr / pole_pairs) counts, so we report
both the raw offset and the reduced electrical offset (counts + degrees).

Within-session repeatability is necessary but NOT sufficient: a slipping magnet
reads identically across 5 back-to-back cals yet drifts tens of degrees after a
load event. So we also (a) flag monotonic in-session creep, and (b) persist the
electrical offset per serial to a history file and flag cross-session DRIFT vs
the last run (the signal that actually catches a slipping magnet). Run this once
to set a baseline, apply a load / spin, then run again -- persistent drift means
a mechanical fix (re-bond the magnet), not a calibration problem.

For an absolute encoder the commutation-relevant electrical offset should not
depend on where the rotor is parked -- but any magnet eccentricity / air-gap
variation makes the cal's lockin sweep (a partial mechanical revolution)
average a slightly different arc depending on its start position. So a rotor
hand-moved between sessions can read tens of degrees apart with NO slip. To make
the cross-session check meaningful, by default we PARK the rotor at a fixed
absolute angle (--park-deg, closed-loop position) before each cal so every sweep
starts from the same arc. Pass --no-park for the legacy behavior. The valid slip
signal is then drift across a controlled LOAD event with parking on, not a bare
hand-moved comparison.

Bench-safe for a FREE shaft (the rotor turns a few slow revolutions per run).
Does NOT write to flash. The last calibration is left live in RAM so you can
immediately re-characterize; pass --save to persist it.
"""

import argparse
import json
import math
import os
import statistics
import sys
import time

# tools/ contains a local odrive/ package that shadows the installed one;
# drop cwd AND this script's own dir so `import odrive` finds the venv package.
_SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
sys.path = [p for p in sys.path if p not in ("", ".", os.getcwd(), _SCRIPT_DIR)]

import odrive
from odrive.enums import (
    AXIS_STATE_CLOSED_LOOP_CONTROL,
    AXIS_STATE_ENCODER_OFFSET_CALIBRATION,
    AXIS_STATE_MOTOR_CALIBRATION,
    AXIS_STATE_IDLE,
    CONTROL_MODE_POSITION_CONTROL,
    INPUT_MODE_PASSTHROUGH,
)

# Default cross-session history file (per-serial last electrical offset).
_DEFAULT_HISTORY = os.path.abspath(os.path.join(
    _SCRIPT_DIR, "..", "spider-motor-tools", "reports", "encoder_offset_history.json"))


def parse_args():
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--serial-number")
    p.add_argument("--axis", type=int, default=0, choices=(0, 1))
    p.add_argument("--runs", type=int, default=5)
    p.add_argument("--timeout", type=float, default=25.0,
                   help="Seconds to wait for each calibration to return to IDLE.")
    p.add_argument("--save", action="store_true",
                   help="save_configuration() after the last run (persists offset).")
    p.add_argument("--history", default=_DEFAULT_HISTORY,
                   help="Per-serial offset history file for cross-session drift check.")
    p.add_argument("--no-history", action="store_true",
                   help="Skip the cross-session drift check and history write.")
    p.add_argument("--drift-threshold-deg", type=float, default=5.0,
                   help="Flag cross-session electrical drift above this (deg).")
    p.add_argument("--park-deg", type=float, default=0.0,
                   help="Park the rotor at this absolute mechanical angle (deg "
                        "within one turn) before EACH cal, so the lockin sweep "
                        "starts from the same arc and cross-session offsets are "
                        "comparable. Needs a roughly-valid live offset to move.")
    p.add_argument("--no-park", action="store_true",
                   help="Do not park the rotor before each cal (legacy behavior).")
    p.add_argument("--park-vel", type=float, default=2.0,
                   help="Velocity limit (motor turns/s) while parking.")
    p.add_argument("--park-timeout", type=float, default=6.0,
                   help="Seconds to wait for the park move to settle.")
    return p.parse_args()


def errs(axis):
    return (int(axis.error), int(axis.motor.error),
            int(axis.encoder.error), int(axis.controller.error))


def wait_idle(axis, timeout):
    deadline = time.monotonic() + timeout
    time.sleep(0.4)
    while axis.current_state != AXIS_STATE_IDLE and time.monotonic() < deadline:
        time.sleep(0.2)
    return axis.current_state == AXIS_STATE_IDLE


def park_rotor(axis, park_frac, vel_limit, timeout):
    """Drive the rotor (closed-loop position) to the nearest absolute angle whose
    fractional turn == park_frac, so every cal's lockin sweep starts from the
    same physical arc. Returns the settled fractional position, or None on
    failure (e.g. encoder not ready / closed-loop entry refused).

    Uses whatever commutation offset is currently live -- even ~20 deg off still
    produces ~94% torque, plenty to creep the free shaft into position.
    """
    c, e = axis.controller, axis.encoder
    if not bool(e.is_ready):
        return None
    for attr in (axis, axis.motor, axis.encoder, axis.controller):
        try:
            attr.error = 0
        except Exception:
            pass
    c.config.control_mode = CONTROL_MODE_POSITION_CONTROL
    c.config.input_mode = INPUT_MODE_PASSTHROUGH
    c.config.vel_limit = max(vel_limit, 1.0)
    axis.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
    time.sleep(0.3)
    if axis.current_state != AXIS_STATE_CLOSED_LOOP_CONTROL:
        axis.requested_state = AXIS_STATE_IDLE
        return None
    cur = float(e.pos_estimate)
    base = math.floor(cur) + park_frac
    target = min((base - 1.0, base, base + 1.0), key=lambda t: abs(t - cur))
    c.input_pos = target
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if abs(float(e.pos_estimate) - target) < 0.01:
            break
        time.sleep(0.05)
    time.sleep(0.3)                       # let it hold/settle at the target
    settled = float(e.pos_estimate) % 1.0
    axis.requested_state = AXIS_STATE_IDLE
    time.sleep(0.2)
    return settled


def circ_diff(a, b, period):
    """Signed shortest distance from b to a on a ring of size `period`."""
    d = (a - b) % period
    if d > period / 2.0:
        d -= period
    return d


def circ_mean(vals, period):
    """Mean of ring values, unwrapped relative to the first to avoid wrap bias."""
    ref = vals[0]
    unwrapped = [ref + circ_diff(v, ref, period) for v in vals]
    return statistics.fmean(unwrapped) % period


def load_history(path):
    try:
        with open(path, encoding="utf-8") as f:
            return json.load(f)
    except (FileNotFoundError, ValueError):
        return {}


def save_history(path, hist):
    os.makedirs(os.path.dirname(path), exist_ok=True)
    with open(path, "w", encoding="utf-8") as f:
        json.dump(hist, f, indent=2, sort_keys=True)
        f.write("\n")


def main():
    args = parse_args()
    print("Connecting to ODrive...", flush=True)
    dev = odrive.find_any(serial_number=args.serial_number, timeout=25)
    axis = getattr(dev, f"axis{args.axis}")
    m, e = axis.motor, axis.encoder

    cpr = int(e.config.cpr)
    pp = int(m.config.pole_pairs)
    counts_per_elec = cpr / pp
    print(f"serial={format(dev.serial_number, 'x')}  vbus={float(dev.vbus_voltage):.1f}V  "
          f"FET={float(axis.fet_thermistor.temperature):.1f}C", flush=True)
    print(f"encoder mode={int(e.config.mode)}  cpr={cpr}  pole_pairs={pp}  "
          f"counts/elec-rev={counts_per_elec:.1f}", flush=True)
    print(f"stored offset={int(e.config.offset)}  offset_float={float(e.config.offset_float):.4f}  "
          f"dir={int(e.config.direction)}  motor.is_calibrated={bool(m.is_calibrated)}", flush=True)

    try:
        dev.clear_errors()
    except Exception:
        pass

    # Encoder offset cal needs a calibrated motor (R/L). Run motor cal once if needed.
    if not bool(m.is_calibrated):
        print("\nmotor not calibrated -- running MOTOR_CALIBRATION once first...", flush=True)
        axis.requested_state = AXIS_STATE_MOTOR_CALIBRATION
        if not wait_idle(axis, 20) or not bool(m.is_calibrated):
            sys.exit(f"motor calibration failed errs={errs(axis)}")
        print(f"  R={float(m.config.phase_resistance):.5f} ohm  "
              f"L={float(m.config.phase_inductance)*1e6:.1f} uH  ok", flush=True)

    c = axis.controller
    saved_cfg = (c.config.control_mode, c.config.input_mode, c.config.vel_limit)
    park_frac = (args.park_deg % 360.0) / 360.0

    results = []
    print(f"\n=== running ENCODER_OFFSET_CALIBRATION x{args.runs} "
          f"({'park@%.0fdeg' % args.park_deg if not args.no_park else 'no-park'}) ===",
          flush=True)
    for i in range(1, args.runs + 1):
        if not args.no_park:
            settled = park_rotor(axis, park_frac, args.park_vel, args.park_timeout)
            if settled is None:
                print(f"  run {i}: park SKIPPED (encoder not ready / no closed-loop)",
                      flush=True)
            else:
                print(f"  run {i}: parked at {settled*360.0:6.2f}deg "
                      f"(target {args.park_deg:.1f})", flush=True)
        try:
            dev.clear_errors()
        except Exception:
            pass
        axis.requested_state = AXIS_STATE_ENCODER_OFFSET_CALIBRATION
        idle = wait_idle(axis, args.timeout)
        off = int(e.config.offset)
        offf = float(e.config.offset_float)
        direction = int(e.config.direction)
        ready = bool(e.is_ready)
        er = errs(axis)
        elec = off % counts_per_elec            # reduced offset (commutation-relevant)
        elec_deg = elec / counts_per_elec * 360.0
        ok = idle and ready and not any(er)
        results.append({"run": i, "offset": off, "offset_float": offf,
                        "direction": direction, "elec_counts": elec,
                        "elec_deg": elec_deg, "ready": ready, "errors": er, "ok": ok})
        print(f"  run {i}: offset={off:6d}  offset_float={offf:8.4f}  dir={direction}  "
              f"elec={elec:6.1f}cnt ({elec_deg:6.2f}deg)  ready={ready}  "
              f"errs={er}  {'OK' if ok else 'FAIL'}", flush=True)
        time.sleep(0.4)

    axis.requested_state = AXIS_STATE_IDLE
    # restore controller config we touched while parking
    (c.config.control_mode, c.config.input_mode, c.config.vel_limit) = saved_cfg

    good = [r for r in results if r["ok"]]
    print("\n=== SUMMARY ===", flush=True)
    print(f"{len(good)}/{len(results)} runs succeeded", flush=True)
    deg = 360.0 / counts_per_elec        # counts -> electrical degrees
    cur_elec = None
    flags = []
    if len(good) >= 2:
        offs = [r["offset"] for r in good]
        elecs = [r["elec_counts"] for r in good]          # in run order
        dirs = {r["direction"] for r in good}
        spread = max(offs) - min(offs)
        elec_spread = max(elecs) - min(elecs)
        cur_elec = circ_mean(elecs, counts_per_elec)
        # within-session creep: net signed drift first->last good run
        creep = circ_diff(elecs[-1], elecs[0], counts_per_elec)
        print(f"raw offset   : min={min(offs)} max={max(offs)} spread={spread} cnt  "
              f"mean={statistics.fmean(offs):.1f} stdev={statistics.pstdev(offs):.2f}", flush=True)
        print(f"elec offset  : spread={elec_spread*deg:.2f} deg  "
              f"mean={cur_elec:.1f}cnt ({cur_elec*deg:.2f} deg)", flush=True)
        print(f"in-session   : creep first->last = {creep*deg:+.2f} deg", flush=True)
        print(f"direction    : {'STABLE ' + str(next(iter(dirs))) if len(dirs) == 1 else 'UNSTABLE ' + str(dirs)}", flush=True)
        if elec_spread * deg > 10.0:
            flags.append(f"within-session scatter {elec_spread*deg:.1f} deg > 10")
        if abs(creep * deg) > args.drift_threshold_deg:
            flags.append(f"monotonic in-session creep {creep*deg:+.1f} deg")
        if len(dirs) != 1:
            flags.append(f"direction unstable {dirs}")

    # --- cross-session drift check against stored history ---
    serial = format(dev.serial_number, "x")
    if not args.no_history and cur_elec is not None:
        hist = load_history(args.history)
        prev = hist.get(serial)
        if prev is not None and prev.get("counts_per_elec") == counts_per_elec:
            drift = circ_diff(cur_elec, prev["elec_counts"], counts_per_elec)
            print(f"\ncross-session: last={prev['elec_counts']*deg:.2f} deg "
                  f"({prev.get('time','?')})  now={cur_elec*deg:.2f} deg  "
                  f"drift={drift*deg:+.2f} deg", flush=True)
            if abs(drift * deg) > args.drift_threshold_deg:
                flags.append(f"cross-session drift {drift*deg:+.1f} deg > {args.drift_threshold_deg}")
        else:
            print(f"\ncross-session: no prior baseline for {serial} -- this run becomes the baseline", flush=True)
        hist[serial] = {"elec_counts": cur_elec, "elec_deg": cur_elec * deg,
                        "raw_offset_mean": statistics.fmean([r["offset"] for r in good]),
                        "runs": len(good), "counts_per_elec": counts_per_elec,
                        "time": time.strftime("%Y-%m-%d %H:%M:%S")}
        save_history(args.history, hist)

    if flags:
        print("\nVERDICT: offset NOT stable -> " + "; ".join(flags), flush=True)
        print("  -> suspect slipping sensor magnet / loose encoder. Re-run after a "
              "load event; persistent drift = mechanical fix (re-bond magnet).", flush=True)
    elif len(good) >= 2:
        print("\nVERDICT: offset repeatable within AND across sessions -> mount solid.", flush=True)

    if args.save and good and results[-1]["ok"]:
        e.config.pre_calibrated = True
        m.config.pre_calibrated = True
        dev.save_configuration()
        print(f"\nSAVED offset={int(e.config.offset)} to flash (pre_calibrated=True).", flush=True)
    else:
        print("\n(no flash write; last calibration is live in RAM -- pass --save to persist)", flush=True)


if __name__ == "__main__":
    main()
