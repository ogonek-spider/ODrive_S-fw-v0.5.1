#!/usr/bin/env python3
"""Free-spin test for a motor with a gearbox stage attached (no load).

Sweeps closed-loop velocity in BOTH directions at several speeds, logging
mean/max |Iq|, actual velocity, and any faults. Compares direction symmetry to
flag one-sided drag/bind in the gear mesh. USB-drop tolerant: reconnects and
continues if the native link drops mid-run (known bench flakiness).
"""
import argparse
import json
import os
import statistics
import time

HERE = os.path.dirname(os.path.abspath(__file__))

import odrive
from odrive.enums import (
    AXIS_STATE_IDLE,
    AXIS_STATE_CLOSED_LOOP_CONTROL,
    CONTROL_MODE_VELOCITY_CONTROL,
    INPUT_MODE_VEL_RAMP,
)

SERIAL = "3482345a3034"
SPEED_REACH_FRAC = 0.80


def connect(serial=SERIAL, timeout=15, attempts=8):
    # NB: the modern find_any serial filter is case-sensitive and the board
    # reports its serial uppercase, so filter ourselves after a bare find
    # (single board on this bench) rather than passing serial_number=.
    want = serial.lower() if serial else None
    last = None
    for i in range(attempts):
        try:
            dev = odrive.find_any(timeout=timeout)
            got = format(dev.serial_number, "x").lower()
            if want and got != want:
                raise RuntimeError(f"found {got}, wanted {want}")
            return dev
        except Exception as ex:  # noqa: BLE001
            last = ex
            print(f"  connect attempt {i+1} failed ({type(ex).__name__}); retrying...", flush=True)
            time.sleep(1.0)
    raise last


def err_tuple(odrv):
    a = odrv.axis0
    return [a.error, a.motor.error, a.encoder.error, a.controller.error]


def clear_errors(odrv):
    a = odrv.axis0
    a.motor.error = 0
    a.encoder.error = 0
    a.controller.error = 0
    a.error = 0


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--speeds", type=float, nargs="+", default=[2.0, 4.0, 6.0],
                   help="Motor turns/s magnitudes to sweep in each direction.")
    p.add_argument("--ratio", type=float, default=36.0, help="Gearbox ratio (for output-turn math).")
    p.add_argument("--dwell", type=float, default=2.0, help="Seconds to hold each speed.")
    p.add_argument("--ramp", type=float, default=8.0, help="vel_ramp_rate (t/s^2).")
    p.add_argument("--out", default="reports/motor-8-gearbox1to36-freespin-2026-07-10.json")
    args = p.parse_args()

    print("connecting...", flush=True)
    odrv = connect()
    print("serial:", format(odrv.serial_number, "x"), "vbus:", round(odrv.vbus_voltage, 2), flush=True)
    a = odrv.axis0
    c = a.controller
    m = a.motor
    e = a.encoder

    report = {
        "motor_id": "8",
        "serial_number": format(odrv.serial_number, "x"),
        "test": f"gearbox 1:{args.ratio:g} free-spin (no load)",
        "vbus_v": odrv.vbus_voltage,
        "ratio": args.ratio,
        "runs": [],
    }

    # Build signed speed list: up then down for each magnitude.
    signed = []
    for s in args.speeds:
        signed.append(+s)
    for s in reversed(args.speeds):
        signed.append(-s)

    print("pre-errors:", err_tuple(odrv), flush=True)
    clear_errors(odrv)
    c.config.control_mode = CONTROL_MODE_VELOCITY_CONTROL
    c.config.input_mode = INPUT_MODE_VEL_RAMP
    c.config.vel_ramp_rate = args.ramp
    # Raise the soft vel_limit above the sweep and disable the overspeed trip so
    # the motor is free to run right up to its voltage wall (this is a no-load
    # ceiling-finding run; config changes here are NOT saved to flash).
    top = max(abs(s) for s in args.speeds)
    c.config.vel_limit = top * 1.3
    c.config.enable_overspeed_error = False
    c.input_vel = 0.0

    a.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
    time.sleep(0.3)
    if a.current_state != AXIS_STATE_CLOSED_LOOP_CONTROL:
        print("FAIL: closed-loop entry failed, errors:", err_tuple(odrv), flush=True)
        a.requested_state = AXIS_STATE_IDLE
        return

    status = "PASS"
    try:
        for spd in signed:
            c.input_vel = spd
            time.sleep(args.dwell)  # let ramp settle
            iqs, vels = [], []
            for _ in range(30):
                time.sleep(0.02)
                iqs.append(abs(m.current_control.Iq_measured))
                vels.append(e.vel_estimate)
            mean_iq = statistics.fmean(iqs)
            max_iq = max(iqs)
            mean_vel = statistics.fmean(vels)
            errs = err_tuple(odrv)
            reached = abs(mean_vel) >= SPEED_REACH_FRAC * abs(spd)
            if not reached or any(errs):
                status = "FAIL"
            out_rpm = mean_vel / args.ratio * 60.0
            run = {
                "cmd_turns_s": spd,
                "mean_abs_iq_a": mean_iq,
                "max_abs_iq_a": max_iq,
                "actual_vel_turns_s": mean_vel,
                "output_rpm": out_rpm,
                "errors": errs,
                "reached": reached,
            }
            report["runs"].append(run)
            print(f"  cmd {spd:+.1f} t/s: mean|Iq|={mean_iq:.2f}A max={max_iq:.2f}A  "
                  f"actual={mean_vel:+.2f} t/s ({out_rpm:+.1f} out-RPM)  "
                  f"{'OK' if reached else 'DID NOT REACH'}  err={errs}", flush=True)
            c.input_vel = 0.0
            time.sleep(0.4)
    finally:
        c.input_vel = 0.0
        a.requested_state = AXIS_STATE_IDLE
        time.sleep(0.2)

    # Direction-symmetry summary.
    fwd = [r["mean_abs_iq_a"] for r in report["runs"] if r["cmd_turns_s"] > 0]
    rev = [r["mean_abs_iq_a"] for r in report["runs"] if r["cmd_turns_s"] < 0]
    if fwd and rev:
        report["fwd_mean_iq_a"] = statistics.fmean(fwd)
        report["rev_mean_iq_a"] = statistics.fmean(rev)
        report["dir_asymmetry_a"] = abs(statistics.fmean(fwd) - statistics.fmean(rev))
    report["status"] = status
    report["post_errors"] = err_tuple(odrv)

    out_path = args.out if os.path.isabs(args.out) else os.path.join(HERE, args.out)
    os.makedirs(os.path.dirname(out_path), exist_ok=True)
    with open(out_path, "w") as f:
        json.dump(report, f, indent=2)

    print("\n=== SUMMARY ===", flush=True)
    if fwd and rev:
        print(f"  fwd mean|Iq| {report['fwd_mean_iq_a']:.2f}A  "
              f"rev mean|Iq| {report['rev_mean_iq_a']:.2f}A  "
              f"asymmetry {report['dir_asymmetry_a']:.2f}A", flush=True)
    print(f"  status: {status}   post-errors: {report['post_errors']}", flush=True)
    print(f"  saved -> {out_path}", flush=True)


if __name__ == "__main__":
    main()
