#!/usr/bin/env python3
"""
Automated ODrive motor health battery.

Runs a non-destructive sequence of checks and prints a PASS/WARN/FAIL summary,
optionally writing a JSON report. NOTHING is ever written to flash
(no save_configuration); all config changes are RAM-only and restored on exit,
so a power cycle returns the drive to its saved state.

Checks
  1. Motor calibration repeatability  -> phase resistance / inductance scatter
  2. Encoder offset calibration repeatability -> commutation offset scatter
  3. Free-spin current sweep          -> can it reach speed, and at what current

A clean encoder + healthy windings but scattered commutation offset points to
the encoder magnet's mechanical coupling (loose / eccentric magnet). High
current that only appears with a gearbox attached points to the gearbox.

Run an encoder signal-integrity check separately with encoder_hand_test.py.

NOTE: tools live alongside a local `odrive/` package dir in the firmware repo
that can shadow the installed package. This script strips cwd from sys.path.
"""

import argparse
import json
import os
import statistics
import sys
import time

# Avoid shadowing the installed `odrive` package with a local ./odrive dir.
sys.path = [p for p in sys.path if p not in ("", ".", os.getcwd())]

import odrive
from odrive.enums import (
    AXIS_STATE_CLOSED_LOOP_CONTROL,
    AXIS_STATE_ENCODER_OFFSET_CALIBRATION,
    AXIS_STATE_IDLE,
    AXIS_STATE_MOTOR_CALIBRATION,
    CONTROL_MODE_VELOCITY_CONTROL,
    INPUT_MODE_PASSTHROUGH,
)

# Pass/warn thresholds (tune per motor family).
R_SPREAD_WARN = 0.02          # ohm, max-min across motor cal runs
L_SPREAD_WARN = 0.10e-3       # H
OFFSET_SPREAD_WARN = 0.10     # rad, max-min across encoder offset cal runs
OFFSET_SPREAD_FAIL = 0.25     # rad
SPEED_REACH_FRAC = 0.80       # actual/commanded velocity to count as "reached"


def parse_args():
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--motor-id", help="Motor number/label for this unit (e.g. 10). "
                                      "Recorded in the report and used to auto-name JSON.")
    p.add_argument("--serial-number")
    p.add_argument("--axis", type=int, default=0, choices=(0, 1))
    p.add_argument("--motorcal-runs", type=int, default=4)
    p.add_argument("--offset-runs", type=int, default=5)
    p.add_argument("--speeds", type=float, nargs="+", default=[5.0, 10.0],
                   help="Free-spin velocities to test (motor turns/s).")
    p.add_argument("--current-limit", type=float, default=10.0,
                   help="Motor current limit during the test (A).")
    p.add_argument("--fet-temp-limit", type=float, default=70.0)
    p.add_argument("--skip-motorcal", action="store_true")
    p.add_argument("--skip-offset", action="store_true")
    p.add_argument("--skip-spin", action="store_true",
                   help="Skip the free-spin current sweep (no closed-loop motion).")
    p.add_argument("--json", help="Write the full report to this JSON path.")
    return p.parse_args()


def wait_idle(axis, timeout):
    deadline = time.monotonic() + timeout
    while axis.current_state != AXIS_STATE_IDLE:
        time.sleep(0.2)
        if time.monotonic() > deadline:
            return False
    return True


def err_tuple(axis):
    return (int(axis.error), int(axis.motor.error),
            int(axis.encoder.error), int(axis.controller.error))


def clear_errors(axis):
    axis.error = 0
    axis.motor.error = 0
    axis.encoder.error = 0
    axis.controller.error = 0


def run_calibration(axis, state, timeout):
    """Clear errors, request a calibration state, confirm it actually started,
    then wait for the return to IDLE.

    Returns (ran, errors): ran is False if the state transition was rejected
    (e.g. a latched error), which must NOT be mistaken for a clean run.
    """
    clear_errors(axis)
    time.sleep(0.05)
    axis.requested_state = state
    ran = False
    deadline = time.monotonic() + 2.0
    while time.monotonic() < deadline:
        time.sleep(0.05)
        if axis.current_state == state:
            ran = True
            break
    wait_idle(axis, timeout)
    return ran, err_tuple(axis)


def motorcal_repeatability(axis, runs):
    m = axis.motor
    rs, ls, results = [], [], []
    print(f"\n=== motor calibration x{runs} (phase R/L) ===", flush=True)
    for k in range(runs):
        ran, errs = run_calibration(axis, AXIS_STATE_MOTOR_CALIBRATION, 15)
        valid = ran and errs[0] == 0 and errs[1] == 0
        r, l = m.config.phase_resistance, m.config.phase_inductance
        if valid:
            rs.append(r); ls.append(l)
        results.append({"run": k + 1, "R_ohm": r, "L_H": l,
                        "calibrated": bool(m.is_calibrated),
                        "errors": errs, "ran": ran, "valid": valid})
        print(f"  run{k+1}: R={r:.4f} ohm  L={l*1e3:.4f} mH  "
              f"{'OK' if valid else 'DID-NOT-RUN' if not ran else 'ERROR'}  "
              f"errs={errs}", flush=True)
    if len(rs) < 2:
        return {"status": "ERROR", "reason": "fewer than 2 valid runs",
                "valid_runs": len(rs), "runs": results}
    r_spread = max(rs) - min(rs)
    l_spread = max(ls) - min(ls)
    status = "PASS"
    if r_spread > R_SPREAD_WARN or l_spread > L_SPREAD_WARN:
        status = "WARN"
    if len(rs) < runs:
        status = "WARN"
    return {"status": status, "R_spread_ohm": r_spread, "L_spread_H": l_spread,
            "R_mean_ohm": statistics.fmean(rs), "L_mean_H": statistics.fmean(ls),
            "valid_runs": len(rs), "runs": results}


def offset_repeatability(axis, runs):
    e = axis.encoder
    offs, results = [], []
    print(f"\n=== encoder offset calibration x{runs} (commutation offset) ===", flush=True)
    for k in range(runs):
        ran, errs = run_calibration(axis, AXIS_STATE_ENCODER_OFFSET_CALIBRATION, 25)
        valid = ran and errs[0] == 0 and errs[2] == 0
        off = e.config.offset_float
        if valid:
            offs.append(off)
        results.append({"run": k + 1, "offset_float": off,
                        "motor_direction": axis.motor.config.direction,
                        "errors": errs, "ran": ran, "valid": valid})
        print(f"  run{k+1}: offset_float={off:.5f}  motor.dir={axis.motor.config.direction}  "
              f"{'OK' if valid else 'DID-NOT-RUN' if not ran else 'ERROR'}  "
              f"errs={errs}", flush=True)
    if len(offs) < 2:
        return {"status": "ERROR", "reason": "fewer than 2 valid runs",
                "valid_runs": len(offs), "runs": results}
    spread = max(offs) - min(offs)
    status = "PASS"
    if spread > OFFSET_SPREAD_FAIL:
        status = "FAIL"
    elif spread > OFFSET_SPREAD_WARN:
        status = "WARN"
    if len(offs) < runs and status == "PASS":
        status = "WARN"
    return {"status": status, "offset_spread_rad": spread,
            "offset_mean_rad": statistics.fmean(offs),
            "offset_min": min(offs), "offset_max": max(offs),
            "valid_runs": len(offs), "runs": results}


def freespin(axis, speeds, fet_limit):
    e = axis.encoder; m = axis.motor; c = axis.controller
    results = []
    print("\n=== free-spin current sweep ===", flush=True)
    clear_errors(axis)
    c.input_vel = 0.0
    axis.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
    time.sleep(0.2)
    if axis.current_state != AXIS_STATE_CLOSED_LOOP_CONTROL:
        return {"status": "FAIL", "reason": "closed-loop entry failed",
                "errors": err_tuple(axis), "runs": results}
    status = "PASS"
    try:
        for spd in speeds:
            c.input_vel = spd
            time.sleep(1.0)
            iqs, vels = [], []
            for _ in range(25):
                time.sleep(0.02)
                iqs.append(abs(m.current_control.Iq_measured))
                vels.append(e.vel_estimate)
                if axis.fet_thermistor.temperature > fet_limit:
                    raise RuntimeError("FET temperature limit hit during spin")
            mean_iq = statistics.fmean(iqs); max_iq = max(iqs)
            mean_vel = statistics.fmean(vels)
            reached = abs(mean_vel) >= SPEED_REACH_FRAC * spd
            if not reached:
                status = "FAIL"
            results.append({"cmd_turns_s": spd, "mean_abs_iq_a": mean_iq,
                            "max_abs_iq_a": max_iq, "actual_vel_turns_s": mean_vel,
                            "reached": reached})
            print(f"  cmd {spd:.0f} t/s: mean|Iq|={mean_iq:.2f}A max={max_iq:.2f}A  "
                  f"actual={mean_vel:.2f} t/s  {'OK' if reached else 'DID NOT REACH'}",
                  flush=True)
            c.input_vel = 0.0
            time.sleep(0.3)
    finally:
        c.input_vel = 0.0
        axis.requested_state = AXIS_STATE_IDLE
        time.sleep(0.2)
    return {"status": status, "runs": results}


def main():
    args = parse_args()
    print("Connecting to ODrive...", flush=True)
    dev = odrive.find_any(serial_number=args.serial_number, timeout=20)
    axis = getattr(dev, f"axis{args.axis}")
    m = axis.motor; e = axis.encoder; c = axis.controller

    fw = getattr(odrive, "__version__", "?")
    report = {
        "motor_id": args.motor_id,
        "serial_number": format(dev.serial_number, "x"),
        "fw_version": fw,
        "axis": args.axis,
        "vbus_v": float(dev.vbus_voltage),
        "fet_temp_start_c": float(axis.fet_thermistor.temperature),
        "stored": {
            "phase_resistance_ohm": m.config.phase_resistance,
            "phase_inductance_H": m.config.phase_inductance,
            "pole_pairs": m.config.pole_pairs,
            "torque_constant": m.config.torque_constant,
            "motor_direction": m.config.direction,
            "encoder_mode": e.config.mode,
            "encoder_cpr": e.config.cpr,
            "encoder_direction": e.config.direction,
            "offset_float": e.config.offset_float,
        },
        "pre_errors": err_tuple(axis),
    }
    if args.motor_id is not None:
        print(f"MOTOR #{args.motor_id}", flush=True)
    print(f"serial={report['serial_number']} fw={fw} vbus={report['vbus_v']:.1f}V "
          f"FET={report['fet_temp_start_c']:.1f}C", flush=True)
    print(f"stored: R={m.config.phase_resistance:.4f} ohm  "
          f"L={m.config.phase_inductance*1e3:.4f} mH  pole_pairs={m.config.pole_pairs}  "
          f"enc_mode={e.config.mode} cpr={e.config.cpr}", flush=True)

    # Save RAM config we touch; restored in finally.
    saved = (m.config.current_lim, c.config.control_mode, c.config.input_mode,
             c.config.vel_limit, axis.config.enable_watchdog)
    if any(report["pre_errors"]):
        print(f"NOTE: pre-existing errors {report['pre_errors']} cleared before testing",
              flush=True)
    try:
        clear_errors(axis)
        axis.config.enable_watchdog = False
        m.config.current_lim = args.current_limit
        c.config.vel_limit = max(args.speeds) * 1.4 if args.speeds else 12.0
        c.config.control_mode = CONTROL_MODE_VELOCITY_CONTROL
        c.config.input_mode = INPUT_MODE_PASSTHROUGH
        c.input_vel = 0.0

        if not args.skip_motorcal:
            report["motor_calibration"] = motorcal_repeatability(axis, args.motorcal_runs)
        if not args.skip_offset:
            report["encoder_offset"] = offset_repeatability(axis, args.offset_runs)
        if not args.skip_spin:
            report["free_spin"] = freespin(axis, args.speeds, args.fet_temp_limit)
    finally:
        c.input_vel = 0.0
        axis.requested_state = AXIS_STATE_IDLE
        time.sleep(0.3)
        (m.config.current_lim, c.config.control_mode, c.config.input_mode,
         c.config.vel_limit, axis.config.enable_watchdog) = saved
        report["fet_temp_end_c"] = float(axis.fet_thermistor.temperature)
        report["post_errors"] = err_tuple(axis)

    # Summary
    print("\n=== SUMMARY ===", flush=True)
    overall = "PASS"
    for key, label in (("motor_calibration", "windings (R/L)"),
                       ("encoder_offset", "commutation offset"),
                       ("free_spin", "free-spin")):
        sec = report.get(key)
        if not sec:
            continue
        st = sec["status"]
        if st in ("FAIL", "ERROR"):
            overall = "FAIL"
        elif st == "WARN" and overall != "FAIL":
            overall = "WARN"
        print(f"  {label:<22} {st}", flush=True)
    eo = report.get("encoder_offset")
    if eo and "offset_spread_rad" in eo:
        print(f"    offset spread = {eo['offset_spread_rad']:.3f} rad "
              f"(WARN>{OFFSET_SPREAD_WARN}, FAIL>{OFFSET_SPREAD_FAIL})", flush=True)
    report["overall"] = overall
    label = f"motor #{args.motor_id} " if args.motor_id is not None else ""
    print(f"  {label}OVERALL: {overall}", flush=True)
    print("  (config changes were RAM-only; nothing saved to flash)", flush=True)

    out_path = args.json
    if out_path is None and args.motor_id is not None:
        out_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)), "reports")
        os.makedirs(out_dir, exist_ok=True)
        out_path = os.path.join(out_dir, f"motor-{args.motor_id}-health.json")
    if out_path:
        with open(out_path, "w", encoding="utf-8") as f:
            json.dump(report, f, indent=2)
            f.write("\n")
        print(f"  report -> {out_path}", flush=True)


if __name__ == "__main__":
    main()
