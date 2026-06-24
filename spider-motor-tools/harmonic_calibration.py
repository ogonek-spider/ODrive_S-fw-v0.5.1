#!/usr/bin/env python3
"""
Offline encoder harmonic-compensation calibration for an ODrive S joint.

Magnetic absolute encoders (onboard AS5047P, joint-side MT6701) have a
systematic angle error that repeats once and twice per *mechanical* revolution:
a 1st harmonic from magnet eccentricity and a 2nd harmonic from tilt / field
nonlinearity. This tool measures those four Fourier coefficients and (optionally)
writes them to the encoder config so the firmware can subtract the error from
the position estimate and the electrical phase.

How it works
------------
Spin the motor at a slow *constant* velocity. The true mechanical angle then
advances linearly in time, so any deviation of the raw encoder reading from a
straight line *is* the encoder error. We sample the raw, uncompensated counts
(`shadow_count`, `count_in_cpr`), detrend `shadow_count` against time to get the
error in counts, and least-squares fit

    err(theta) = c1*cos(theta) + s1*sin(theta) + c2*cos(2*theta) + s2*sin(2*theta)

with theta = 2*pi*count_in_cpr/cpr -- exactly the variable the firmware uses in
`Encoder::update()`. The fitted coefficients are the config fields
`harmonic_cos_1/sin_1/cos_2/sin_2` (in counts); firmware subtracts them.

Calibration must cover whole mechanical revolutions of the *encoder being
calibrated*:
  * axis0 AS5047P sits on the motor shaft -> one motor turn == one encoder turn,
    so a few motor turns is plenty.
  * axis1 MT6701 sits after the ~34:1 gearbox -> one encoder turn needs ~34
    motor turns AND the joint must be free to rotate through full output turns.
    If the leg is range-limited, decouple it first or this fit will be poor.

Safety: config changes are RAM-only unless you pass --save. Nothing is spun
faster than --speed; the firmware over-temp/encoder guards stay active. The
motor must already be calibrated and commutating on --drive-axis.

NOTE: tools live alongside a local `odrive/` package dir in the firmware repo
that can shadow the installed package. This script strips cwd from sys.path.
"""

import argparse
import json
import math
import os
import sys
import time

# Avoid shadowing the installed `odrive` package with a local ./odrive dir.
sys.path = [p for p in sys.path if p not in ("", ".", os.getcwd())]

import numpy as np
import odrive
from odrive.enums import (
    AXIS_STATE_CLOSED_LOOP_CONTROL,
    AXIS_STATE_IDLE,
    CONTROL_MODE_VELOCITY_CONTROL,
    INPUT_MODE_PASSTHROUGH,
)


def parse_args():
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--motor-id", help="Motor number/label (e.g. 7). Recorded and "
                                      "used to auto-name the JSON report.")
    p.add_argument("--serial-number")
    p.add_argument("--drive-axis", type=int, default=0, choices=(0, 1),
                   help="Axis whose MOTOR is spun. Default 0.")
    p.add_argument("--encoder-axis", type=int, default=None, choices=(0, 1),
                   help="Axis whose ENCODER is calibrated. Default = --drive-axis "
                        "(use 1 to calibrate the joint-side MT6701 driven via the "
                        "gearbox on axis0).")
    p.add_argument("--speed", type=float, default=12.0,
                   help="Constant motor velocity during the sweep (motor turns/s). "
                        "Default 12. Spin FAST: at low speed cogging-induced "
                        "velocity ripple swamps the true (fixed) encoder error and "
                        "the fit is not repeatable. Inertia filters cogging at "
                        "speed, leaving the speed-independent encoder harmonic.")
    p.add_argument("--averages", type=int, default=4,
                   help="Number of constant-velocity passes to average. Default 4. "
                        "Pass-to-pass spread is reported as a repeatability check.")
    p.add_argument("--current-limit", type=float, default=12.0,
                   help="Motor current limit during the sweep (A). Default 12.")
    p.add_argument("--enc-revs", type=float, default=8.0,
                   help="Target number of revolutions of the CALIBRATED encoder to "
                        "cover. Sweep duration is derived from this, --speed and the "
                        "(motor:encoder) ratio. Default 8.")
    p.add_argument("--ratio", type=float, default=1.0,
                   help="Motor turns per turn of the calibrated encoder. 1 for an "
                        "on-shaft encoder (axis0 AS5047P); ~34 for the joint MT6701 "
                        "after the gearbox. Default 1.")
    p.add_argument("--apply", action="store_true",
                   help="Write the fitted coefficients + enable flag to the encoder "
                        "config in RAM (not flash).")
    p.add_argument("--save", action="store_true",
                   help="Also save_configuration() to flash. Implies --apply.")
    p.add_argument("--json", help="Summary JSON path "
                                  "(default reports/harmonic-cal-<id>-axis<n>.json).")
    return p.parse_args()


def err_tuple(axis):
    return (int(axis.error), int(axis.motor.error),
            int(axis.encoder.error), int(axis.controller.error))


def clear_errors(axis):
    axis.error = 0
    axis.motor.error = 0
    axis.encoder.error = 0
    axis.controller.error = 0


def fit_harmonics(theta, err_counts):
    """Least-squares fit err = c1 cos t + s1 sin t + c2 cos 2t + s2 sin 2t (+DC).

    Returns (coeffs dict, dc, residual_after) all in counts.
    """
    A = np.column_stack([
        np.cos(theta), np.sin(theta),
        np.cos(2.0 * theta), np.sin(2.0 * theta),
        np.ones_like(theta),
    ])
    x, *_ = np.linalg.lstsq(A, err_counts, rcond=None)
    fit = A @ x
    coeffs = {
        "harmonic_cos_1": float(x[0]),
        "harmonic_sin_1": float(x[1]),
        "harmonic_cos_2": float(x[2]),
        "harmonic_sin_2": float(x[3]),
    }
    return coeffs, float(x[4]), err_counts - (fit - x[4])


def main():
    args = parse_args()
    if args.save:
        args.apply = True
    enc_axis_n = args.encoder_axis if args.encoder_axis is not None else args.drive_axis

    print("Connecting to ODrive...", flush=True)
    dev = odrive.find_any(serial_number=args.serial_number, timeout=20)
    drive = getattr(dev, f"axis{args.drive_axis}")
    enc_axis = getattr(dev, f"axis{enc_axis_n}")
    m = drive.motor
    c = drive.controller
    e = enc_axis.encoder
    cpr = int(e.config.cpr)
    pole_pairs = int(m.config.pole_pairs)

    if not (int(e.config.mode) & 0x100):
        print(f"WARNING: axis{enc_axis_n} encoder mode {int(e.config.mode)} is not "
              f"absolute. Harmonic comp targets magnetic absolute encoders.",
              flush=True)
    if not e.is_ready:
        print(f"REFUSING TO RUN: axis{enc_axis_n} encoder is not ready.", flush=True)
        sys.exit(2)

    enc_revs = max(args.enc_revs, 0.5)
    motor_turns = enc_revs * args.ratio
    duration = motor_turns / max(abs(args.speed), 1e-3)

    fw = getattr(odrive, "__version__", "?")
    print(f"serial={format(dev.serial_number, 'x')} fw={fw} "
          f"vbus={float(dev.vbus_voltage):.1f}V", flush=True)
    print(f"drive axis{args.drive_axis} motor -> calibrate axis{enc_axis_n} encoder "
          f"(cpr={cpr}, pole_pairs={pole_pairs})", flush=True)
    print(f"spin {args.speed:.2f} motor t/s for {duration:.1f}s "
          f"(~{enc_revs:.1f} encoder revs at ratio {args.ratio:g})", flush=True)

    # Save the RAM config we touch; restored in finally.
    saved = (m.config.current_lim, c.config.control_mode, c.config.input_mode,
             c.config.vel_limit, drive.config.enable_watchdog,
             bool(e.config.enable_harmonic_compensation))

    def counts_to_mech_deg(counts):
        return counts / cpr * 360.0

    def collect_one_pass():
        """Sample count_in_cpr at constant velocity for `duration`, detrend,
        and fit. Returns (coeffs, stats) or (None, reason)."""
        samples = []
        t_start = time.monotonic()
        deadline = t_start + duration
        while True:
            now = time.monotonic()
            # Single field per sample: count_in_cpr_ is the latched instantaneous
            # absolute count (no PLL lag, no inter-read skew). Unwrapped in
            # software to recover the continuous angle for detrending.
            samples.append((now - t_start, int(e.count_in_cpr)))
            errs = err_tuple(drive)
            if any(errs):
                return None, f"axis fault errs={errs}"
            if drive.current_state != AXIS_STATE_CLOSED_LOOP_CONTROL:
                return None, f"left closed-loop (state={drive.current_state})"
            if now >= deadline:
                break
            # A small delay keeps USB load from starving the motor control loop
            # (CONTROL_DEADLINE_MISSED) while still giving fine angular sampling:
            # at 12 t/s, 1 ms is ~16 counts, ~33 samples/rev.
            time.sleep(0.001)
        t = np.array([s[0] for s in samples])
        cic = np.array([s[1] for s in samples], dtype=float)
        # Unwrap into a continuous angle (counts); at constant velocity the
        # straight line is the true angle and the residual is the encoder error.
        # theta uses the raw (wrapped) count_in_cpr -- the exact variable the
        # firmware feeds the harmonic correction.
        unwrapped = np.unwrap(2.0 * math.pi * cic / cpr) / (2.0 * math.pi) * cpr
        slope, intercept = np.polyfit(t, unwrapped, 1)
        err_counts = unwrapped - (slope * t + intercept)
        theta = 2.0 * math.pi * cic / cpr
        coeffs, dc, resid = fit_harmonics(theta, err_counts)
        stats = {
            "n": len(samples),
            "covered_revs": abs(unwrapped[-1] - unwrapped[0]) / cpr,
            "pre_rms": float(np.std(err_counts)),
            "post_rms": float(np.std(resid)),
            "pre_pp": float(err_counts.max() - err_counts.min()),
            "post_pp": float(resid.max() - resid.min()),
        }
        return coeffs, stats

    n_avg = max(1, args.averages)
    stop_reason = "ok"
    passes = []  # (coeffs, stats)
    t0 = time.monotonic()
    try:
        clear_errors(drive)
        drive.config.enable_watchdog = False
        # Measure the RAW encoder: turn compensation off during the sweep.
        e.config.enable_harmonic_compensation = False
        m.config.current_lim = args.current_limit
        c.config.vel_limit = abs(args.speed) * 1.4
        c.config.control_mode = CONTROL_MODE_VELOCITY_CONTROL
        c.config.input_mode = INPUT_MODE_PASSTHROUGH
        c.input_vel = 0.0
        drive.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
        time.sleep(0.3)
        if drive.current_state != AXIS_STATE_CLOSED_LOOP_CONTROL:
            stop_reason = f"closed-loop entry failed errs={err_tuple(drive)}"
            print("FAIL:", stop_reason, flush=True)
            return
        c.input_vel = args.speed
        time.sleep(max(1.0, 1.5 * abs(args.speed) / 10.0))  # let velocity settle

        for p in range(n_avg):
            coeffs, stats = collect_one_pass()
            if coeffs is None:
                stop_reason = stats
                break
            passes.append((coeffs, stats))
            a1 = math.hypot(coeffs["harmonic_cos_1"], coeffs["harmonic_sin_1"])
            a2 = math.hypot(coeffs["harmonic_cos_2"], coeffs["harmonic_sin_2"])
            print(f"  pass {p+1}/{n_avg}: 1st amp={a1:5.1f} 2nd amp={a2:5.1f} cts  "
                  f"RMS {stats['pre_rms']:.1f}->{stats['post_rms']:.1f} cts  "
                  f"({stats['covered_revs']:.1f} revs)", flush=True)
    finally:
        c.input_vel = 0.0
        drive.requested_state = AXIS_STATE_IDLE
        time.sleep(0.3)
        (m.config.current_lim, c.config.control_mode, c.config.input_mode,
         c.config.vel_limit, drive.config.enable_watchdog,
         restore_comp) = saved

    if not passes:
        print(f"ABORTED before enough data ({stop_reason}).", flush=True)
        sys.exit(1)

    # Average the coefficients across passes; spread across passes is the
    # repeatability check (a true encoder error is speed- and pass-stable).
    keys = ("harmonic_cos_1", "harmonic_sin_1", "harmonic_cos_2", "harmonic_sin_2")
    coeffs = {k: float(np.mean([p[0][k] for p in passes])) for k in keys}
    spread = {k: float(np.std([p[0][k] for p in passes])) for k in keys}
    samples_n = sum(p[1]["n"] for p in passes)
    covered_revs = float(np.mean([p[1]["covered_revs"] for p in passes]))
    pre_rms = float(np.mean([p[1]["pre_rms"] for p in passes]))
    post_rms = float(np.mean([p[1]["post_rms"] for p in passes]))
    pre_pp = float(np.mean([p[1]["pre_pp"] for p in passes]))
    post_pp = float(np.mean([p[1]["post_pp"] for p in passes]))
    amp1 = math.hypot(coeffs["harmonic_cos_1"], coeffs["harmonic_sin_1"])
    amp2 = math.hypot(coeffs["harmonic_cos_2"], coeffs["harmonic_sin_2"])
    amp1_spread = math.hypot(spread["harmonic_cos_1"], spread["harmonic_sin_1"])
    amp2_spread = math.hypot(spread["harmonic_cos_2"], spread["harmonic_sin_2"])

    on_shaft = (enc_axis_n == args.drive_axis and args.ratio == 1.0)
    elec_note = ""
    if on_shaft:
        elec_note = (f" | 1st-harmonic electrical error "
                     f"~{counts_to_mech_deg(amp1) * pole_pairs:.1f} deg "
                     f"(x{pole_pairs} pole-pairs)")

    print(f"\n=== HARMONIC FIT (avg of {len(passes)} pass(es) @ {args.speed:g} t/s) ===",
          flush=True)
    print(f"  samples         : {samples_n} total, ~{covered_revs:.1f} revs/pass",
          flush=True)
    if covered_revs < 2.0:
        print("  WARNING: <2 encoder revolutions/pass; fit is unreliable. "
              "Increase --enc-revs / set --ratio for a geared encoder.", flush=True)
    print(f"  1st harmonic    : cos={coeffs['harmonic_cos_1']:+.2f}  "
          f"sin={coeffs['harmonic_sin_1']:+.2f}  "
          f"amp={amp1:.2f} cts = {counts_to_mech_deg(amp1):.3f} deg mech "
          f"(pass spread +-{amp1_spread:.1f} cts)", flush=True)
    print(f"  2nd harmonic    : cos={coeffs['harmonic_cos_2']:+.2f}  "
          f"sin={coeffs['harmonic_sin_2']:+.2f}  "
          f"amp={amp2:.2f} cts = {counts_to_mech_deg(amp2):.3f} deg mech "
          f"(pass spread +-{amp2_spread:.1f} cts)", flush=True)
    print(f"  error RMS       : {pre_rms:.2f} -> {post_rms:.2f} cts  "
          f"({counts_to_mech_deg(pre_rms):.3f} -> {counts_to_mech_deg(post_rms):.3f} deg)",
          flush=True)
    print(f"  error pk-pk     : {pre_pp:.2f} -> {post_pp:.2f} cts  "
          f"({counts_to_mech_deg(pre_pp):.3f} -> {counts_to_mech_deg(post_pp):.3f} deg)"
          f"{elec_note}", flush=True)
    if max(amp1_spread, amp2_spread) > 0.25 * max(amp1, amp2, 1.0):
        print("  WARNING: coefficients vary >25% across passes -- the signal is "
              "contaminated by velocity ripple/cogging, not a fixed encoder error. "
              "Spin FASTER (inertia filters cogging) for a trustworthy fit.",
              flush=True)

    report = {
        "test": "encoder harmonic compensation calibration",
        "motor_id": args.motor_id,
        "serial_number": format(dev.serial_number, "x"),
        "fw_version": fw,
        "drive_axis": args.drive_axis,
        "encoder_axis": enc_axis_n,
        "cpr": cpr,
        "pole_pairs": pole_pairs,
        "params": {
            "speed_turns_s": args.speed,
            "ratio": args.ratio,
            "enc_revs_target": args.enc_revs,
            "current_limit_a": args.current_limit,
            "averages": n_avg,
        },
        "passes": len(passes),
        "samples_n": samples_n,
        "covered_encoder_revs": round(covered_revs, 3),
        "coefficients_counts": coeffs,
        "coefficient_pass_spread_counts": spread,
        "error_rms_counts": {"before": round(pre_rms, 3), "after": round(post_rms, 3)},
        "error_pkpk_counts": {"before": round(pre_pp, 3), "after": round(post_pp, 3)},
        "harmonic_amp_counts": {"first": round(amp1, 3), "second": round(amp2, 3)},
        "stop_reason": stop_reason,
        "applied": bool(args.apply),
        "saved_to_flash": bool(args.save),
    }

    if args.apply:
        e.config.harmonic_cos_1 = coeffs["harmonic_cos_1"]
        e.config.harmonic_sin_1 = coeffs["harmonic_sin_1"]
        e.config.harmonic_cos_2 = coeffs["harmonic_cos_2"]
        e.config.harmonic_sin_2 = coeffs["harmonic_sin_2"]
        e.config.enable_harmonic_compensation = True
        print("\n  applied coefficients + enabled compensation (RAM)", flush=True)
        if args.save:
            dev.save_configuration()
            print("  saved to flash", flush=True)
    else:
        # Leave the encoder's compensation flag as it was found.
        e.config.enable_harmonic_compensation = restore_comp
        print("\n  (dry run) to apply to RAM re-run with --apply, "
              "or to flash with --save. Manual apply:", flush=True)
        ax = f"odrv0.axis{enc_axis_n}.encoder.config"
        for k, v in coeffs.items():
            print(f"    {ax}.{k} = {v:.6f}", flush=True)
        print(f"    {ax}.enable_harmonic_compensation = True", flush=True)
        print("    odrv0.save_configuration()", flush=True)

    out_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)), "reports")
    json_path = args.json
    if json_path is None and args.motor_id is not None:
        json_path = os.path.join(
            out_dir, f"harmonic-cal-{args.motor_id}-axis{enc_axis_n}.json")
    if json_path:
        os.makedirs(os.path.dirname(os.path.abspath(json_path)), exist_ok=True)
        with open(json_path, "w", encoding="utf-8") as f:
            json.dump(report, f, indent=2)
            f.write("\n")
        print(f"  report -> {json_path}", flush=True)


if __name__ == "__main__":
    main()
