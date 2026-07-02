#!/usr/bin/env python3
"""
Erase ODrive config, configure DIY Hall motor on axis0, calibrate and test.

Good for iterative magnet-distance experiments: encoder offset scatter is the
key quality metric -- tighter scatter = better magnet placement.

Steps:
  1. Connect, erase_configuration (ODrive reboots)
  2. Reconnect, configure motor + Hall encoder + controller in RAM
  3. Motor calibration (R/L measurement)
  4. Encoder offset calibration x N (scatter tracks magnet distance quality)
  5. Free-spin velocity test
  6. Summary; optionally save to flash with --save

Usage:
  python diy_hall_setup_and_check.py [--pole-pairs 15] [--offset-runs 5]
         [--speeds 5 10] [--current-limit 12] [--no-thermistor] [--save]
"""

import argparse
import os
import statistics
import sys
import time

sys.path = [p for p in sys.path if p not in ("", ".", os.getcwd())]

import odrive
from odrive.enums import (
    AXIS_STATE_CLOSED_LOOP_CONTROL,
    AXIS_STATE_ENCODER_OFFSET_CALIBRATION,
    AXIS_STATE_IDLE,
    AXIS_STATE_MOTOR_CALIBRATION,
    CONTROL_MODE_VELOCITY_CONTROL,
    ENCODER_MODE_HALL,
    INPUT_MODE_VEL_RAMP,
)
MOTOR_TYPE_HIGH_CURRENT = 0  # PMSM_CURRENT_CONTROL in v0.6 package

# MF52 B3950 10k thermistor on GPIO4 (3.3V – 10k – pin – NTC – GND)
# Calibrated 2026-06-24; valid for ~15–130 °C range.
THERM_COEFFS = (-1352.190918, 1590.688110, -684.021851, 141.426056)
THERM_LIMITS = (70.0, 90.0)


def parse_args():
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--pole-pairs", type=int, default=15,
                   help="Motor pole pairs (default 15 for hoverboard DIY)")
    p.add_argument("--offset-runs", type=int, default=5,
                   help="Encoder offset calibration repetitions (default 5)")
    p.add_argument("--speeds", type=float, nargs="+", default=[5.0, 10.0],
                   help="Free-spin test velocities in motor t/s (default 5 10)")
    p.add_argument("--current-limit", type=float, default=12.0,
                   help="Motor current limit A (default 12)")
    p.add_argument("--no-thermistor", action="store_true",
                   help="Skip motor thermistor setup (if not installed)")
    p.add_argument("--save", action="store_true",
                   help="Save configuration to flash after setup")
    return p.parse_args()


def wait_idle(axis, timeout=30):
    deadline = time.monotonic() + timeout
    while axis.current_state != AXIS_STATE_IDLE:
        time.sleep(0.2)
        if time.monotonic() > deadline:
            return False
    return True


def clear_errors(axis):
    axis.motor.error = 0
    axis.encoder.error = 0
    axis.controller.error = 0
    axis.error = 0


def errs(axis):
    return (int(axis.error), int(axis.motor.error),
            int(axis.encoder.error), int(axis.controller.error))


def reconnect(label, timeout=25):
    print(f"Reconnecting ({label})...", end=" ", flush=True)
    dev = odrive.find_any(timeout=timeout)
    print(f"OK  vbus={dev.vbus_voltage:.1f}V", flush=True)
    return dev


def main():
    args = parse_args()
    pole_pairs = args.pole_pairs
    cpr = 6 * pole_pairs

    # ── 1. Connect and erase ──────────────────────────────────────────────────
    print("Connecting...", flush=True)
    dev = odrive.find_any(timeout=20)
    print(f"Connected: {format(dev.serial_number, 'x')}  vbus={dev.vbus_voltage:.1f}V",
          flush=True)

    print("\n--- Erasing configuration ---", flush=True)
    try:
        dev.erase_configuration()
    except Exception:
        pass  # ODrive reboots; connection drops normally
    time.sleep(3)

    dev = reconnect("post-erase")
    axis = dev.axis0
    m = axis.motor
    e = axis.encoder
    c = axis.controller

    # ── 2. Configure in RAM ───────────────────────────────────────────────────
    print("\n--- Configuring motor + Hall encoder (RAM) ---", flush=True)

    m.config.motor_type = MOTOR_TYPE_HIGH_CURRENT
    m.config.pole_pairs = pole_pairs
    m.config.calibration_current = 5.0
    m.config.resistance_calib_max_voltage = 4.0
    m.config.torque_constant = 0.194
    m.config.current_lim = args.current_limit

    e.config.mode = ENCODER_MODE_HALL
    e.config.cpr = cpr
    e.config.calib_range = 0.05
    e.config.bandwidth = 20.0
    e.config.ignore_illegal_hall_state = False

    c.config.control_mode = CONTROL_MODE_VELOCITY_CONTROL
    c.config.input_mode = INPUT_MODE_VEL_RAMP
    c.config.vel_gain = 0.10
    c.config.vel_integrator_gain = 0.02
    c.config.vel_limit = max(args.speeds) * 1.5
    c.config.vel_ramp_rate = 1.0
    c.config.enable_overspeed_error = False
    c.config.load_encoder_axis = 0
    c.config.vel_encoder_axis = 0

    if not args.no_thermistor:
        t = axis.motor_thermistor.config
        t.gpio_pin = 4
        t.poly_coefficient_0, t.poly_coefficient_1 = THERM_COEFFS[0], THERM_COEFFS[1]
        t.poly_coefficient_2, t.poly_coefficient_3 = THERM_COEFFS[2], THERM_COEFFS[3]
        t.temp_limit_lower, t.temp_limit_upper = THERM_LIMITS
        t.enabled = True
        print(f"Motor thermistor: {axis.motor_thermistor.temperature:.1f}°C", flush=True)

    print(f"pole_pairs={pole_pairs}  cpr={cpr}  Hall  current_lim={args.current_limit}A",
          flush=True)

    # ── 3. Motor calibration ──────────────────────────────────────────────────
    print("\n--- Motor calibration (R/L) ---", flush=True)
    clear_errors(axis)
    axis.requested_state = AXIS_STATE_MOTOR_CALIBRATION
    if not wait_idle(axis, 15):
        print("TIMEOUT waiting for motor calibration", flush=True)
        return
    e_motor = (int(axis.error), int(axis.motor.error))
    print(f"R={m.config.phase_resistance:.4f} Ω  L={m.config.phase_inductance * 1e3:.4f} mH  "
          f"calibrated={m.is_calibrated}  errs={e_motor}", flush=True)
    if e_motor[0] or e_motor[1]:
        print("MOTOR CALIBRATION FAILED — check wiring and poles", flush=True)
        return
    m.config.pre_calibrated = True

    # ── 4. Encoder offset calibration (scatter = magnet distance metric) ──────
    print(f"\n--- Encoder offset calibration x{args.offset_runs} "
          f"(scatter tells you magnet distance quality) ---", flush=True)
    offs = []
    for k in range(args.offset_runs):
        clear_errors(axis)
        axis.requested_state = AXIS_STATE_ENCODER_OFFSET_CALIBRATION
        time.sleep(0.3)
        if not wait_idle(axis, 25):
            print(f"  run{k+1}: TIMEOUT", flush=True)
            continue
        e_enc = (int(axis.error), int(axis.encoder.error))
        off = e.config.offset_float
        d = m.config.direction
        ok = e_enc[0] == 0 and e_enc[1] == 0
        if ok:
            offs.append(off)
        print(f"  run{k+1}: offset_float={off:.5f}  motor.dir={d:+d}  "
              f"{'OK' if ok else f'ERR {e_enc}'}",
              flush=True)

    scatter_qual = "n/a"
    spread = None
    if len(offs) >= 2:
        spread = max(offs) - min(offs)
        mean_off = statistics.fmean(offs)
        if spread < 0.10:
            scatter_qual = "EXCELLENT"
        elif spread < 0.25:
            scatter_qual = "OK"
        else:
            scatter_qual = "POOR  (magnets probably too far or not centred)"
        print(f"  spread={spread:.4f} rad  mean={mean_off:.5f}  → {scatter_qual}", flush=True)

    if not offs:
        print("ALL ENCODER OFFSET CALIBRATIONS FAILED — check Hall wiring", flush=True)
        return

    e.config.pre_calibrated = True

    # ── 5. Free-spin test ─────────────────────────────────────────────────────
    print(f"\n--- Free-spin: {args.speeds} motor t/s ---", flush=True)
    clear_errors(axis)
    c.input_vel = 0.0
    axis.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
    time.sleep(0.3)
    spin_ok = True
    if axis.current_state != AXIS_STATE_CLOSED_LOOP_CONTROL:
        print(f"CLOSED-LOOP ENTRY FAILED  errs={errs(axis)}", flush=True)
        spin_ok = False
    else:
        try:
            for spd in args.speeds:
                c.input_vel = spd
                time.sleep(1.2)
                iqs, vels = [], []
                for _ in range(25):
                    time.sleep(0.02)
                    iqs.append(abs(m.current_control.Iq_measured))
                    vels.append(e.vel_estimate)
                mean_iq = statistics.fmean(iqs)
                mean_vel = statistics.fmean(vels)
                reached = abs(mean_vel) >= 0.8 * spd
                if not reached:
                    spin_ok = False
                print(f"  {spd:4.0f} t/s → actual={mean_vel:5.2f} t/s  "
                      f"|Iq|={mean_iq:.2f}A  max={max(iqs):.2f}A  "
                      f"{'OK' if reached else 'DID NOT REACH'}",
                      flush=True)
                c.input_vel = 0.0
                time.sleep(0.4)
        finally:
            c.input_vel = 0.0
            axis.requested_state = AXIS_STATE_IDLE
            time.sleep(0.3)
    post_errs = errs(axis)
    print(f"  final errors: {post_errs}", flush=True)

    # ── Summary ───────────────────────────────────────────────────────────────
    print("\n=== SUMMARY ===", flush=True)
    print(f"  motor calibration:    {'PASS' if m.is_calibrated else 'FAIL'}", flush=True)
    if spread is not None:
        print(f"  offset scatter:       {spread:.4f} rad / {len(offs)} runs  → {scatter_qual}",
              flush=True)
    else:
        print(f"  offset scatter:       {scatter_qual}", flush=True)
    print(f"  free-spin:            {'PASS' if spin_ok else 'FAIL'}", flush=True)
    print(f"  final errors:         {post_errs}", flush=True)

    if args.save:
        print("\nSaving to flash...", flush=True)
        try:
            dev.save_configuration()
        except Exception:
            pass
        time.sleep(3)
        dev = reconnect("post-save")
        axis = dev.axis0
        print(f"Verified: R={axis.motor.config.phase_resistance:.4f} Ω  "
              f"calibrated={axis.motor.is_calibrated}  "
              f"enc_mode={axis.encoder.config.mode}",
              flush=True)
        print("Configuration saved.", flush=True)
    else:
        print("\n(RAM only — not saved. Add --save to write to flash.)", flush=True)


if __name__ == "__main__":
    main()
