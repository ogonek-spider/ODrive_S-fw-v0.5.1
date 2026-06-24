#!/usr/bin/env python3
"""
No-load thermal-rise test for an ODrive S joint motor.

Spins a *free* motor (no gearbox, no load) at a fixed velocity and logs how the
winding temperature (motor thermistor) climbs over time, alongside the FET
temperature and phase current. Stops when the motor reaches a target temp, a
time cap, or any fault/over-temp trip. Produces a temperature-vs-time curve so
you can characterise the no-load self-heating signature of a motor.

Safety: the firmware motor over-temp guard (motor_thermistor limits) stays
active throughout, so this can only run as hot as the flashed limit allows.
The target temp defaults well below the limit. Config changes are RAM-only and
restored on exit; nothing is written to flash (no save_configuration), so a
power cycle returns the drive to its saved state.

NOTE: tools live alongside a local `odrive/` package dir in the firmware repo
that can shadow the installed package. This script strips cwd from sys.path.
"""

import argparse
import json
import os
import sys
import time

# Avoid shadowing the installed `odrive` package with a local ./odrive dir.
sys.path = [p for p in sys.path if p not in ("", ".", os.getcwd())]

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
                                      "used to auto-name the JSON/JSONL report.")
    p.add_argument("--serial-number")
    p.add_argument("--axis", type=int, default=0, choices=(0, 1))
    p.add_argument("--speed", type=float, default=10.0,
                   help="Free-spin velocity (motor turns/s). Default 10.")
    p.add_argument("--current-limit", type=float, default=12.0,
                   help="Motor current limit during the spin (A). Default 12.")
    p.add_argument("--target-temp", type=float, default=55.0,
                   help="Stop once the MOTOR thermistor reaches this temp (C). "
                        "Default 55 (kept below the flashed trip).")
    p.add_argument("--max-minutes", type=float, default=30.0,
                   help="Hard time cap (minutes). Default 30.")
    p.add_argument("--interval", type=float, default=2.0,
                   help="Sample/log interval (seconds). Default 2.")
    p.add_argument("--json", help="Summary JSON path (default reports/motor-<id>-thermal-rise.json).")
    p.add_argument("--jsonl", help="Per-sample JSONL log path (default alongside the JSON).")
    return p.parse_args()


def err_tuple(axis):
    return (int(axis.error), int(axis.motor.error),
            int(axis.encoder.error), int(axis.controller.error))


def clear_errors(axis):
    axis.error = 0
    axis.motor.error = 0
    axis.encoder.error = 0
    axis.controller.error = 0


def main():
    args = parse_args()
    print("Connecting to ODrive...", flush=True)
    dev = odrive.find_any(serial_number=args.serial_number, timeout=20)
    axis = getattr(dev, f"axis{args.axis}")
    m = axis.motor
    e = axis.encoder
    c = axis.controller
    mt = axis.motor_thermistor
    ft = axis.fet_thermistor

    if not mt.config.enabled:
        print("REFUSING TO RUN: motor thermistor is NOT enabled — no winding "
              "over-temp guard. Enable it before a thermal test.", flush=True)
        sys.exit(2)

    fw = getattr(odrive, "__version__", "?")
    t_start_motor = float(mt.temperature)
    t_start_fet = float(ft.temperature)
    report = {
        "test": "no-load thermal rise (free motor, no gearbox, no load)",
        "motor_id": args.motor_id,
        "serial_number": format(dev.serial_number, "x"),
        "fw_version": fw,
        "axis": args.axis,
        "vbus_v": float(dev.vbus_voltage),
        "params": {
            "speed_turns_s": args.speed,
            "current_limit_a": args.current_limit,
            "target_temp_c": args.target_temp,
            "max_minutes": args.max_minutes,
            "interval_s": args.interval,
        },
        "stored": {
            "phase_resistance_ohm": m.config.phase_resistance,
            "pole_pairs": m.config.pole_pairs,
            "torque_constant": m.config.torque_constant,
            "motor_temp_limit_lower": mt.config.temp_limit_lower,
            "motor_temp_limit_upper": mt.config.temp_limit_upper,
        },
        "motor_temp_start_c": t_start_motor,
        "fet_temp_start_c": t_start_fet,
        "pre_errors": err_tuple(axis),
        "samples": [],
    }

    if args.motor_id is not None:
        print(f"MOTOR #{args.motor_id}", flush=True)
    print(f"serial={report['serial_number']} fw={fw} vbus={report['vbus_v']:.1f}V", flush=True)
    print(f"start: MOTOR={t_start_motor:.1f}C  FET={t_start_fet:.1f}C  "
          f"R={m.config.phase_resistance:.4f} ohm", flush=True)
    print(f"spin {args.speed:.0f} t/s, current_lim {args.current_limit:.0f}A, "
          f"stop at MOTOR>={args.target_temp:.0f}C or {args.max_minutes:.0f} min "
          f"(firmware trips at {mt.config.temp_limit_lower:.0f}/{mt.config.temp_limit_upper:.0f}C)",
          flush=True)

    # Save the RAM config we touch; restored in finally.
    saved = (m.config.current_lim, c.config.control_mode, c.config.input_mode,
             c.config.vel_limit, axis.config.enable_watchdog)

    jsonl_f = None
    out_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)), "reports")
    json_path = args.json
    jsonl_path = args.jsonl
    if args.motor_id is not None:
        if json_path is None:
            json_path = os.path.join(out_dir, f"motor-{args.motor_id}-thermal-rise.json")
        if jsonl_path is None:
            jsonl_path = os.path.join(out_dir, f"motor-{args.motor_id}-thermal-rise.jsonl")
    if jsonl_path:
        os.makedirs(os.path.dirname(os.path.abspath(jsonl_path)), exist_ok=True)
        jsonl_f = open(jsonl_path, "w", encoding="utf-8")

    stop_reason = "unknown"
    t0 = time.monotonic()
    try:
        clear_errors(axis)
        axis.config.enable_watchdog = False
        m.config.current_lim = args.current_limit
        c.config.vel_limit = abs(args.speed) * 1.4
        c.config.control_mode = CONTROL_MODE_VELOCITY_CONTROL
        c.config.input_mode = INPUT_MODE_PASSTHROUGH
        c.input_vel = 0.0
        axis.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
        time.sleep(0.3)
        if axis.current_state != AXIS_STATE_CLOSED_LOOP_CONTROL:
            stop_reason = f"closed-loop entry failed errs={err_tuple(axis)}"
            print("FAIL:", stop_reason, flush=True)
            return
        c.input_vel = args.speed

        deadline = t0 + args.max_minutes * 60.0
        last_temp = t_start_motor
        last_t = t0
        recent = []  # rolling window to debounce the noisy thermistor
        while True:
            now = time.monotonic()
            elapsed = now - t0
            motor_t = float(mt.temperature)
            fet_t = float(ft.temperature)
            iq = float(m.current_control.Iq_measured)
            vel = float(e.vel_estimate)
            errs = err_tuple(axis)
            # Instantaneous rise rate over the last interval (C/min).
            dt = max(now - last_t, 1e-6)
            rate = (motor_t - last_temp) / dt * 60.0
            sample = {"t_s": round(elapsed, 1), "motor_c": round(motor_t, 2),
                      "fet_c": round(fet_t, 2), "iq_a": round(iq, 2),
                      "vel_turns_s": round(vel, 2), "rate_c_min": round(rate, 2),
                      "errors": errs}
            report["samples"].append(sample)
            if jsonl_f:
                jsonl_f.write(json.dumps(sample) + "\n")
                jsonl_f.flush()
            print(f"  t={elapsed:6.0f}s  MOTOR={motor_t:5.1f}C ({motor_t - t_start_motor:+4.1f})  "
                  f"FET={fet_t:5.1f}C  |Iq|={abs(iq):4.1f}A  vel={vel:5.1f}t/s  "
                  f"rate={rate:+5.2f}C/min", flush=True)
            last_temp, last_t = motor_t, now

            if any(errs):
                stop_reason = f"axis fault errs={errs}"
                break
            if axis.current_state != AXIS_STATE_CLOSED_LOOP_CONTROL:
                stop_reason = f"left closed-loop (state={axis.current_state}) — likely over-temp trip"
                break
            # Debounce: the motor thermistor glitches by several degrees on
            # single samples while spinning, so trigger only when the MEDIAN of
            # the last 3 reads crosses the target (a lone spike won't false-trip).
            recent.append(motor_t)
            recent = recent[-3:]
            if len(recent) == 3 and sorted(recent)[1] >= args.target_temp:
                stop_reason = f"reached target {args.target_temp:.0f}C (median of last 3)"
                break
            if now >= deadline:
                stop_reason = f"time cap {args.max_minutes:.0f} min"
                break
            time.sleep(args.interval)
    finally:
        c.input_vel = 0.0
        axis.requested_state = AXIS_STATE_IDLE
        time.sleep(0.3)
        (m.config.current_lim, c.config.control_mode, c.config.input_mode,
         c.config.vel_limit, axis.config.enable_watchdog) = saved
        report["motor_temp_end_c"] = float(mt.temperature)
        report["fet_temp_end_c"] = float(ft.temperature)
        report["post_errors"] = err_tuple(axis)
        report["elapsed_s"] = round(time.monotonic() - t0, 1)
        report["stop_reason"] = stop_reason
        if jsonl_f:
            jsonl_f.close()

    # Summary
    samples = report["samples"]
    dT = report["motor_temp_end_c"] - t_start_motor
    mins = report["elapsed_s"] / 60.0
    avg_rate = dT / mins if mins > 0 else 0.0
    steady_iq = (sum(abs(s["iq_a"]) for s in samples[-5:]) / max(len(samples[-5:]), 1)
                 if samples else 0.0)
    report["summary"] = {
        "delta_t_c": round(dT, 2),
        "avg_rate_c_min": round(avg_rate, 3),
        "peak_rate_c_min": round(max((s["rate_c_min"] for s in samples[1:]), default=0.0), 2),
        "steady_abs_iq_a": round(steady_iq, 2),
    }
    print("\n=== SUMMARY ===", flush=True)
    print(f"  stop reason     : {stop_reason}", flush=True)
    print(f"  MOTOR temp      : {t_start_motor:.1f}C -> {report['motor_temp_end_c']:.1f}C "
          f"(+{dT:.1f} in {mins:.1f} min)", flush=True)
    print(f"  FET temp        : {t_start_fet:.1f}C -> {report['fet_temp_end_c']:.1f}C", flush=True)
    print(f"  avg rise rate   : {avg_rate:.2f} C/min  (peak {report['summary']['peak_rate_c_min']:.2f})",
          flush=True)
    print(f"  steady |Iq|     : {steady_iq:.1f} A", flush=True)
    print("  (config changes were RAM-only; nothing saved to flash)", flush=True)

    if json_path:
        os.makedirs(os.path.dirname(os.path.abspath(json_path)), exist_ok=True)
        with open(json_path, "w", encoding="utf-8") as f:
            json.dump(report, f, indent=2)
            f.write("\n")
        print(f"  report -> {json_path}", flush=True)
        if jsonl_path:
            print(f"  samples -> {jsonl_path}", flush=True)


if __name__ == "__main__":
    main()
