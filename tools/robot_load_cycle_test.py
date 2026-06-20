#!/usr/bin/env python3
"""Cycle a geared joint up/down under load and log per-cycle current/torque.

Flow (all closed-loop on the load encoder, velocity control with a software
braking profile, so it never relies on a live mode switch):
  1. Drive the joint to load = 0.
  2. Hold 0 and wait for the start file (operator places/secures the weight).
  3. Run N cycles 0 -> lift_angle -> 0.
  4. Return to 0 and HOLD, waiting for the release file (operator removes the
     weight first). The motor is never released under load -- no dropped weight.
"""

import argparse
import json
import math
import os
import statistics
import sys
import time
from datetime import datetime, timezone

script_directory = os.path.dirname(os.path.abspath(__file__))
if sys.path and os.path.abspath(sys.path[0]) == script_directory:
    sys.path.pop(0)

import odrive
from odrive.enums import (
    AXIS_STATE_CLOSED_LOOP_CONTROL,
    AXIS_STATE_IDLE,
    CONTROL_MODE_VELOCITY_CONTROL,
    INPUT_MODE_PASSTHROUGH,
)


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--gear-ratio", type=float, default=34.0)
    parser.add_argument("--cycles", type=int, default=10)
    parser.add_argument("--lift-degrees", type=float, default=30.0,
                        help="Joint angle to lift to each cycle (default 30).")
    parser.add_argument("--lift-sign", type=int, choices=(-1, 1), default=-1,
                        help="Load-angle sign of the lift (-1 = against gravity / "
                             "'up' on this joint; default -1).")
    parser.add_argument("--cycle-vel", type=float, default=4.0,
                        help="Motor velocity for cycle moves (turns/s, default 4).")
    parser.add_argument("--position-vel", type=float, default=2.0,
                        help="Motor velocity for the initial move to 0 (turns/s).")
    parser.add_argument("--accel", type=float, default=20.0,
                        help="Braking decel target (motor turns/s^2).")
    parser.add_argument("--tolerance-turns", type=float, default=0.0015,
                        help="Endpoint tolerance on the load encoder (turns).")
    parser.add_argument("--current-limit", type=float, default=15.0)
    parser.add_argument("--fet-temperature-limit", type=float, default=70.0)
    parser.add_argument("--watchdog-timeout", type=float, default=1.0)
    parser.add_argument("--hold-kp", type=float, default=25.0,
                        help="P gain (motor turns/s per load-turn error) for the "
                             "software position hold between phases.")
    parser.add_argument("--hold-vel-limit", type=float, default=2.0,
                        help="Max correction velocity during software hold.")
    parser.add_argument("--start-file", default="/tmp/start_cycles",
                        help="Hold 0 until this file appears (weight placed).")
    parser.add_argument("--release-file", default="/tmp/release_hold",
                        help="After cycling, hold 0 until this appears, then power "
                             "down (weight removed first).")
    parser.add_argument("--signal-timeout", type=float, default=900.0,
                        help="Max seconds to wait for a start/release file.")
    parser.add_argument("--serial-number")
    parser.add_argument("--output", default="load-cycle.json")
    return parser.parse_args()


def errors(device):
    a = device.axis0
    return (int(a.error), int(a.motor.error), int(a.encoder.error),
            int(a.controller.error), int(device.axis1.error),
            int(device.axis1.encoder.error))


def check_safe(device, args):
    fault = errors(device)
    if any(fault):
        raise RuntimeError(f"ODrive fault: {fault}")
    temp = float(device.axis0.fet_thermistor.temperature)
    if not math.isfinite(temp) or temp > args.fet_temperature_limit:
        raise RuntimeError(f"FET temperature exceeded: {temp:.1f} C")
    return temp


def load_turns(device):
    return float(device.axis1.encoder.pos_estimate)


def measure_coupling(device, args):
    """Brisk velocity nudge to learn load-turns-per-motor-turn (sign + size)."""
    axis = device.axis0
    controller = axis.controller
    m0 = float(axis.encoder.pos_estimate)
    l0 = load_turns(device)
    deadline = time.monotonic() + 1.0
    while time.monotonic() < deadline:
        axis.watchdog_feed()
        check_safe(device, args)
        controller.input_vel = 0.6
        time.sleep(0.02)
    controller.input_vel = 0.0
    time.sleep(0.2)
    d_motor = float(axis.encoder.pos_estimate) - m0
    d_load = load_turns(device) - l0
    if abs(d_motor) < 0.01 or abs(d_load) < 0.2 / args.gear_ratio * abs(d_motor):
        raise RuntimeError(
            f"Load barely moved on nudge (d_motor={d_motor:.3f}, d_load={d_load:.4f})")
    coupling = d_load / d_motor
    print(f"  coupling = {coupling:+.4f} load-turn/motor-turn "
          f"(~{abs(1/coupling):.1f}:1)", flush=True)
    return coupling


def move_load_to(device, args, coupling, target_turns, max_vel, label):
    """Velocity-control move of the load encoder to target_turns with a software
    braking profile. Returns peak |Iq|, duration, and overshoot past target."""
    axis = device.axis0
    controller = axis.controller
    motor_dir = math.copysign(1.0, coupling)
    kt = float(axis.motor.config.torque_constant)
    start = load_turns(device)
    approaching_from = math.copysign(1.0, target_turns - start) if target_turns != start else 1.0
    started = time.monotonic()
    peak_iq = 0.0
    extreme = start
    stall_ref_m = float(axis.encoder.pos_estimate)
    stall_ref_l = start
    stall_at = time.monotonic() + 1.0
    while True:
        axis.watchdog_feed()
        check_safe(device, args)
        load = load_turns(device)
        extreme = max(extreme, load) if approaching_from > 0 else min(extreme, load)
        error = target_turns - load
        peak_iq = max(peak_iq, abs(float(axis.motor.current_control.Iq_measured)))
        if abs(error) <= args.tolerance_turns:
            break
        if time.monotonic() - started > 40.0:
            raise RuntimeError(f"{label}: move timeout (load={load*360:.1f} deg)")
        remaining_motor = abs(error / coupling)
        braking = math.sqrt(max(0.0, 2.0 * args.accel * remaining_motor))
        command = min(max_vel, braking)
        command = max(command, 0.25)
        controller.input_vel = math.copysign(command, error) * motor_dir
        if time.monotonic() >= stall_at:
            dm = abs(float(axis.encoder.pos_estimate) - stall_ref_m)
            dl = abs(load_turns(device) - stall_ref_l)
            if dm > 0.4 and dl < abs(coupling) * 0.1:
                raise RuntimeError(f"{label}: motor moving but load stalled")
            stall_ref_m = float(axis.encoder.pos_estimate)
            stall_ref_l = load_turns(device)
            stall_at = time.monotonic() + 1.0
        time.sleep(0.02)
    controller.input_vel = 0.0
    overshoot = max(0.0, (extreme - target_turns) * approaching_from) * 360.0
    return {
        "label": label,
        "target_deg": round(target_turns * 360, 2),
        "final_deg": round(load_turns(device) * 360, 3),
        "duration_s": round(time.monotonic() - started, 3),
        "peak_iq_a": round(peak_iq, 3),
        "peak_output_torque_nm": round(peak_iq * kt * args.gear_ratio, 2),
        "overshoot_deg": round(overshoot, 3),
    }


def hold_load_until(device, args, coupling, target_turns, signal_file, prompt):
    """Software P-hold of the load at target until signal_file appears. The motor
    keeps applying torque the whole time, so the load is never released."""
    axis = device.axis0
    controller = axis.controller
    motor_dir = math.copysign(1.0, coupling)
    kt = float(axis.motor.config.torque_constant)
    if os.path.exists(signal_file):
        os.remove(signal_file)
    print("\n" + "*" * 58, flush=True)
    for line in prompt:
        print("  " + line, flush=True)
    print(f"  signal:  touch {signal_file}", flush=True)
    print("*" * 58 + "\n", flush=True)
    started = time.monotonic()
    next_note = started
    while not os.path.exists(signal_file):
        axis.watchdog_feed()
        check_safe(device, args)
        error = target_turns - load_turns(device)
        cmd = max(-args.hold_vel_limit,
                  min(args.hold_vel_limit, args.hold_kp * error))
        controller.input_vel = cmd * motor_dir
        if time.monotonic() - started > args.signal_timeout:
            raise RuntimeError(f"Timed out waiting for {signal_file}")
        if time.monotonic() >= next_note:
            iq = float(axis.motor.current_control.Iq_measured)
            print(f"  holding {target_turns*360:+.1f} deg... "
                  f"load={load_turns(device)*360:+.2f} deg  Iq={iq:+.2f}A  "
                  f"torque={abs(iq)*kt*args.gear_ratio:.1f}Nm", flush=True)
            next_note += 10.0
        time.sleep(0.02)
    os.remove(signal_file)
    controller.input_vel = 0.0
    print(f"  signal received ({os.path.basename(signal_file)}).", flush=True)


def main():
    args = parse_args()
    if args.cycles <= 0:
        raise ValueError("--cycles must be positive")
    print("Connecting to ODrive...", flush=True)
    device = odrive.find_any(serial_number=args.serial_number, timeout=20)
    axis = device.axis0
    controller = axis.controller
    if any(errors(device)):
        raise RuntimeError(f"Pre-existing errors: {errors(device)}")
    if not axis.motor.is_calibrated or not axis.encoder.is_ready:
        raise RuntimeError("axis0 motor/encoder must be calibrated and ready")
    if hasattr(device.axis1.encoder, "mt6701_debug_crc_ok"):
        if not device.axis1.encoder.mt6701_debug_crc_ok:
            raise RuntimeError("Load-side MT6701 CRC invalid")

    lift_turns = args.lift_sign * args.lift_degrees / 360.0
    old_config = (
        axis.motor.config.current_lim,
        controller.config.control_mode,
        controller.config.input_mode,
        controller.config.vel_limit,
        controller.config.enable_current_mode_vel_limit,
        controller.config.enable_overspeed_error,
        axis.config.enable_watchdog,
        axis.config.watchdog_timeout,
    )
    result = {
        "timestamp_utc": datetime.now(timezone.utc).isoformat(),
        "test": "load-cycle",
        "gear_ratio": args.gear_ratio,
        "torque_constant": float(axis.motor.config.torque_constant),
        "cycles": args.cycles,
        "lift_degrees": args.lift_degrees,
        "lift_sign": args.lift_sign,
        "cycle_vel_turns_s": args.cycle_vel,
        "moves": [],
    }
    try:
        axis.motor.config.current_lim = args.current_limit
        controller.config.control_mode = CONTROL_MODE_VELOCITY_CONTROL
        controller.config.input_mode = INPUT_MODE_PASSTHROUGH
        controller.config.enable_current_mode_vel_limit = False
        controller.config.enable_overspeed_error = False
        controller.config.vel_limit = max(args.cycle_vel, args.position_vel) * 1.5
        controller.input_vel = 0.0
        axis.config.watchdog_timeout = args.watchdog_timeout
        axis.watchdog_feed()
        axis.config.enable_watchdog = True
        axis.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
        time.sleep(0.2)
        if axis.current_state != AXIS_STATE_CLOSED_LOOP_CONTROL:
            raise RuntimeError(
                f"Closed-loop entry failed: state={axis.current_state} "
                f"errors={errors(device)}")

        print(f"Start load = {load_turns(device)*360:.2f} deg", flush=True)
        coupling = measure_coupling(device, args)

        print("Moving to 0 deg...", flush=True)
        result["moves"].append(
            move_load_to(device, args, coupling, 0.0, args.position_vel, "to-zero"))
        print(f"  at {load_turns(device)*360:.2f} deg", flush=True)

        hold_load_until(
            device, args, coupling, 0.0, args.start_file,
            ["HOLDING 0 deg. PLACE / SECURE THE WEIGHT.",
             f"Then start {args.cycles} cycles (0 -> {lift_turns*360:+.0f} -> 0)."])

        print(f"\n=== {args.cycles} CYCLES "
              f"(0 -> {lift_turns*360:+.0f} deg -> 0) ===", flush=True)
        cycles = []
        for i in range(1, args.cycles + 1):
            up = move_load_to(device, args, coupling, lift_turns,
                              args.cycle_vel, f"cycle{i}-up")
            down = move_load_to(device, args, coupling, 0.0,
                                args.cycle_vel, f"cycle{i}-down")
            result["moves"].extend((up, down))
            cycles.append((up, down))
            print(f"  cycle {i:2d}/{args.cycles}: "
                  f"up peak={up['peak_output_torque_nm']:6.1f}Nm "
                  f"({up['duration_s']:.2f}s, over={up['overshoot_deg']:.2f}deg)  "
                  f"down peak={down['peak_output_torque_nm']:6.1f}Nm "
                  f"({down['duration_s']:.2f}s)", flush=True)

        up_peaks = [u["peak_output_torque_nm"] for u, _ in cycles]
        down_peaks = [d["peak_output_torque_nm"] for _, d in cycles]
        up_times = [u["duration_s"] for u, _ in cycles]
        result["summary"] = {
            "up_peak_torque_max_nm": max(up_peaks),
            "up_peak_torque_mean_nm": round(statistics.fmean(up_peaks), 2),
            "down_peak_torque_max_nm": max(down_peaks),
            "up_time_mean_s": round(statistics.fmean(up_times), 3),
            "max_overshoot_deg": max(m["overshoot_deg"] for m in result["moves"]),
        }
        print("\n=== SUMMARY ===", flush=True)
        print(f"up   peak torque: max={max(up_peaks):.1f}Nm "
              f"mean={statistics.fmean(up_peaks):.1f}Nm", flush=True)
        print(f"down peak torque: max={max(down_peaks):.1f}Nm", flush=True)
        print(f"max overshoot: {result['summary']['max_overshoot_deg']:.2f} deg",
              flush=True)

        hold_load_until(
            device, args, coupling, 0.0, args.release_file,
            ["CYCLES COMPLETE. STILL HOLDING 0 deg.",
             "REMOVE THE WEIGHT NOW (holding current falls to ~0 as it comes off)."])
    finally:
        try:
            controller.input_vel = 0.0
            axis.watchdog_feed()
            time.sleep(0.2)
            axis.requested_state = AXIS_STATE_IDLE
            time.sleep(0.3)
        finally:
            axis.config.enable_watchdog = False
            (
                axis.motor.config.current_lim,
                controller.config.control_mode,
                controller.config.input_mode,
                controller.config.vel_limit,
                controller.config.enable_current_mode_vel_limit,
                controller.config.enable_overspeed_error,
                axis.config.enable_watchdog,
                axis.config.watchdog_timeout,
            ) = old_config
            result["final_errors"] = errors(device)
            result["final_load_deg"] = round(load_turns(device) * 360, 2)
            with open(args.output, "w", encoding="utf-8") as f:
                json.dump(result, f, indent=2)
                f.write("\n")
            print(f"\nIDLE  load={result['final_load_deg']:.2f} deg  "
                  f"errors={result['final_errors']}  results={args.output}",
                  flush=True)


if __name__ == "__main__":
    main()
