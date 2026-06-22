#!/usr/bin/env python3
"""Endurance duty-cycle: alternate sweeping and holding a geared joint under load.

One cycle = sweep_seconds of continuous load 0 <-> lift_degrees sweeps, then
hold_seconds of static hold at 0 deg. Repeats until --duration elapses. Built for
unattended hour-long runs: every telemetry sample is appended to a JSONL log and
flushed+fsync'd to disk, so a USB/power drop loses at most the last sample. The
34:1 gearbox is non-backdrivable, so a watchdog-triggered IDLE on disconnect
holds the joint mechanically rather than dropping the load.

Flow (closed-loop velocity control on axis0, load encoder = axis1):
  1. Drive load to 0 deg (no weight yet).
  2. Hold 0 and wait for --start-file (operator places/secures the weight).
  3. Run the sweep/hold duty cycle for --duration seconds.
  4. Return to 0 and HOLD, waiting for --release-file (operator removes weight).
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
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--duration", type=float, default=3600.0,
                        help="Total duty-cycle run time in seconds (default 3600).")
    parser.add_argument("--sweep-seconds", type=float, default=60.0,
                        help="Seconds of continuous sweeping per cycle (default 60).")
    parser.add_argument("--hold-seconds", type=float, default=60.0,
                        help="Seconds of static hold at 0 deg per cycle (default 60).")
    parser.add_argument("--gear-ratio", type=float, default=34.0)
    parser.add_argument("--lift-degrees", type=float, default=45.0,
                        help="Joint angle swept to each stroke (default 45).")
    parser.add_argument("--lift-sign", type=int, choices=(-1, 1), default=1,
                        help="Load-angle sign of the sweep target (+1 = leg-down on "
                             "this joint; default +1).")
    parser.add_argument("--sweep-vel", type=float, default=3.0,
                        help="Motor velocity for sweep moves (turns/s, default 3).")
    parser.add_argument("--position-vel", type=float, default=2.0,
                        help="Motor velocity for the initial move to 0 (turns/s).")
    parser.add_argument("--accel", type=float, default=18.0,
                        help="Braking decel target (motor turns/s^2).")
    parser.add_argument("--tolerance-turns", type=float, default=0.0015,
                        help="Endpoint tolerance on the load encoder (turns).")
    parser.add_argument("--hold-kp", type=float, default=25.0,
                        help="P gain (motor turns/s per load-turn error) for the "
                             "software position hold.")
    parser.add_argument("--hold-vel-limit", type=float, default=2.0,
                        help="Max correction velocity during software hold.")
    parser.add_argument("--current-limit", type=float, default=15.0)
    parser.add_argument("--fet-temperature-limit", type=float, default=70.0)
    parser.add_argument("--load-deg-min", type=float, default=-10.0,
                        help="Abort if load angle falls below this (runaway guard).")
    parser.add_argument("--load-deg-max", type=float, default=None,
                        help="Abort if load angle exceeds this (default lift+15).")
    parser.add_argument("--watchdog-timeout", type=float, default=1.0)
    parser.add_argument("--sample-interval", type=float, default=1.0,
                        help="Telemetry sample / fsync period in seconds.")
    parser.add_argument("--start-file", default="/tmp/start_cycles",
                        help="Hold 0 until this file appears (weight placed).")
    parser.add_argument("--release-file", default="/tmp/release_hold",
                        help="After the run, hold 0 until this appears, then power "
                             "down (weight removed first).")
    parser.add_argument("--signal-timeout", type=float, default=1800.0,
                        help="Max seconds to wait for a start/release file.")
    parser.add_argument("--serial-number")
    parser.add_argument("--output", default="endurance.json",
                        help="Final summary JSON. The per-sample log is written "
                             "alongside it as <output without .json>.jsonl.")
    parser.add_argument("--log",
                        help="Explicit path for the incremental JSONL log "
                             "(default: derived from --output).")
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


class Telemetry:
    """Append-only JSONL sink, flushed+fsync'd every write so the log survives an
    abrupt disconnect."""

    def __init__(self, path):
        self.path = path
        self._fh = open(path, "w", encoding="utf-8")

    def write(self, record):
        self._fh.write(json.dumps(record, separators=(",", ":")))
        self._fh.write("\n")
        self._fh.flush()
        os.fsync(self._fh.fileno())

    def close(self):
        try:
            self._fh.flush()
            os.fsync(self._fh.fileno())
        finally:
            self._fh.close()


def move_load_blocking(device, args, coupling, target_turns, max_vel, label,
                       telemetry, clock, sample_state):
    """Velocity-control move of the load encoder to target_turns with a software
    braking profile, sampling telemetry on the shared cadence. Returns peak |Iq|."""
    axis = device.axis0
    controller = axis.controller
    motor_dir = math.copysign(1.0, coupling)
    kt = float(axis.motor.config.torque_constant)
    started = time.monotonic()
    peak_iq = 0.0
    stall_ref_m = float(axis.encoder.pos_estimate)
    stall_ref_l = load_turns(device)
    stall_at = time.monotonic() + 1.0
    while True:
        axis.watchdog_feed()
        temp = check_safe(device, args)
        load = load_turns(device)
        guard_load(args, load)
        error = target_turns - load
        iq = float(axis.motor.current_control.Iq_measured)
        peak_iq = max(peak_iq, abs(iq))
        maybe_sample(telemetry, clock, sample_state, device, args, kt, iq, temp,
                     load, label)
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
    return peak_iq


def guard_load(args, load):
    deg = load * 360.0
    if deg < args.load_deg_min or deg > args._load_deg_max:
        raise RuntimeError(
            f"Load angle {deg:.1f} deg outside guard "
            f"[{args.load_deg_min}, {args._load_deg_max}] -- aborting")


def maybe_sample(telemetry, clock, state, device, args, kt, iq, temp, load, phase):
    now = time.monotonic()
    if now < state["next"]:
        return
    state["next"] += args.sample_interval
    out_torque = abs(iq) * kt * args.gear_ratio
    state["peak_iq"] = max(state["peak_iq"], abs(iq))
    state["peak_temp"] = max(state["peak_temp"], temp)
    record = {
        "t_s": round(now - clock["start"], 2),
        "cycle": state["cycle"],
        "phase": phase,
        "iq_a": round(iq, 3),
        "out_torque_nm": round(out_torque, 2),
        "load_deg": round(load * 360.0, 3),
        "motor_turns": round(float(device.axis0.encoder.pos_estimate), 4),
        "fet_c": round(temp, 1),
    }
    telemetry.write(record)


def hold_at(device, args, coupling, target_turns, seconds, telemetry, clock,
            state, phase_label):
    """Software P-hold of the load at target for `seconds`, sampling telemetry."""
    axis = device.axis0
    controller = axis.controller
    motor_dir = math.copysign(1.0, coupling)
    kt = float(axis.motor.config.torque_constant)
    end = time.monotonic() + seconds
    while time.monotonic() < end:
        axis.watchdog_feed()
        temp = check_safe(device, args)
        load = load_turns(device)
        guard_load(args, load)
        error = target_turns - load
        cmd = max(-args.hold_vel_limit, min(args.hold_vel_limit, args.hold_kp * error))
        controller.input_vel = cmd * motor_dir
        iq = float(axis.motor.current_control.Iq_measured)
        maybe_sample(telemetry, clock, state, device, args, kt, iq, temp, load,
                     phase_label)
        time.sleep(0.02)
    controller.input_vel = 0.0


def wait_for_signal(device, args, coupling, target_turns, signal_file, prompt,
                    telemetry, clock, state):
    """P-hold at target until signal_file appears (motor never released)."""
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
        temp = check_safe(device, args)
        load = load_turns(device)
        guard_load(args, load)
        error = target_turns - load
        cmd = max(-args.hold_vel_limit, min(args.hold_vel_limit, args.hold_kp * error))
        controller.input_vel = cmd * motor_dir
        iq = float(axis.motor.current_control.Iq_measured)
        maybe_sample(telemetry, clock, state, device, args, kt, iq, temp, load,
                     "wait-signal")
        if time.monotonic() - started > args.signal_timeout:
            raise RuntimeError(f"Timed out waiting for {signal_file}")
        if time.monotonic() >= next_note:
            print(f"  holding {target_turns*360:+.1f} deg... "
                  f"load={load*360:+.2f} deg  Iq={iq:+.2f}A  "
                  f"torque={abs(iq)*kt*args.gear_ratio:.1f}Nm", flush=True)
            next_note += 10.0
        time.sleep(0.02)
    os.remove(signal_file)
    controller.input_vel = 0.0
    print(f"  signal received ({os.path.basename(signal_file)}).", flush=True)


def main():
    args = parse_args()
    if args.duration <= 0:
        raise ValueError("--duration must be positive")
    args._load_deg_max = (args.load_deg_max if args.load_deg_max is not None
                          else args.lift_degrees + 15.0)
    log_path = args.log or (os.path.splitext(args.output)[0] + ".jsonl")

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
        "test": "endurance-duty-cycle",
        "gear_ratio": args.gear_ratio,
        "torque_constant": float(axis.motor.config.torque_constant),
        "duration_s": args.duration,
        "sweep_seconds": args.sweep_seconds,
        "hold_seconds": args.hold_seconds,
        "lift_degrees": args.lift_degrees,
        "lift_sign": args.lift_sign,
        "current_limit": args.current_limit,
        "log_file": os.path.abspath(log_path),
        "cycles": [],
    }
    telemetry = Telemetry(log_path)
    clock = {"start": time.monotonic()}
    state = {"next": clock["start"], "cycle": 0, "peak_iq": 0.0, "peak_temp": 0.0}
    print(f"Logging telemetry -> {os.path.abspath(log_path)}", flush=True)

    try:
        axis.motor.config.current_lim = args.current_limit
        controller.config.control_mode = CONTROL_MODE_VELOCITY_CONTROL
        controller.config.input_mode = INPUT_MODE_PASSTHROUGH
        controller.config.enable_current_mode_vel_limit = False
        controller.config.enable_overspeed_error = False
        controller.config.vel_limit = max(args.sweep_vel, args.position_vel) * 1.5
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
        move_load_blocking(device, args, coupling, 0.0, args.position_vel,
                           "to-zero", telemetry, clock, state)
        print(f"  at {load_turns(device)*360:.2f} deg", flush=True)

        wait_for_signal(
            device, args, coupling, 0.0, args.start_file,
            ["HOLDING 0 deg. PLACE / SECURE THE WEIGHT.",
             f"Then run {args.duration/60:.0f} min of "
             f"[{args.sweep_seconds:.0f}s sweep 0<->{lift_turns*360:+.0f} deg, "
             f"{args.hold_seconds:.0f}s hold 0 deg]."],
            telemetry, clock, state)

        run_start = time.monotonic()
        print(f"\n=== DUTY CYCLE: {args.duration/60:.0f} min "
              f"(sweep 0<->{lift_turns*360:+.0f} deg) ===", flush=True)
        cycle_index = 0
        while time.monotonic() - run_start < args.duration:
            cycle_index += 1
            state["cycle"] = cycle_index

            # --- sweep phase: continuous 0 <-> lift strokes for sweep_seconds ---
            sweep_end = time.monotonic() + args.sweep_seconds
            target = lift_turns
            strokes = 0
            sweep_peak_iq = 0.0
            while time.monotonic() < sweep_end and \
                    time.monotonic() - run_start < args.duration:
                p = move_load_blocking(device, args, coupling, target,
                                       args.sweep_vel,
                                       f"c{cycle_index}-sweep", telemetry, clock,
                                       state)
                sweep_peak_iq = max(sweep_peak_iq, p)
                strokes += 1
                target = 0.0 if target == lift_turns else lift_turns
            # leave the joint at 0 before the hold phase
            if abs(load_turns(device)) > args.tolerance_turns * 4:
                move_load_blocking(device, args, coupling, 0.0, args.sweep_vel,
                                   f"c{cycle_index}-settle", telemetry, clock, state)

            # --- hold phase: static at 0 deg for hold_seconds ---
            hold_remaining = min(
                args.hold_seconds,
                max(0.0, args.duration - (time.monotonic() - run_start)))
            hold_loads = []
            if hold_remaining > 0:
                hold_start_load = load_turns(device) * 360.0
                hold_at(device, args, coupling, 0.0, hold_remaining, telemetry,
                        clock, state, f"c{cycle_index}-hold")
                hold_loads = [hold_start_load, load_turns(device) * 360.0]

            kt = float(axis.motor.config.torque_constant)
            cyc = {
                "cycle": cycle_index,
                "strokes": strokes,
                "sweep_peak_torque_nm": round(sweep_peak_iq * kt * args.gear_ratio, 2),
                "hold_load_deg": [round(x, 3) for x in hold_loads],
                "fet_c": round(float(axis.fet_thermistor.temperature), 1),
                "elapsed_s": round(time.monotonic() - run_start, 1),
            }
            result["cycles"].append(cyc)
            print(f"  cycle {cycle_index:2d}: {strokes:2d} strokes, "
                  f"sweep peak {cyc['sweep_peak_torque_nm']:5.1f}Nm, "
                  f"hold droop {hold_loads[-1] if hold_loads else 0:+.2f}deg, "
                  f"fet {cyc['fet_c']:.1f}C, t={cyc['elapsed_s']:.0f}s", flush=True)

        peaks = [c["sweep_peak_torque_nm"] for c in result["cycles"]]
        result["summary"] = {
            "completed_cycles": len(result["cycles"]),
            "total_strokes": sum(c["strokes"] for c in result["cycles"]),
            "sweep_peak_torque_max_nm": max(peaks) if peaks else None,
            "sweep_peak_torque_mean_nm": round(statistics.fmean(peaks), 2) if peaks else None,
            "peak_iq_a": round(state["peak_iq"], 3),
            "peak_fet_c": round(state["peak_temp"], 1),
            "run_seconds": round(time.monotonic() - run_start, 1),
        }
        print("\n=== SUMMARY ===", flush=True)
        print(f"cycles completed : {result['summary']['completed_cycles']}", flush=True)
        print(f"total strokes    : {result['summary']['total_strokes']}", flush=True)
        print(f"sweep peak torque: max={result['summary']['sweep_peak_torque_max_nm']}Nm "
              f"mean={result['summary']['sweep_peak_torque_mean_nm']}Nm", flush=True)
        print(f"peak FET temp    : {result['summary']['peak_fet_c']} C", flush=True)

        wait_for_signal(
            device, args, coupling, 0.0, args.release_file,
            ["RUN COMPLETE. STILL HOLDING 0 deg.",
             "REMOVE THE WEIGHT NOW (holding current falls to ~0 as it comes off)."],
            telemetry, clock, state)
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
            telemetry.close()
            print(f"\nIDLE  load={result['final_load_deg']:.2f} deg  "
                  f"errors={result['final_errors']}\n"
                  f"summary={args.output}\nlog={log_path}", flush=True)


if __name__ == "__main__":
    main()
