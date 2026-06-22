#!/usr/bin/env python3
"""Alternating hold/move duty cycle under load, logging current/torque/temp.

Everything runs in velocity control on the load encoder (software braking
profile for moves, software P-hold for holds) so there is never a live mode
switch and the motor never releases under load except on a safety abort.

Flow:
  1. Arm, learn motor->load coupling, drive (unloaded) to the hold pose.
  2. P-hold the hold pose and wait for the start file (operator secures weight).
  3. Run the alternating schedule (default: hold 60s, move 60s, ... = 5 min,
     starting and ending with a hold).
  4. Return to the hold pose and HOLD, waiting for the release file (operator
     removes the weight first).
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
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--gear-ratio", type=float, default=34.0)
    p.add_argument("--hold-deg", type=float, default=90.0,
                   help="Absolute load-encoder angle to hold (max-torque pose).")
    p.add_argument("--move-to-deg", type=float, default=50.0,
                   help="Far end of the move sweep; leg sweeps hold<->move-to "
                        "(may be above or below the hold pose).")
    p.add_argument("--phase-seconds", type=float, default=60.0)
    p.add_argument("--total-seconds", type=float, default=300.0)
    p.add_argument("--cycle-vel", type=float, default=4.0,
                   help="Motor turns/s cruise during the move phase.")
    p.add_argument("--position-vel", type=float, default=2.0)
    p.add_argument("--accel", type=float, default=20.0)
    p.add_argument("--tolerance-turns", type=float, default=0.0015)
    p.add_argument("--hold-kp", type=float, default=25.0)
    p.add_argument("--hold-vel-limit", type=float, default=2.0)
    p.add_argument("--current-limit", type=float, default=25.0)
    p.add_argument("--fet-temperature-limit", type=float, default=80.0)
    p.add_argument("--endstop-lo-deg", type=float, default=3.0)
    p.add_argument("--endstop-hi-deg", type=float, default=108.0)
    p.add_argument("--watchdog-timeout", type=float, default=1.0)
    p.add_argument("--sample-interval", type=float, default=1.0)
    p.add_argument("--start-file", default="/tmp/start_cycles")
    p.add_argument("--release-file", default="/tmp/release_hold")
    p.add_argument("--signal-timeout", type=float, default=900.0)
    p.add_argument("--serial-number")
    p.add_argument("--output", default="leg-5kg-duty-5min.json")
    return p.parse_args()


def errors(device):
    a = device.axis0
    return (int(a.error), int(a.motor.error), int(a.encoder.error),
            int(a.controller.error), int(device.axis1.error),
            int(device.axis1.encoder.error))


class Guard:
    """Per-iteration safety: faults, FET temp, encoder freshness, endstops."""
    def __init__(self, device, args):
        self.device = device
        self.args = args
        self.last_sc = int(device.axis1.encoder.mt6701_debug_sample_count)
        self.last_sc_t = time.monotonic()

    def check(self):
        d, args = self.device, self.args
        fault = errors(d)
        if any(fault):
            raise RuntimeError(f"ODrive fault: {fault}")
        t = float(d.axis0.fet_thermistor.temperature)
        if not math.isfinite(t) or t > args.fet_temperature_limit:
            raise RuntimeError(f"FET temperature exceeded: {t:.1f} C")
        sc = int(d.axis1.encoder.mt6701_debug_sample_count)
        now = time.monotonic()
        if sc != self.last_sc:
            self.last_sc, self.last_sc_t = sc, now
        elif now - self.last_sc_t > 0.15:
            raise RuntimeError("axis1 load encoder stale (sample_count frozen)")
        pos = float(d.axis1.encoder.pos_estimate) * 360.0
        if pos < args.endstop_lo_deg or pos > args.endstop_hi_deg:
            raise RuntimeError(f"endstop guard: load={pos:.2f} deg")
        return t


def load_turns(device):
    return float(device.axis1.encoder.pos_estimate)


def measure_coupling(device, args, guard):
    axis = device.axis0
    m0 = float(axis.encoder.pos_estimate)
    l0 = load_turns(device)
    deadline = time.monotonic() + 1.0
    while time.monotonic() < deadline:
        axis.watchdog_feed()
        guard.check()
        axis.controller.input_vel = 0.6
        time.sleep(0.02)
    axis.controller.input_vel = 0.0
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


def sample(device, args, result, t0, phase, peak):
    a = device.axis0
    kt = float(a.motor.config.torque_constant)
    iq = float(a.motor.current_control.Iq_measured)
    temp = float(a.fet_thermistor.temperature)
    load_deg = load_turns(device) * 360.0
    elapsed = time.monotonic() - t0
    peak["iq"] = max(peak["iq"], abs(iq))
    peak["temp"] = max(peak["temp"], temp)
    result["samples"].append({
        "t_s": round(elapsed, 2),
        "phase": phase,
        "iq_a": round(iq, 3),
        "output_torque_nm": round(abs(iq) * kt * args.gear_ratio, 2),
        "load_deg": round(load_deg, 3),
        "fet_c": round(temp, 1),
    })
    print(f"t={elapsed:6.1f}s [{phase:4s}] Iq={iq:6.2f}A "
          f"torque={abs(iq)*kt*args.gear_ratio:6.2f}Nm "
          f"load={load_deg:6.2f}deg fet={temp:4.1f}C", flush=True)


def run_hold(device, args, guard, target_turns, motor_dir, duration,
             result, t0, peak):
    axis = device.axis0
    started = time.monotonic()
    next_s = time.monotonic()
    lo_deg, hi_deg = 1e9, -1e9
    while time.monotonic() - started < duration:
        axis.watchdog_feed()
        guard.check()
        error = target_turns - load_turns(device)
        cmd = max(-args.hold_vel_limit, min(args.hold_vel_limit, args.hold_kp * error))
        axis.controller.input_vel = cmd * motor_dir
        d = load_turns(device) * 360.0
        lo_deg, hi_deg = min(lo_deg, d), max(hi_deg, d)
        if time.monotonic() >= next_s:
            sample(device, args, result, t0, "hold", peak)
            next_s += args.sample_interval
        time.sleep(0.02)
    return {"max_hold_dev_deg": round(max(abs(hi_deg - target_turns * 360),
                                          abs(lo_deg - target_turns * 360)), 3)}


def run_move(device, args, guard, hi_turns, lo_turns, motor_dir, duration,
             result, t0, peak):
    axis = device.axis0
    started = time.monotonic()
    next_s = time.monotonic()
    target = lo_turns          # move down first
    sweeps = 0
    while time.monotonic() - started < duration:
        axis.watchdog_feed()
        guard.check()
        load = load_turns(device)
        error = target - load
        if abs(error) <= args.tolerance_turns:
            target = hi_turns if target == lo_turns else lo_turns
            sweeps += 1
            continue
        remaining_motor = abs(error / guard_coupling[0])
        braking = math.sqrt(max(0.0, 2.0 * args.accel * remaining_motor))
        command = max(0.25, min(args.cycle_vel, braking))
        axis.controller.input_vel = math.copysign(command, error) * motor_dir
        if time.monotonic() >= next_s:
            sample(device, args, result, t0, "move", peak)
            next_s += args.sample_interval
        time.sleep(0.02)
    axis.controller.input_vel = 0.0
    return {"sweeps": sweeps}


def hold_until_signal(device, args, guard, target_turns, motor_dir,
                      signal_file, prompt):
    axis = device.axis0
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
        guard.check()
        error = target_turns - load_turns(device)
        cmd = max(-args.hold_vel_limit, min(args.hold_vel_limit, args.hold_kp * error))
        axis.controller.input_vel = cmd * motor_dir
        if time.monotonic() - started > args.signal_timeout:
            raise RuntimeError(f"Timed out waiting for {signal_file}")
        if time.monotonic() >= next_note:
            iq = float(axis.motor.current_control.Iq_measured)
            print(f"  holding {target_turns*360:+.1f}deg... "
                  f"load={load_turns(device)*360:+.2f}deg Iq={iq:+.2f}A "
                  f"torque={abs(iq)*kt*args.gear_ratio:.1f}Nm", flush=True)
            next_note += 10.0
        time.sleep(0.02)
    os.remove(signal_file)
    axis.controller.input_vel = 0.0


# tiny holder so run_move can see coupling without threading it everywhere
guard_coupling = [1.0]


def build_schedule(args):
    schedule = []
    remaining = args.total_seconds
    phase = "hold"          # start with a hold
    while remaining > 1e-6:
        dur = min(args.phase_seconds, remaining)
        schedule.append((phase, dur))
        remaining -= dur
        phase = "move" if phase == "hold" else "hold"
    return schedule


def main():
    args = parse_args()
    if abs(args.move_to_deg - args.hold_deg) < 1.0:
        raise ValueError("--move-to-deg must differ from --hold-deg")
    for v in (args.hold_deg, args.move_to_deg):
        if v < args.endstop_lo_deg or v > args.endstop_hi_deg:
            raise ValueError("hold/move angles must be inside the endstop band")

    print("Connecting to ODrive...", flush=True)
    device = odrive.find_any(serial_number=args.serial_number, timeout=20)
    axis = device.axis0
    controller = axis.controller
    if any(errors(device)):
        raise RuntimeError(f"Pre-existing errors: {errors(device)}")
    if not axis.motor.is_calibrated or not axis.encoder.is_ready:
        raise RuntimeError("axis0 motor/encoder must be calibrated and ready")
    if not device.axis1.encoder.mt6701_debug_crc_ok:
        raise RuntimeError("Load-side MT6701 CRC invalid")

    hold_turns = args.hold_deg / 360.0
    move_to_turns = args.move_to_deg / 360.0
    schedule = build_schedule(args)
    guard = Guard(device, args)

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
        "test": "load-duty-hold-move",
        "gear_ratio": args.gear_ratio,
        "torque_constant": float(axis.motor.config.torque_constant),
        "hold_deg": args.hold_deg,
        "move_to_deg": args.move_to_deg,
        "schedule": [{"phase": p, "seconds": round(s, 1)} for p, s in schedule],
        "samples": [],
        "phases": [],
    }
    peak = {"iq": 0.0, "temp": 0.0}

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
            raise RuntimeError(f"Closed-loop entry failed: state={axis.current_state} "
                               f"errors={errors(device)}")

        print(f"Start load = {load_turns(device)*360:.2f} deg", flush=True)
        coupling = measure_coupling(device, args, guard)
        guard_coupling[0] = coupling
        motor_dir = math.copysign(1.0, coupling)

        # Drive (unloaded) to the hold pose using a braking profile.
        print(f"Moving to hold pose {args.hold_deg:.1f} deg...", flush=True)
        started = time.monotonic()
        while abs(hold_turns - load_turns(device)) > args.tolerance_turns:
            axis.watchdog_feed()
            guard.check()
            error = hold_turns - load_turns(device)
            remaining_motor = abs(error / coupling)
            braking = math.sqrt(max(0.0, 2.0 * args.accel * remaining_motor))
            command = max(0.25, min(args.position_vel, braking))
            controller.input_vel = math.copysign(command, error) * motor_dir
            if time.monotonic() - started > 60.0:
                raise RuntimeError("Timeout reaching hold pose")
            time.sleep(0.02)
        controller.input_vel = 0.0
        print(f"  at {load_turns(device)*360:.2f} deg", flush=True)

        hold_until_signal(
            device, args, guard, hold_turns, motor_dir, args.start_file,
            [f"HOLDING {args.hold_deg:.0f} deg. SECURE THE 5 kg WEIGHT firmly",
             "(it will swing through the move phase -- make sure it cannot fall off).",
             f"Then start the {args.total_seconds:.0f}s hold/move duty cycle."])

        print(f"\n=== DUTY CYCLE: {len(schedule)} phases "
              f"({args.phase_seconds:.0f}s each) ===", flush=True)
        t0 = time.monotonic()
        for i, (phase, dur) in enumerate(schedule, 1):
            print(f"\n--- phase {i}/{len(schedule)}: {phase.upper()} {dur:.0f}s ---",
                  flush=True)
            if phase == "hold":
                info = run_hold(device, args, guard, hold_turns, motor_dir, dur,
                                result, t0, peak)
            else:
                info = run_move(device, args, guard, hold_turns, move_to_turns,
                                motor_dir, dur, result, t0, peak)
            info.update({"phase": phase, "index": i})
            result["phases"].append(info)

        iqs = [s["iq_a"] for s in result["samples"]]
        result["summary"] = {
            "peak_iq_a": round(peak["iq"], 3),
            "mean_abs_iq_a": round(statistics.fmean(abs(x) for x in iqs), 3) if iqs else None,
            "peak_fet_c": round(peak["temp"], 1),
            "peak_output_torque_nm": round(
                peak["iq"] * float(axis.motor.config.torque_constant) * args.gear_ratio, 2),
        }
        print("\n=== SUMMARY ===", flush=True)
        print(f"peak Iq           = {peak['iq']:.2f} A", flush=True)
        print(f"peak output torque= {result['summary']['peak_output_torque_nm']:.2f} Nm",
              flush=True)
        print(f"peak FET temp     = {peak['temp']:.1f} C", flush=True)

        hold_until_signal(
            device, args, guard, hold_turns, motor_dir, args.release_file,
            ["DUTY CYCLE COMPLETE. STILL HOLDING -- REMOVE THE WEIGHT NOW",
             "(holding current falls to ~0 as it comes off)."])
    finally:
        try:
            controller.input_vel = 0.0
            axis.watchdog_feed()
            time.sleep(0.2)
            axis.requested_state = AXIS_STATE_IDLE
            time.sleep(0.3)
        finally:
            axis.config.enable_watchdog = False
            (axis.motor.config.current_lim,
             controller.config.control_mode,
             controller.config.input_mode,
             controller.config.vel_limit,
             controller.config.enable_current_mode_vel_limit,
             controller.config.enable_overspeed_error,
             axis.config.enable_watchdog,
             axis.config.watchdog_timeout) = old_config
            result["final_errors"] = errors(device)
            result["final_load_deg"] = round(load_turns(device) * 360.0, 2)
            with open(args.output, "w", encoding="utf-8") as f:
                json.dump(result, f, indent=2)
                f.write("\n")
            print(f"\nIDLE load={result['final_load_deg']:.2f}deg "
                  f"errors={result['final_errors']} results={args.output}", flush=True)


if __name__ == "__main__":
    main()
