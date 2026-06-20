#!/usr/bin/env python3
"""Hold the current pose at 0 deg under load and log drift / current / temperature.

Workflow: the script enters closed-loop position control holding the present
pose, prints a clear "PUT THE WEIGHT" prompt, then samples for --duration
seconds. The moment the load is applied is visible as a jump in Iq / drift.
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
    CONTROL_MODE_POSITION_CONTROL,
    CONTROL_MODE_VELOCITY_CONTROL,
    INPUT_MODE_PASSTHROUGH,
)


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--duration", type=float, default=300.0,
                        help="Hold duration in seconds (default 300 = 5 min).")
    parser.add_argument("--gear-ratio", type=float, default=34.0)
    parser.add_argument("--load-feedback", action="store_true",
                        help="Also log axis1 (load-side MT6701) position.")
    parser.add_argument("--goto-load-turns", type=float, default=None,
                        help="Before holding, drive axis1 (load encoder) to this "
                             "value in turns using closed-loop position control. "
                             "Requires --load-feedback.")
    parser.add_argument("--goto-tolerance-turns", type=float, default=0.0015,
                        help="Stop positioning when |axis1 - target| is below this "
                             "(default 0.0015 turns ~= 0.5 deg).")
    parser.add_argument("--goto-vel-limit", type=float, default=1.0,
                        help="Motor velocity limit during positioning (turns/s).")
    parser.add_argument("--goto-step-turns", type=float, default=1.0,
                        help="Max motor-turn step per positioning iteration.")
    parser.add_argument("--sample-interval", type=float, default=1.0)
    parser.add_argument("--current-limit", type=float, default=15.0)
    parser.add_argument("--fet-temperature-limit", type=float, default=70.0)
    parser.add_argument("--max-drift-degrees", type=float, default=10.0,
                        help="Abort if output drifts more than this (default 10).")
    parser.add_argument("--watchdog-timeout", type=float, default=1.0)
    parser.add_argument(
        "--wait-for-removal", action="store_true",
        help="At the end of the hold, KEEP holding (do not release the motor) "
             "and wait until the release file appears, so the weight can be "
             "removed first. Prevents dropping the load on auto-stop.")
    parser.add_argument(
        "--release-file", default="/tmp/release_hold",
        help="Path polled by --wait-for-removal; create it to release the motor "
             "(e.g. `touch /tmp/release_hold`).")
    parser.add_argument(
        "--release-wait-timeout", type=float, default=900.0,
        help="Max seconds to keep holding while waiting for release (default 900).")
    parser.add_argument("--serial-number")
    parser.add_argument("--output",
                        default="hold-test.json")
    return parser.parse_args()


def errors(device, load):
    axis = device.axis0
    values = [int(axis.error), int(axis.motor.error),
              int(axis.encoder.error), int(axis.controller.error)]
    if load:
        values.extend((int(device.axis1.error), int(device.axis1.encoder.error)))
    return tuple(values)


def check_safe(device, args):
    fault = errors(device, args.load_feedback)
    if any(fault):
        raise RuntimeError(f"ODrive fault: {fault}")
    temperature = float(device.axis0.fet_thermistor.temperature)
    if not math.isfinite(temperature) or temperature > args.fet_temperature_limit:
        raise RuntimeError(f"FET temperature exceeded: {temperature:.1f} C")
    return temperature


def drive_to_load_target(device, args, target_turns):
    """Drive axis1 (load encoder) to target_turns using VELOCITY control on the
    motor with a software braking profile -- mirrors the characterization tool to
    avoid overspeed faults from large position steps. Returns the signed coupling
    (load turns per motor turn)."""
    axis = device.axis0
    controller = axis.controller

    def load_turns():
        return float(device.axis1.encoder.pos_estimate)

    controller.config.control_mode = CONTROL_MODE_VELOCITY_CONTROL
    controller.config.input_mode = INPUT_MODE_PASSTHROUGH
    controller.input_vel = 0.0
    old_overspeed = controller.config.enable_overspeed_error
    controller.config.enable_overspeed_error = False
    try:
        error = target_turns - load_turns()
        print(f"Positioning: load={load_turns():.4f} -> target={target_turns:.4f} "
              f"turns (error {error*360:.2f} deg)", flush=True)

        # Measure motor->load coupling with a velocity nudge. Needs to be brisk
        # enough to break gearbox stiction (~0.5 turns/s threshold on this joint).
        nudge_vel = 0.6
        motor_before = float(axis.encoder.pos_estimate)
        load_before = load_turns()
        deadline = time.monotonic() + 1.0
        while time.monotonic() < deadline:
            axis.watchdog_feed()
            check_safe(device, args)
            controller.input_vel = nudge_vel
            time.sleep(0.02)
        controller.input_vel = 0.0
        time.sleep(0.2)
        d_motor = float(axis.encoder.pos_estimate) - motor_before
        d_load = load_turns() - load_before
        if abs(d_motor) < 0.01 or abs(d_load) < 0.2 / args.gear_ratio * abs(d_motor):
            raise RuntimeError(
                f"Load barely moved on nudge (d_motor={d_motor:.3f}, "
                f"d_load={d_load:.4f}); joint may be stuck/at a stop -- aborting")
        coupling = d_load / d_motor
        # motor velocity sign that drives load toward +target:
        motor_dir = math.copysign(1.0, coupling)
        print(f"  coupling = {coupling:+.4f} load-turn/motor-turn "
              f"(~{abs(1/coupling):.1f}:1), motor_dir={motor_dir:+.0f}", flush=True)

        started = time.monotonic()
        stall_ref_motor = float(axis.encoder.pos_estimate)
        stall_ref_load = load_turns()
        stall_check = time.monotonic() + 1.0
        while True:
            axis.watchdog_feed()
            check_safe(device, args)
            error = target_turns - load_turns()
            if abs(error) <= args.goto_tolerance_turns:
                break
            if time.monotonic() - started > 90.0:
                raise RuntimeError("Positioning timeout (90 s)")
            remaining_motor = abs(error / coupling)
            braking = math.sqrt(max(0.0, 2.0 * 20.0 * remaining_motor))
            command = min(args.goto_vel_limit, braking)
            command = max(command, 0.1)
            controller.input_vel = math.copysign(command, error) * motor_dir
            if time.monotonic() >= stall_check:
                moved_motor = abs(float(axis.encoder.pos_estimate) - stall_ref_motor)
                moved_load = abs(load_turns() - stall_ref_load)
                if moved_motor > 0.3 and moved_load < abs(coupling) * 0.1:
                    raise RuntimeError(
                        "Motor moving but load stalled -- possible hard stop")
                stall_ref_motor = float(axis.encoder.pos_estimate)
                stall_ref_load = load_turns()
                stall_check = time.monotonic() + 1.0
            time.sleep(0.02)

        controller.input_vel = 0.0
        deadline = time.monotonic() + 1.0
        while time.monotonic() < deadline:
            axis.watchdog_feed()
            check_safe(device, args)
            time.sleep(0.02)
        print(f"  reached load={load_turns():.4f} turns "
              f"({load_turns()*360:.2f} deg)", flush=True)
        return coupling
    finally:
        controller.input_vel = 0.0
        controller.config.enable_overspeed_error = old_overspeed
        controller.config.control_mode = CONTROL_MODE_POSITION_CONTROL
        controller.config.input_mode = INPUT_MODE_PASSTHROUGH
        controller.input_pos = float(axis.encoder.pos_estimate)


def main():
    args = parse_args()
    if args.goto_load_turns is not None and not args.load_feedback:
        raise ValueError("--goto-load-turns requires --load-feedback")
    print("Connecting to ODrive...", flush=True)
    device = odrive.find_any(serial_number=args.serial_number, timeout=20)
    axis = device.axis0
    controller = axis.controller

    if any(errors(device, args.load_feedback)):
        raise RuntimeError(f"Pre-existing errors: {errors(device, args.load_feedback)}")
    if not axis.motor.is_calibrated or not axis.encoder.is_ready:
        raise RuntimeError("axis0 motor/encoder must be calibrated and ready")

    zero = {
        "motor": float(axis.encoder.pos_estimate),
        "load": float(device.axis1.encoder.pos_estimate) if args.load_feedback else None,
    }

    def motor_drift_deg():
        return (float(axis.encoder.pos_estimate) - zero["motor"]) / args.gear_ratio * 360.0

    def load_drift_deg():
        return (float(device.axis1.encoder.pos_estimate) - zero["load"]) * 360.0

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
        "test": "hold-0-under-load",
        "duration_s": args.duration,
        "gear_ratio": args.gear_ratio,
        "torque_constant": float(axis.motor.config.torque_constant),
        "load_feedback": args.load_feedback,
        "samples": [],
    }

    try:
        axis.motor.config.current_lim = args.current_limit
        controller.config.control_mode = CONTROL_MODE_POSITION_CONTROL
        controller.config.input_mode = INPUT_MODE_PASSTHROUGH
        # For a hold/stall test the controller must apply full current to resist
        # back-driving load; the vel-limit current throttle would otherwise cut
        # torque and let the joint run away, and overspeed would fault on a sag.
        controller.config.enable_current_mode_vel_limit = False
        controller.config.enable_overspeed_error = False
        if args.goto_load_turns is not None:
            controller.config.vel_limit = args.goto_vel_limit
        controller.input_pos = zero["motor"]
        axis.config.watchdog_timeout = args.watchdog_timeout
        axis.watchdog_feed()
        axis.config.enable_watchdog = True

        axis.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
        time.sleep(0.2)
        if axis.current_state != AXIS_STATE_CLOSED_LOOP_CONTROL:
            raise RuntimeError(
                f"Closed-loop entry failed: state={axis.current_state} "
                f"errors={errors(device, args.load_feedback)}"
            )

        if args.goto_load_turns is not None:
            result["goto_coupling"] = drive_to_load_target(
                device, args, args.goto_load_turns)
            controller.config.vel_limit = old_config[3]
            # A live velocity->position mode switch leaves the velocity-loop
            # integrator winding the motor (drifts ~0.8 turns/s). Re-enter the
            # position loop cleanly from IDLE -- the joint rests at its gravity
            # equilibrium here (~0 A), so it barely moves while disarmed.
            controller.input_vel = 0.0
            axis.requested_state = AXIS_STATE_IDLE
            time.sleep(0.4)
            controller.config.control_mode = CONTROL_MODE_POSITION_CONTROL
            controller.config.input_mode = INPUT_MODE_PASSTHROUGH
            controller.input_pos = float(axis.encoder.pos_estimate)
            axis.watchdog_feed()
            axis.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
            time.sleep(0.2)
            if axis.current_state != AXIS_STATE_CLOSED_LOOP_CONTROL:
                raise RuntimeError(
                    f"Re-arm after positioning failed: state={axis.current_state} "
                    f"errors={errors(device, args.load_feedback)}")
            # Confirm the position loop actually holds before arming the test.
            print("Settling at target...", flush=True)
            settle_deadline = time.monotonic() + 3.0
            stable_since = None
            while True:
                axis.watchdog_feed()
                check_safe(device, args)
                vel = abs(float(axis.encoder.vel_estimate))
                if vel < 0.05:
                    stable_since = stable_since or time.monotonic()
                    if time.monotonic() - stable_since > 1.0:
                        break
                else:
                    stable_since = None
                if time.monotonic() > settle_deadline:
                    raise RuntimeError(
                        f"Did not settle after move (vel={vel:.2f} turns/s)")
                time.sleep(0.02)
            # Do NOT re-command input_pos here: the standing position error
            # against the fixed setpoint is what generates the holding torque.
            # Resetting the setpoint to the actual position zeros that error and
            # the joint falls. Use the encoder only as the drift reference.
            zero["motor"] = float(axis.encoder.pos_estimate)
            zero["load"] = float(device.axis1.encoder.pos_estimate)
            print(f"  settled; holding load={zero['load']:.4f} turns "
                  f"({zero['load']*360:.2f} deg)", flush=True)

        kt = float(axis.motor.config.torque_constant)
        print("\n" + "=" * 50, flush=True)
        print("  HOLDING 0 deg -- PUT THE WEIGHT ON NOW", flush=True)
        print(f"  logging for {args.duration:.0f} s", flush=True)
        print("=" * 50 + "\n", flush=True)

        started = time.monotonic()
        next_sample = started
        peak_iq = 0.0
        peak_temp = 0.0
        max_motor_drift = 0.0
        max_load_drift = 0.0
        while True:
            now = time.monotonic()
            elapsed = now - started
            axis.watchdog_feed()
            temperature = check_safe(device, args)
            iq = float(axis.motor.current_control.Iq_measured)
            m_drift = motor_drift_deg()
            peak_iq = max(peak_iq, abs(iq))
            peak_temp = max(peak_temp, temperature)
            max_motor_drift = max(max_motor_drift, abs(m_drift))
            l_drift = load_drift_deg() if args.load_feedback else None
            if l_drift is not None:
                max_load_drift = max(max_load_drift, abs(l_drift))

            if abs(m_drift) > args.max_drift_degrees:
                raise RuntimeError(
                    f"Output drift {m_drift:.2f} deg exceeded limit "
                    f"{args.max_drift_degrees:.2f} deg -- aborting"
                )

            if now >= next_sample:
                sample = {
                    "t_s": round(elapsed, 2),
                    "iq_a": round(iq, 3),
                    "motor_torque_nm": round(abs(iq) * kt, 3),
                    "output_torque_nm": round(abs(iq) * kt * args.gear_ratio, 2),
                    "motor_drift_deg": round(m_drift, 3),
                    "fet_c": round(temperature, 1),
                }
                if l_drift is not None:
                    sample["load_drift_deg"] = round(l_drift, 3)
                result["samples"].append(sample)
                line = (f"t={elapsed:6.1f}s  Iq={iq:6.2f}A  "
                        f"out_torque={abs(iq)*kt*args.gear_ratio:6.2f}Nm  "
                        f"motor_drift={m_drift:+6.2f}deg  fet={temperature:4.1f}C")
                if l_drift is not None:
                    line += f"  load_drift={l_drift:+6.2f}deg"
                print(line, flush=True)
                next_sample += args.sample_interval

            if elapsed >= args.duration:
                break
            time.sleep(0.02)

        iqs = [s["iq_a"] for s in result["samples"]]
        result["summary"] = {
            "peak_iq_a": round(peak_iq, 3),
            "mean_iq_a": round(statistics.fmean(iqs), 3) if iqs else None,
            "peak_output_torque_nm": round(peak_iq * kt * args.gear_ratio, 2),
            "max_motor_drift_deg": round(max_motor_drift, 3),
            "max_load_drift_deg": round(max_load_drift, 3) if args.load_feedback else None,
            "peak_fet_c": round(peak_temp, 1),
        }
        print("\n=== SUMMARY ===", flush=True)
        print(f"peak Iq           = {peak_iq:.2f} A", flush=True)
        print(f"peak output torque= {peak_iq*kt*args.gear_ratio:.2f} Nm", flush=True)
        print(f"max motor drift   = {max_motor_drift:.3f} deg", flush=True)
        if args.load_feedback:
            print(f"max load drift    = {max_load_drift:.3f} deg", flush=True)
        print(f"peak FET temp     = {peak_temp:.1f} C", flush=True)

        if args.wait_for_removal:
            # Keep holding so the load can be removed before the motor releases.
            if os.path.exists(args.release_file):
                os.remove(args.release_file)
            print("\n" + "*" * 56, flush=True)
            print("  HOLD COMPLETE -- STILL HOLDING THE LOAD.", flush=True)
            print("  REMOVE THE WEIGHT NOW. The holding current will fall to", flush=True)
            print("  ~0 A as it comes off. Then release with:", flush=True)
            print(f"      touch {args.release_file}", flush=True)
            print("*" * 56 + "\n", flush=True)
            wait_started = time.monotonic()
            next_note = wait_started
            while not os.path.exists(args.release_file):
                axis.watchdog_feed()
                temperature = check_safe(device, args)
                if abs(motor_drift_deg()) > args.max_drift_degrees:
                    raise RuntimeError("Drift exceeded limit while awaiting release")
                if time.monotonic() - wait_started > args.release_wait_timeout:
                    raise RuntimeError(
                        f"Release wait timed out after {args.release_wait_timeout:.0f}s")
                if time.monotonic() >= next_note:
                    iq = float(axis.motor.current_control.Iq_measured)
                    print(f"  holding... Iq={iq:+.2f}A "
                          f"out_torque={abs(iq)*kt*args.gear_ratio:.1f}Nm "
                          f"(touch {args.release_file} to release)", flush=True)
                    next_note += 10.0
                time.sleep(0.02)
            os.remove(args.release_file)
            print("Release signal received; powering down.", flush=True)
    finally:
        try:
            axis.watchdog_feed()
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
            result["final_errors"] = errors(device, args.load_feedback)
            with open(args.output, "w", encoding="utf-8") as result_file:
                json.dump(result, result_file, indent=2)
                result_file.write("\n")
            print(f"\nIDLE  errors={result['final_errors']}  results={args.output}",
                  flush=True)


if __name__ == "__main__":
    main()
