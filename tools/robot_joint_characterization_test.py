#!/usr/bin/env python3
"""Measure joint stiction and maximum practical 0-to-45-degree move speed."""

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
    CONTROL_MODE_TORQUE_CONTROL,
    CONTROL_MODE_VELOCITY_CONTROL,
    INPUT_MODE_PASSTHROUGH,
)


def parse_args():
    parser = argparse.ArgumentParser(
        description=(
            "Characterize a 0-to-45-degree geared joint using velocity control. "
            "The current physical pose is treated as 0 degrees."
        ),
        epilog=(
            "No load encoder: %(prog)s --motor-feedback "
            "--output no-load-characterization.json\n"
            "Load encoder on axis1: %(prog)s --load-feedback "
            "--output loaded-characterization.json"
        ),
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    feedback = parser.add_mutually_exclusive_group(required=True)
    feedback.add_argument(
        "--motor-feedback",
        action="store_true",
        help="No load-side encoder: derive output position from axis0 / gear ratio.",
    )
    feedback.add_argument(
        "--load-feedback",
        action="store_true",
        help="Use axis1 as the load-side output encoder.",
    )
    parser.add_argument("--gear-ratio", type=float, default=34.0)
    parser.add_argument(
        "--motor-direction",
        type=int,
        choices=(-1, 1),
        help=(
            "Motor velocity/torque sign that increases output position. "
            "Default: +1 for motor feedback; controller.position_direction "
            "for load feedback when available."
        ),
    )
    parser.add_argument("--travel-degrees", type=float, default=45.0)
    parser.add_argument("--stiction-positions", type=int, default=5)
    parser.add_argument("--max-test-current", type=float, default=12.0)
    parser.add_argument("--current-ramp-rate", type=float, default=2.0)
    parser.add_argument("--breakaway-degrees", type=float, default=0.03)
    parser.add_argument("--current-limit", type=float, default=15.0)
    parser.add_argument(
        "--speed-candidates",
        type=float,
        nargs="+",
        default=[2.0, 4.0, 6.0, 8.0, 10.0, 11.0, 12.0],
        help="Motor velocity candidates in turns/s.",
    )
    parser.add_argument(
        "--speed-repetitions",
        type=int,
        default=6,
        help=(
            "Repetitions per speed. Each repetition times forward, back, "
            "forward (default 6)."
        ),
    )
    parser.add_argument(
        "--auto-tune-speed",
        action="store_true",
        help="Automatically tune directional braking leads for each speed.",
    )
    parser.add_argument(
        "--tune-repetitions",
        type=int,
        default=2,
        help="Short repetitions per braking-lead tuning iteration (default 2).",
    )
    parser.add_argument(
        "--max-tune-iterations",
        type=int,
        default=4,
        help="Maximum braking-lead adjustments per speed (default 4).",
    )
    parser.add_argument(
        "--max-stop-lead-degrees",
        type=float,
        default=15.0,
        help="Maximum automatically selected braking lead (default 15 degrees).",
    )
    parser.add_argument("--move-accel", type=float, default=20.0)
    parser.add_argument("--positioning-speed", type=float, default=2.0)
    parser.add_argument("--position-tolerance", type=float, default=0.15)
    parser.add_argument(
        "--forward-stop-lead-degrees",
        type=float,
        default=4.0,
        help=(
            "Braking lead for increasing output position (default 4.0 degrees)."
        ),
    )
    parser.add_argument(
        "--reverse-stop-lead-degrees",
        type=float,
        default=4.0,
        help=(
            "Braking lead for decreasing output position (default 4.0 degrees)."
        ),
    )
    parser.add_argument("--max-overshoot", type=float, default=2.0)
    parser.add_argument("--max-ramp-velocity", type=float, default=1.0)
    parser.add_argument("--fet-temperature-limit", type=float, default=70.0)
    parser.add_argument("--watchdog-timeout", type=float, default=1.0)
    parser.add_argument("--serial-number")
    parser.add_argument(
        "--output",
        default="joint-characterization.json",
        help="JSON result path (default joint-characterization.json).",
    )
    parser.add_argument(
        "--skip-stiction", action="store_true", help="Run only the speed test."
    )
    parser.add_argument(
        "--skip-speed", action="store_true", help="Run only the stiction test."
    )
    return parser.parse_args()


def errors(device, args):
    axis = device.axis0
    values = [
        int(axis.error),
        int(axis.motor.error),
        int(axis.encoder.error),
        int(axis.controller.error),
    ]
    if args.load_feedback:
        values.extend((int(device.axis1.error), int(device.axis1.encoder.error)))
    return tuple(values)


def check_safe(device, args):
    fault = errors(device, args)
    if any(fault):
        raise RuntimeError(f"ODrive fault: {fault}")
    temperature = float(device.axis0.fet_thermistor.temperature)
    if not math.isfinite(temperature) or temperature > args.fet_temperature_limit:
        raise RuntimeError(f"FET temperature exceeded: {temperature:.1f} C")
    return temperature


def resolve_motor_direction(axis, args):
    if args.motor_direction is not None:
        return args.motor_direction
    if args.motor_feedback:
        return 1
    try:
        return -1 if axis.controller.config.position_direction < 0 else 1
    except AttributeError:
        return 1


class OutputPosition:
    def __init__(self, device, args):
        self.device = device
        self.args = args
        self.zero_turns = self.read_turns()

    def read_turns(self):
        if self.args.load_feedback:
            return float(self.device.axis1.encoder.pos_estimate)
        return float(self.device.axis0.encoder.pos_estimate) / self.args.gear_ratio

    def degrees(self):
        return (self.read_turns() - self.zero_turns) * 360.0


def enter_closed_loop(axis):
    axis.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
    time.sleep(0.2)
    if axis.current_state != AXIS_STATE_CLOSED_LOOP_CONTROL:
        raise RuntimeError(
            f"Closed-loop entry failed: state={axis.current_state}, "
            f"errors={(axis.error, axis.motor.error, axis.encoder.error, axis.controller.error)}"
        )


def stop_velocity(device, args, output=None, timeout=3.0):
    axis = device.axis0
    axis.controller.input_vel = 0.0
    deadline = time.monotonic() + timeout
    positions = []
    while abs(axis.encoder.vel_estimate) > 0.05:
        time.sleep(0.01)
        axis.watchdog_feed()
        check_safe(device, args)
        if output is not None:
            positions.append(output.degrees())
        if time.monotonic() >= deadline:
            raise RuntimeError(
                f"Motor did not stop; velocity={axis.encoder.vel_estimate:.2f} turns/s"
            )
    if output is not None:
        positions.append(output.degrees())
    return positions


def move_to(
    device,
    output,
    target_deg,
    max_speed,
    motor_direction,
    args,
    stop_leads=None,
):
    """Move with a software velocity profile and return move statistics."""
    axis = device.axis0
    controller = axis.controller
    controller.config.control_mode = CONTROL_MODE_VELOCITY_CONTROL
    controller.config.input_mode = INPUT_MODE_PASSTHROUGH

    start_deg = output.degrees()
    started = time.monotonic()
    peak_iq = 0.0
    peak_speed = 0.0
    max_position = start_deg
    min_position = start_deg
    timeout = max(10.0, abs(target_deg - start_deg) / 10.0 + 8.0)
    forward_lead, reverse_lead = stop_leads or (
        args.forward_stop_lead_degrees,
        args.reverse_stop_lead_degrees,
    )
    stop_lead = forward_lead if target_deg >= start_deg else reverse_lead

    while True:
        time.sleep(0.01)
        axis.watchdog_feed()
        temperature = check_safe(device, args)
        position = output.degrees()
        error_deg = target_deg - position
        peak_iq = max(peak_iq, abs(axis.motor.current_control.Iq_measured))
        peak_speed = max(peak_speed, abs(axis.encoder.vel_estimate))
        max_position = max(max_position, position)
        min_position = min(min_position, position)

        if abs(error_deg) <= stop_lead:
            break
        if time.monotonic() - started >= timeout:
            raise RuntimeError(
                f"Move timeout: target={target_deg:.2f} position={position:.2f}"
            )

        remaining_motor_turns = abs(error_deg) / 360.0 * args.gear_ratio
        braking_speed = math.sqrt(max(0.0, 2.0 * args.move_accel * remaining_motor_turns))
        command = min(max_speed, braking_speed)
        command = max(command, min(0.2, max_speed))
        controller.input_vel = (
            math.copysign(command, error_deg) * motor_direction
        )

    braking_positions = stop_velocity(device, args, output)
    for position in braking_positions:
        max_position = max(max_position, position)
        min_position = min(min_position, position)
    final_deg = output.degrees()
    overshoot = (
        max(0.0, max_position - target_deg)
        if target_deg >= start_deg
        else max(0.0, target_deg - min_position)
    )
    return {
        "start_deg": start_deg,
        "target_deg": target_deg,
        "final_deg": final_deg,
        "error_deg": final_deg - target_deg,
        "overshoot_deg": overshoot,
        "duration_s": time.monotonic() - started,
        "peak_iq_a": peak_iq,
        "peak_motor_speed_turns_s": peak_speed,
        "peak_fet_c": temperature,
        "stop_lead_degrees": stop_lead,
    }


def torque_ramp(device, output, output_direction, motor_direction, args):
    axis = device.axis0
    controller = axis.controller
    controller.input_torque = 0.0
    controller.config.input_mode = INPUT_MODE_PASSTHROUGH
    controller.config.control_mode = CONTROL_MODE_TORQUE_CONTROL

    stationary_since = None
    deadline = time.monotonic() + 2.0
    while stationary_since is None or time.monotonic() - stationary_since < 0.1:
        time.sleep(0.005)
        axis.watchdog_feed()
        check_safe(device, args)
        if abs(axis.encoder.vel_estimate) < 0.02:
            stationary_since = stationary_since or time.monotonic()
        else:
            stationary_since = None
        if time.monotonic() >= deadline:
            raise RuntimeError("Motor did not settle before torque ramp")

    start_deg = output.degrees()
    torque_constant = axis.motor.config.torque_constant
    current_cmd = 0.0
    peak_iq = 0.0
    old_vel_limit = controller.config.enable_current_mode_vel_limit
    old_overspeed = controller.config.enable_overspeed_error
    controller.config.enable_current_mode_vel_limit = False
    controller.config.enable_overspeed_error = False
    try:
        while current_cmd < args.max_test_current:
            time.sleep(0.005)
            axis.watchdog_feed()
            check_safe(device, args)
            current_cmd = min(
                args.max_test_current,
                current_cmd + args.current_ramp_rate * 0.005,
            )
            controller.input_torque = (
                output_direction * motor_direction * current_cmd * torque_constant
            )
            measured_iq = abs(axis.motor.current_control.Iq_measured)
            peak_iq = max(peak_iq, measured_iq)
            movement = abs(output.degrees() - start_deg)
            if movement >= args.breakaway_degrees:
                return {
                    "breakaway": True,
                    "iq_measured_a": measured_iq,
                    "iq_command_a": current_cmd,
                    "movement_deg": movement,
                }
            if abs(axis.encoder.vel_estimate) >= args.max_ramp_velocity:
                raise RuntimeError("Torque-ramp velocity safety limit exceeded")
        return {
            "breakaway": False,
            "iq_measured_a": peak_iq,
            "iq_command_a": args.max_test_current,
            "movement_deg": abs(output.degrees() - start_deg),
        }
    finally:
        controller.input_torque = 0.0
        controller.config.enable_current_mode_vel_limit = old_vel_limit
        controller.config.enable_overspeed_error = old_overspeed
        controller.config.control_mode = CONTROL_MODE_VELOCITY_CONTROL
        controller.input_vel = 0.0
        stop_velocity(device, args)


def run_stiction(device, output, motor_direction, args):
    positions = [
        args.travel_degrees * i / (max(2, args.stiction_positions) - 1)
        for i in range(max(2, args.stiction_positions))
    ]
    results = []
    print("\n=== STICTION TEST ===", flush=True)
    for index, position in enumerate(positions, 1):
        move = move_to(
            device, output, position, args.positioning_speed, motor_direction, args
        )
        print(
            f"[{index}/{len(positions)}] position={move['final_deg']:.2f} deg",
            flush=True,
        )
        settled_position = output.degrees()
        for direction, label in ((1, "FWD"), (-1, "REV")):
            result = torque_ramp(
                device, output, direction, motor_direction, args
            )
            result.update({"position_deg": settled_position, "direction": label})
            results.append(result)
            status = (
                f"{result['iq_measured_a']:.3f} A"
                if result["breakaway"]
                else f"NO BREAKAWAY at {result['iq_command_a']:.1f} A"
            )
            print(f"  {label}: {status}", flush=True)
            move_to(
                device,
                output,
                settled_position,
                args.positioning_speed,
                motor_direction,
                args,
            )
    return results


def run_speed_repetitions(
    device,
    output,
    motor_direction,
    speed,
    repetitions,
    stop_leads,
    args,
    label,
):
    runs = []
    move_to(
        device,
        output,
        0.0,
        args.positioning_speed,
        motor_direction,
        args,
        stop_leads,
    )
    for repetition in range(1, repetitions + 1):
        forward_1 = move_to(
            device,
            output,
            args.travel_degrees,
            speed,
            motor_direction,
            args,
            stop_leads,
        )
        reverse = move_to(
            device, output, 0.0, speed, motor_direction, args, stop_leads
        )
        forward_2 = move_to(
            device,
            output,
            args.travel_degrees,
            speed,
            motor_direction,
            args,
            stop_leads,
        )
        runs.append(
            {
                "repetition": repetition,
                "forward_1": forward_1,
                "reverse": reverse,
                "forward_2": forward_2,
            }
        )
        print(
            f"  {label} {repetition}/{repetitions}: "
            f"forward={forward_1['duration_s']:.3f}s, "
            f"back={reverse['duration_s']:.3f}s, "
            f"forward={forward_2['duration_s']:.3f}s",
            flush=True,
        )
        if repetition < repetitions:
            move_to(
                device,
                output,
                0.0,
                args.positioning_speed,
                motor_direction,
                args,
                stop_leads,
            )
    return runs


def summarize_speed_runs(speed, runs, stop_leads, args):
    forward_moves = [
        run[key]
        for run in runs
        for key in ("forward_1", "forward_2")
    ]
    reverse_moves = [run["reverse"] for run in runs]
    all_moves = forward_moves + reverse_moves
    failures = [
        {
            "repetition": run["repetition"],
            "leg": key,
            "error_deg": run[key]["error_deg"],
            "overshoot_deg": run[key]["overshoot_deg"],
        }
        for run in runs
        for key in ("forward_1", "reverse", "forward_2")
        if abs(run[key]["error_deg"]) > args.max_overshoot
        or run[key]["overshoot_deg"] > args.max_overshoot
    ]
    forward_times = [move["duration_s"] for move in forward_moves]
    reverse_times = [move["duration_s"] for move in reverse_moves]
    return {
        "motor_speed_turns_s": speed,
        "output_speed_deg_s": speed * 360.0 / args.gear_ratio,
        "passed": not failures,
        "repetitions": len(runs),
        "stop_leads_deg": {
            "forward": stop_leads[0],
            "reverse": stop_leads[1],
        },
        "runs": runs,
        "forward_mean_s": statistics.fmean(forward_times),
        "forward_worst_s": max(forward_times),
        "reverse_mean_s": statistics.fmean(reverse_times),
        "reverse_worst_s": max(reverse_times),
        "all_moves_mean_s": statistics.fmean(forward_times + reverse_times),
        "peak_iq_a": max(move["peak_iq_a"] for move in all_moves),
        "failures": failures,
        "forward_mean_error_deg": statistics.fmean(
            move["error_deg"] for move in forward_moves
        ),
        "reverse_mean_error_deg": statistics.fmean(
            move["error_deg"] for move in reverse_moves
        ),
    }


def adjusted_stop_leads(result, args):
    forward = (
        result["stop_leads_deg"]["forward"]
        + result["forward_mean_error_deg"]
    )
    reverse = (
        result["stop_leads_deg"]["reverse"]
        - result["reverse_mean_error_deg"]
    )
    return (
        min(args.max_stop_lead_degrees, max(0.25, forward)),
        min(args.max_stop_lead_degrees, max(0.25, reverse)),
    )


def print_speed_result(result):
    print(
        f"{result['motor_speed_turns_s']:5.1f} motor turns/s: "
        f"0→45 mean={result['forward_mean_s']:.3f}s "
        f"worst={result['forward_worst_s']:.3f}s; "
        f"45→0 mean={result['reverse_mean_s']:.3f}s; "
        f"leads={result['stop_leads_deg']['forward']:.2f}/"
        f"{result['stop_leads_deg']['reverse']:.2f}deg; "
        f"peak_iq={result['peak_iq_a']:.2f}A "
        f"{'PASS' if result['passed'] else 'FAIL'}",
        flush=True,
    )
    for failure in result["failures"]:
        print(
            f"    FAIL run {failure['repetition']} {failure['leg']}: "
            f"final_error={failure['error_deg']:+.2f}deg "
            f"overshoot={failure['overshoot_deg']:.2f}deg",
            flush=True,
        )


def tune_speed(device, output, motor_direction, speed, initial_leads, args):
    leads = initial_leads
    history = []
    for iteration in range(1, args.max_tune_iterations + 1):
        print(
            f"  tune {iteration}/{args.max_tune_iterations}: "
            f"leads={leads[0]:.2f}/{leads[1]:.2f}deg",
            flush=True,
        )
        runs = run_speed_repetitions(
            device,
            output,
            motor_direction,
            speed,
            args.tune_repetitions,
            leads,
            args,
            "tune",
        )
        result = summarize_speed_runs(speed, runs, leads, args)
        history.append(result)
        if result["passed"]:
            return leads, history
        new_leads = adjusted_stop_leads(result, args)
        if all(abs(new - old) < 0.05 for new, old in zip(new_leads, leads)):
            break
        leads = new_leads
    return None, history


def run_speed_test(device, output, motor_direction, args):
    results = []
    print("\n=== 0-TO-45 SPEED TEST ===", flush=True)
    leads = (
        args.forward_stop_lead_degrees,
        args.reverse_stop_lead_degrees,
    )
    for speed in args.speed_candidates:
        tuning_history = []
        if args.auto_tune_speed:
            tuned_leads, tuning_history = tune_speed(
                device, output, motor_direction, speed, leads, args
            )
            if tuned_leads is None:
                print(f"{speed:5.1f} motor turns/s: TUNING FAILED", flush=True)
                results.append(
                    {
                        "motor_speed_turns_s": speed,
                        "passed": False,
                        "tuning_history": tuning_history,
                    }
                )
                break
            leads = tuned_leads

        runs = run_speed_repetitions(
            device,
            output,
            motor_direction,
            speed,
            args.speed_repetitions,
            leads,
            args,
            "run",
        )
        result = summarize_speed_runs(speed, runs, leads, args)
        result["tuning_history"] = tuning_history
        results.append(result)
        print_speed_result(result)
        if not result["passed"]:
            break
    return results


def print_summary(stiction, speed):
    print("\n=== SUMMARY ===", flush=True)
    valid_iq = [
        item["iq_measured_a"] for item in stiction if item["breakaway"]
    ]
    if valid_iq:
        print(
            f"Stiction mean={statistics.fmean(valid_iq):.3f}A "
            f"min={min(valid_iq):.3f}A max={max(valid_iq):.3f}A",
            flush=True,
        )
    passed = [item for item in speed if item["passed"]]
    if passed:
        fastest = passed[-1]
        print(
            f"Safe 0→45 time={fastest['forward_worst_s']:.3f}s "
            f"(mean={fastest['forward_mean_s']:.3f}s over "
            f"{fastest['repetitions'] * 2} forward moves)",
            flush=True,
        )
        print(
            f"Safe 45→0 time={fastest['reverse_worst_s']:.3f}s "
            f"(mean={fastest['reverse_mean_s']:.3f}s over "
            f"{fastest['repetitions']} reverse moves)",
            flush=True,
        )
        print(
            f"Selected speed={fastest['motor_speed_turns_s']:.1f} motor turns/s; "
            f"braking leads={fastest['stop_leads_deg']['forward']:.2f}/"
            f"{fastest['stop_leads_deg']['reverse']:.2f} deg",
            flush=True,
        )
    elif speed:
        print("No speed candidate passed the movement criteria.", flush=True)


def main():
    args = parse_args()
    if args.skip_stiction and args.skip_speed:
        raise ValueError("Cannot use --skip-stiction and --skip-speed together")
    if args.gear_ratio <= 0 or args.travel_degrees <= 0:
        raise ValueError("Gear ratio and travel must be positive")
    if args.max_test_current > args.current_limit:
        raise ValueError("--max-test-current cannot exceed --current-limit")
    if any(speed <= 0 for speed in args.speed_candidates):
        raise ValueError("All speed candidates must be positive")
    args.speed_candidates = sorted(set(args.speed_candidates))
    if args.speed_repetitions <= 0:
        raise ValueError("--speed-repetitions must be positive")
    if args.tune_repetitions <= 0:
        raise ValueError("--tune-repetitions must be positive")
    if args.max_tune_iterations <= 0:
        raise ValueError("--max-tune-iterations must be positive")
    if args.max_stop_lead_degrees <= 0:
        raise ValueError("--max-stop-lead-degrees must be positive")
    if args.forward_stop_lead_degrees <= 0:
        raise ValueError("--forward-stop-lead-degrees must be positive")
    if args.reverse_stop_lead_degrees <= 0:
        raise ValueError("--reverse-stop-lead-degrees must be positive")

    print("Connecting to ODrive...", flush=True)
    device = odrive.find_any(serial_number=args.serial_number, timeout=20)
    axis = device.axis0
    controller = axis.controller
    if any(errors(device, args)):
        raise RuntimeError(f"Pre-existing errors: {errors(device, args)}")
    if not axis.motor.is_calibrated or not axis.encoder.is_ready:
        raise RuntimeError("axis0 motor/encoder must be calibrated and ready")
    if args.load_feedback:
        load_encoder = device.axis1.encoder
        if hasattr(load_encoder, "mt6701_debug_crc_ok"):
            if not load_encoder.mt6701_debug_crc_ok:
                raise RuntimeError("Load-side MT6701 CRC is invalid")

    motor_direction = resolve_motor_direction(axis, args)
    output = OutputPosition(device, args)
    old_config = (
        axis.motor.config.current_lim,
        controller.config.control_mode,
        controller.config.input_mode,
        controller.config.vel_limit,
        controller.config.enable_overspeed_error,
        controller.config.enable_current_mode_vel_limit,
        axis.config.enable_watchdog,
        axis.config.watchdog_timeout,
    )
    result = {
        "timestamp_utc": datetime.now(timezone.utc).isoformat(),
        "feedback": "load" if args.load_feedback else "motor",
        "gear_ratio": args.gear_ratio,
        "travel_degrees": args.travel_degrees,
        "motor_direction": motor_direction,
        "auto_tune_speed": args.auto_tune_speed,
        "tune_repetitions": args.tune_repetitions,
        "max_tune_iterations": args.max_tune_iterations,
        "stiction": [],
        "speed": [],
    }

    try:
        axis.motor.config.current_lim = args.current_limit
        controller.config.vel_limit = max(args.speed_candidates) * 1.5
        controller.config.control_mode = CONTROL_MODE_VELOCITY_CONTROL
        controller.config.input_mode = INPUT_MODE_PASSTHROUGH
        controller.input_vel = 0.0
        axis.config.watchdog_timeout = args.watchdog_timeout
        axis.watchdog_feed()
        axis.config.enable_watchdog = True
        enter_closed_loop(axis)

        print(
            f"Current pose = 0 deg; test range = 0..{args.travel_degrees:.1f} deg; "
            f"feedback={result['feedback']}; motor_direction={motor_direction}",
            flush=True,
        )
        if not args.skip_stiction:
            result["stiction"] = run_stiction(
                device, output, motor_direction, args
            )
        if not args.skip_speed:
            result["speed"] = run_speed_test(
                device, output, motor_direction, args
            )
            passed_speeds = [
                item for item in result["speed"] if item.get("passed")
            ]
            if passed_speeds:
                fastest = passed_speeds[-1]
                result["selected_speed"] = {
                    "motor_speed_turns_s": fastest["motor_speed_turns_s"],
                    "output_speed_deg_s": fastest["output_speed_deg_s"],
                    "forward_stop_lead_deg": fastest["stop_leads_deg"]["forward"],
                    "reverse_stop_lead_deg": fastest["stop_leads_deg"]["reverse"],
                    "safe_forward_time_s": fastest["forward_worst_s"],
                    "safe_reverse_time_s": fastest["reverse_worst_s"],
                    "forward_mean_time_s": fastest["forward_mean_s"],
                    "reverse_mean_time_s": fastest["reverse_mean_s"],
                    "peak_iq_a": fastest["peak_iq_a"],
                }
        print_summary(result["stiction"], result["speed"])
    finally:
        try:
            controller.input_torque = 0.0
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
                controller.config.enable_overspeed_error,
                controller.config.enable_current_mode_vel_limit,
                axis.config.enable_watchdog,
                axis.config.watchdog_timeout,
            ) = old_config
            result["final_errors"] = errors(device, args)
            result["final_output_deg"] = output.degrees()
            with open(args.output, "w", encoding="utf-8") as result_file:
                json.dump(result, result_file, indent=2)
                result_file.write("\n")
            print(
                f"IDLE output={result['final_output_deg']:.2f} deg "
                f"errors={result['final_errors']} results={args.output}",
                flush=True,
            )


if __name__ == "__main__":
    main()
