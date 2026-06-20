 #!/usr/bin/env python3
"""
Gearbox sticktion and stuck-point diagnostic.

Phase 1 – STEP MAP  : Moves the output shaft 1 degree at a time, recording
                      peak motor current for each step in both directions.
                      High-current steps reveal binding / stuck points.

Phase 2 – STICKTION : At evenly-spaced positions, ramps motor torque from
                      zero until the output shaft breaks away.
                      Reports breakaway current and direction asymmetry.

Supports two encoder modes:
  default           : load encoder on axis1 (MT6701 magnetic, high resolution)
  --motor-encoder   : motor encoder on axis0, direct drive (no gearbox)
  --motor-gearbox   : motor encoder on axis0, position scaled by --gear-ratio
  --no-load-encoder : hall encoder on axis0 only (use when axis1 disconnected)
                      Position is scaled by --gear-ratio.
"""

import argparse
import math
import os
import statistics
import sys
import time

script_directory = os.path.dirname(os.path.abspath(__file__))
if sys.path and os.path.abspath(sys.path[0]) == script_directory:
    sys.path.pop(0)

import odrive
from odrive.enums import (
    AXIS_STATE_CLOSED_LOOP_CONTROL,
    AXIS_STATE_IDLE,
    CONTROL_MODE_POSITION_CONTROL,
    CONTROL_MODE_TORQUE_CONTROL,
    CONTROL_MODE_VELOCITY_CONTROL,
    INPUT_MODE_PASSTHROUGH,
    INPUT_MODE_TRAP_TRAJ,
)


def parse_args():
    parser = argparse.ArgumentParser(
        description="Measure gearbox sticktion and stuck points."
    )
    parser.add_argument(
        "--sweep-degrees", type=float, default=25.0,
        help="Output shaft test range in degrees (default 25).",
    )
    parser.add_argument(
        "--step-degrees", type=float, default=1.0,
        help="Step size for the step map (output degrees, default 1).",
    )
    parser.add_argument(
        "--step-velocity", type=float, default=0.4,
        help="Motor velocity during each step (motor turns/s, default 0.4).",
    )
    parser.add_argument(
        "--step-accel", type=float, default=1.0,
        help="Motor acceleration for step moves (motor turns/s², default 1.0).",
    )
    parser.add_argument(
        "--positioning-velocity", type=float, default=1.0,
        help="Motor velocity for inter-position moves (motor turns/s, default 1.0).",
    )
    parser.add_argument(
        "--sticktion-positions", type=int, default=6,
        help="Number of evenly-spaced positions to test sticktion at (default 6).",
    )
    parser.add_argument(
        "--skip-step-map", action="store_true",
        help="Skip the position step map and run only the torque-ramp sticktion sweep.",
    )
    parser.add_argument(
        "--torque-ramp-rate", type=float, default=0.03,
        help="Torque ramp rate (Nm/s, default 0.03).",
    )
    parser.add_argument(
        "--max-test-torque", type=float, default=0.4,
        help="Safety ceiling on test torque (Nm, default 0.4).",
    )
    parser.add_argument(
        "--max-test-current", type=float,
        help="Phase-current ceiling in A. Overrides --max-test-torque and "
             "uses motor.config.torque_constant for the command conversion.",
    )
    parser.add_argument(
        "--current-ramp-rate", type=float, default=0.5,
        help="Phase-current ramp rate in A/s with --max-test-current (default 0.5).",
    )
    parser.add_argument(
        "--breakaway-degrees", type=float, default=0.3,
        help="Output motion threshold that counts as breakaway (degrees, default 0.3). "
             "Auto-raised to 1.0 when --no-load-encoder is active.",
    )
    parser.add_argument(
        "--max-breakaway-degrees", type=float, default=3.0,
        help="Maximum allowed motion during torque ramp before aborting (output degrees).",
    )
    parser.add_argument(
        "--max-ramp-velocity", type=float, default=1.0,
        help="Motor velocity safety ceiling during torque ramp in turns/s (default 1).",
    )
    parser.add_argument(
        "--current-limit", type=float, default=8.0,
        help="Motor current limit during test (A, default 8).",
    )
    parser.add_argument(
        "--pos-gain", type=float, default=10.0,
        help="Position controller gain (default 10, lower reduces hunting with sticktion).",
    )
    parser.add_argument(
        "--vel-gain", type=float, default=0.08,
        help="Velocity proportional gain (default 0.08).",
    )
    parser.add_argument(
        "--vel-integrator-gain", type=float, default=0.1,
        help="Velocity integrator gain (default 0.1, keep low to avoid hunting through sticktion).",
    )
    parser.add_argument(
        "--fet-temperature-limit", type=float, default=80.0,
    )
    parser.add_argument(
        "--motor-encoder", action="store_true",
        help="Use the configured motor encoder on axis0 directly, with no gearbox. "
             "Intended for the onboard AS5047P magnetic encoder.",
    )
    parser.add_argument(
        "--motor-gearbox", action="store_true",
        help="Use the configured motor encoder on axis0 with a gearbox and no "
             "load-side encoder. Position is scaled by --gear-ratio.",
    )
    parser.add_argument(
        "--no-load-encoder", action="store_true",
        help="Use motor hall encoder (axis0) only — axis1 MT6701 disconnected. "
             "Position is scaled by --gear-ratio.",
    )
    parser.add_argument(
        "--gear-ratio", type=float, default=34.0,
        help="Gearbox reduction (motor turns / output turns). Used with --no-load-encoder.",
    )
    parser.add_argument("--serial-number")
    return parser.parse_args()


# ---------------------------------------------------------------------------
# Helpers — all position values are in OUTPUT SHAFT DEGREES
# ---------------------------------------------------------------------------

def get_errors(device, no_load_encoder=False):
    axis = device.axis0
    if no_load_encoder:
        return (
            axis.error,
            axis.motor.error,
            axis.encoder.error,
            axis.controller.error,
        )
    return (
        axis.error,
        axis.motor.error,
        axis.encoder.error,
        axis.controller.error,
        device.axis1.error,
        device.axis1.encoder.error,
    )


def uses_axis0_encoder(args):
    return args.motor_encoder or args.motor_gearbox or args.no_load_encoder


def check_safe(device, args):
    errors = get_errors(device, uses_axis0_encoder(args))
    if any(errors):
        raise RuntimeError(f"ODrive fault: {errors}")
    if not uses_axis0_encoder(args) and not device.axis1.encoder.mt6701_debug_crc_ok:
        raise RuntimeError("MT6701 CRC invalid")
    temp = device.axis0.fet_thermistor.temperature
    if not math.isfinite(temp) or temp > args.fet_temperature_limit:
        raise RuntimeError(f"Temperature exceeded: {temp:.1f}C")
    return temp


def read_output_deg(device, args):
    """Current output shaft position in degrees."""
    if args.motor_encoder:
        return device.axis0.encoder.pos_estimate * 360.0
    if args.motor_gearbox or args.no_load_encoder:
        return device.axis0.encoder.pos_estimate * 360.0 / args.gear_ratio
    return device.axis1.encoder.pos_estimate * 360.0


def output_deg_to_input_turns(deg, args):
    """Convert output shaft degrees to the value for controller.input_pos."""
    if args.motor_encoder:
        return deg / 360.0
    if args.motor_gearbox or args.no_load_encoder:
        return deg / 360.0 * args.gear_ratio
    return deg / 360.0


def setup_trap(axis, vel, accel):
    axis.trap_traj.config.vel_limit = vel
    axis.trap_traj.config.accel_limit = accel
    axis.trap_traj.config.decel_limit = accel
    axis.controller.config.vel_limit = vel * 2.0
    axis.controller.config.input_mode = INPUT_MODE_TRAP_TRAJ


def wait_for_position(device, target_deg, settle_deg, timeout, label, args):
    started = time.monotonic()
    while True:
        time.sleep(0.03)
        check_safe(device, args)
        pos = read_output_deg(device, args)
        err = pos - target_deg
        vel = device.axis0.encoder.vel_estimate
        if abs(err) < settle_deg and abs(vel) < 0.5:
            return
        if time.monotonic() - started > timeout:
            if abs(err) < settle_deg * 3:
                print(
                    f"  {label} soft-settled at {pos:.2f}deg "
                    f"(target {target_deg:.2f}deg err={err:.2f}deg vel={vel:.2f})",
                    flush=True,
                )
                return
            raise RuntimeError(
                f"{label} timeout at {pos:.2f}deg "
                f"(target {target_deg:.2f}deg error {err:.2f}deg)"
            )


# ---------------------------------------------------------------------------
# Phase 1: Step map
# ---------------------------------------------------------------------------

def run_step_map(device, start_deg, end_deg, args, label):
    axis = device.axis0
    controller = axis.controller
    direction = 1.0 if end_deg >= start_deg else -1.0
    step = direction * abs(args.step_degrees)

    print(f"\n=== STEP MAP {label}: {start_deg:.1f} → {end_deg:.1f} deg ===", flush=True)

    controller.config.control_mode = CONTROL_MODE_POSITION_CONTROL
    setup_trap(axis, args.step_velocity, args.step_accel)

    step_results = []
    pos = start_deg

    while (direction > 0 and pos < end_deg - 0.01) or \
          (direction < 0 and pos > end_deg + 0.01):

        next_pos = pos + step
        if direction > 0:
            next_pos = min(next_pos, end_deg)
        else:
            next_pos = max(next_pos, end_deg)

        controller.input_pos = output_deg_to_input_turns(next_pos, args)

        peak_iq = 0.0
        t0 = time.monotonic()
        while True:
            time.sleep(0.02)
            check_safe(device, args)
            iq = abs(axis.motor.current_control.Iq_measured)
            peak_iq = max(peak_iq, iq)
            err = read_output_deg(device, args) - next_pos
            vel = axis.encoder.vel_estimate
            if abs(err) < 1.5 and abs(vel) < 0.5:
                break
            if time.monotonic() - t0 > 8.0:
                print(
                    f"  WARN step to {next_pos:.1f}deg timed out "
                    f"(err={err:.2f}deg iq={iq:.3f}A)",
                    flush=True,
                )
                break

        actual = read_output_deg(device, args)
        step_results.append((actual, peak_iq))
        bar = "#" * int(peak_iq * 25)
        print(f"  {actual:+6.1f}deg  peak_iq={peak_iq:.3f}A  {bar}", flush=True)
        pos = next_pos

    return step_results


# ---------------------------------------------------------------------------
# Phase 2: Sticktion (torque ramp)
# ---------------------------------------------------------------------------

def torque_ramp(device, start_pos_deg, direction, args):
    """
    Ramp motor torque until output breaks away or max_torque reached.
    Returns (breakaway_iq, breakaway_torque_cmd) or (None, None).
    """
    axis = device.axis0
    controller = axis.controller

    controller.config.control_mode = CONTROL_MODE_TORQUE_CONTROL
    controller.config.input_mode = INPUT_MODE_PASSTHROUGH
    controller.input_torque = 0.0

    # Position hold can leave the rotor relaxing into a cogging minimum when
    # torque mode takes over. Wait for that motion before defining zero motion.
    settle_deadline = time.monotonic() + 2.0
    stationary_since = None
    while stationary_since is None or time.monotonic() - stationary_since < 0.1:
        time.sleep(0.005)
        check_safe(device, args)
        if abs(axis.encoder.vel_estimate) < 0.02:
            stationary_since = stationary_since or time.monotonic()
        else:
            stationary_since = None
        if time.monotonic() >= settle_deadline:
            raise RuntimeError(
                f"Motor did not settle at zero torque; "
                f"velocity={axis.encoder.vel_estimate:.2f} turns/s"
            )
    start_pos_deg = read_output_deg(device, args)

    torque_constant = axis.motor.config.torque_constant
    if args.max_test_current is not None:
        max_torque = args.max_test_current * torque_constant
        torque_ramp_rate = args.current_ramp_rate * torque_constant
    else:
        max_torque = args.max_test_torque
        torque_ramp_rate = args.torque_ramp_rate

    torque = 0.0
    dt = 0.005
    breakaway_iq = None
    breakaway_torque = None
    peak_iq = 0.0
    peak_iq_setpoint = 0.0

    old_current_mode_vel_limit = controller.config.enable_current_mode_vel_limit
    old_overspeed_error = controller.config.enable_overspeed_error
    controller.config.enable_current_mode_vel_limit = False
    controller.config.enable_overspeed_error = False
    try:
        while torque < max_torque:
            time.sleep(dt)
            check_safe(device, args)

            torque = min(torque + torque_ramp_rate * dt, max_torque)
            controller.input_torque = direction * torque

            iq = axis.motor.current_control.Iq_measured
            iq_setpoint = axis.motor.current_control.Iq_setpoint
            peak_iq = max(peak_iq, abs(iq))
            peak_iq_setpoint = max(peak_iq_setpoint, abs(iq_setpoint))
            motion = abs(read_output_deg(device, args) - start_pos_deg)
            velocity = abs(axis.encoder.vel_estimate)

            if motion >= args.breakaway_degrees or motion >= args.max_breakaway_degrees:
                breakaway_iq = abs(iq)
                breakaway_torque = torque
                break
            if velocity >= args.max_ramp_velocity:
                raise RuntimeError(
                    f"Torque ramp velocity limit exceeded: "
                    f"{velocity:.2f} turns/s"
                )
    finally:
        controller.input_torque = 0.0
        controller.config.enable_current_mode_vel_limit = old_current_mode_vel_limit
        controller.config.enable_overspeed_error = old_overspeed_error

    if breakaway_iq is None:
        print(
            f"  diagnostic: peak Iq_setpoint={peak_iq_setpoint:.3f}A "
            f"Iq_measured={peak_iq:.3f}A at command cap",
            flush=True,
        )

    # An unloaded rotor can accelerate and coast far past the small breakaway
    # threshold. Actively brake it before returning to a position trajectory.
    controller.input_vel = 0.0
    controller.config.control_mode = CONTROL_MODE_VELOCITY_CONTROL
    brake_deadline = time.monotonic() + 3.0
    while abs(axis.encoder.vel_estimate) > 0.05:
        time.sleep(dt)
        check_safe(device, args)
        if time.monotonic() >= brake_deadline:
            raise RuntimeError(
                f"Motor did not stop after torque ramp; "
                f"velocity={axis.encoder.vel_estimate:.2f} turns/s"
            )

    hold_deg = read_output_deg(device, args)
    controller.input_pos = output_deg_to_input_turns(hold_deg, args)
    controller.config.control_mode = CONTROL_MODE_POSITION_CONTROL
    controller.config.input_mode = INPUT_MODE_PASSTHROUGH
    time.sleep(0.05)
    return breakaway_iq, breakaway_torque


def run_sticktion_sweep(device, start_deg, end_deg, args):
    axis = device.axis0
    controller = axis.controller
    n = max(args.sticktion_positions, 2)
    positions = [start_deg + (end_deg - start_deg) * i / (n - 1) for i in range(n)]

    print(f"\n=== STICKTION SWEEP: {n} positions ===", flush=True)
    results = []

    for i, target in enumerate(positions):
        print(f"\n--- [{i+1}/{n}] Moving to {target:.1f} deg ---", flush=True)

        controller.config.control_mode = CONTROL_MODE_POSITION_CONTROL
        setup_trap(axis, args.positioning_velocity, args.positioning_velocity * 2)
        controller.input_pos = output_deg_to_input_turns(target, args)
        wait_for_position(device, target, 1.5, 20.0, f"POS{i+1}", args)

        settled = read_output_deg(device, args)
        print(f"  Settled at {settled:.2f} deg", flush=True)
        time.sleep(0.3)

        for direction, label in ((1.0, "FWD"), (-1.0, "REV")):
            iq, torque = torque_ramp(device, settled, direction, args)

            if iq is not None:
                current_cmd = torque / axis.motor.config.torque_constant
                print(
                    f"  {label} breakaway: Iq_measured={iq:.3f}A  "
                    f"Iq_cmd={current_cmd:.3f}A  torque_cmd={torque:.3f}Nm",
                    flush=True,
                )
            else:
                cap = (
                    f"{args.max_test_current:.2f}A"
                    if args.max_test_current is not None
                    else f"{args.max_test_torque:.2f}Nm"
                )
                print(
                    f"  {label} NO BREAKAWAY within {cap} — STUCK",
                    flush=True,
                )
            results.append((settled, label, iq, torque))

            # Return to settled position between directions
            controller.config.control_mode = CONTROL_MODE_POSITION_CONTROL
            setup_trap(axis, args.positioning_velocity, args.positioning_velocity * 2)
            controller.input_pos = output_deg_to_input_turns(settled, args)
            wait_for_position(device, settled, 1.5, 10.0, "RECOVER", args)
            time.sleep(0.2)

    return results


# ---------------------------------------------------------------------------
# Summary
# ---------------------------------------------------------------------------

def print_step_summary(fwd, rev):
    print("\n=== STEP MAP SUMMARY ===", flush=True)
    all_steps = [("FWD", *r) for r in fwd] + [("REV", *r) for r in rev]
    sorted_steps = sorted(all_steps, key=lambda x: -x[2])
    print("Top 5 highest-current steps (likely stuck/binding points):", flush=True)
    for direction, pos, iq in sorted_steps[:5]:
        bar = "#" * int(iq * 25)
        print(f"  {direction} {pos:+6.1f}deg  {iq:.3f}A  {bar}", flush=True)


def print_sticktion_summary(results):
    print("\n=== STICKTION SUMMARY ===", flush=True)
    valid = [(p, d, iq, t) for p, d, iq, t in results if iq is not None]
    stuck = [(p, d) for p, d, iq, t in results if iq is None]

    if valid:
        by_pos = {}
        for p, d, iq, t in valid:
            by_pos.setdefault(round(p, 1), {})[d] = iq

        print(f"  {'Position':>8}  {'FWD(A)':>8}  {'REV(A)':>8}  {'Asym(A)':>8}", flush=True)
        for pos in sorted(by_pos):
            fwd_iq = by_pos[pos].get("FWD", float("nan"))
            rev_iq = by_pos[pos].get("REV", float("nan"))
            asym = abs(fwd_iq - rev_iq) if math.isfinite(fwd_iq + rev_iq) else float("nan")
            print(
                f"  {pos:>8.1f}  {fwd_iq:>8.3f}  {rev_iq:>8.3f}  {asym:>8.3f}",
                flush=True,
            )

        all_iqs = [iq for _, _, iq, _ in valid]
        print(
            f"\n  Mean breakaway: {statistics.fmean(all_iqs):.3f}A  "
            f"Max: {max(all_iqs):.3f}A  "
            f"Min: {min(all_iqs):.3f}A",
            flush=True,
        )

    for p, d in stuck:
        print(f"  *** STUCK at {p:.1f}deg direction={d} ***", flush=True)


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def main():
    args = parse_args()

    encoder_modes = sum(
        (args.motor_encoder, args.motor_gearbox, args.no_load_encoder)
    )
    if encoder_modes > 1:
        raise ValueError(
            "--motor-encoder, --motor-gearbox, and --no-load-encoder "
            "are mutually exclusive"
        )
    if args.max_test_current is not None:
        if args.max_test_current <= 0:
            raise ValueError("--max-test-current must be positive")
        if args.max_test_current > args.current_limit:
            raise ValueError("--max-test-current cannot exceed --current-limit")
        if args.current_ramp_rate <= 0:
            raise ValueError("--current-ramp-rate must be positive")

    # Hall encoder has ~8.5° motor resolution → ~0.24° output at 36:1.
    # Raise breakaway threshold automatically so we don't false-trigger on noise.
    if args.no_load_encoder and args.breakaway_degrees < 1.0:
        print(
            f"INFO: --no-load-encoder active, raising --breakaway-degrees "
            f"from {args.breakaway_degrees} to 1.0 deg (hall encoder resolution)",
            flush=True,
        )
        args.breakaway_degrees = 1.0

    print(
        f"Connecting to ODrive...  "
        f"encoder={
            'motor(axis0)' if args.motor_encoder
            else 'motor(axis0)+gearbox' if args.motor_gearbox
            else 'hall(axis0)' if args.no_load_encoder
            else 'MT6701(axis1)'
        }",
        flush=True,
    )
    device = odrive.find_any(serial_number=args.serial_number, timeout=15)
    axis = device.axis0
    controller = axis.controller

    errors = get_errors(device, uses_axis0_encoder(args))
    if any(errors):
        raise RuntimeError(f"Pre-existing errors: {errors}")
    if not uses_axis0_encoder(args) and not device.axis1.encoder.mt6701_debug_crc_ok:
        raise RuntimeError("MT6701 CRC invalid")
    if not (axis.motor.is_calibrated and axis.encoder.is_ready):
        raise RuntimeError("Motor or encoder not ready")

    old_config = (
        axis.motor.config.current_lim,
        controller.config.control_mode,
        controller.config.input_mode,
        controller.config.pos_gain,
        controller.config.vel_gain,
        controller.config.vel_integrator_gain,
        controller.config.vel_limit,
        controller.config.enable_current_mode_vel_limit,
        controller.config.enable_overspeed_error,
        axis.trap_traj.config.vel_limit,
        axis.trap_traj.config.accel_limit,
        axis.trap_traj.config.decel_limit,
    )

    start_deg = round(read_output_deg(device, args), 1)
    end_deg = start_deg + args.sweep_degrees

    print(
        f"Output position: {start_deg:.2f}deg  "
        f"Sweep: {start_deg:.1f}→{end_deg:.1f}deg  "
        f"Current limit: {args.current_limit}A  "
        + (
            f"Max test current: {args.max_test_current}A  "
            if args.max_test_current is not None
            else f"Max torque: {args.max_test_torque}Nm  "
        )
        + f"Torque constant: {axis.motor.config.torque_constant}Nm/A  "
        f"pos_gain={args.pos_gain}  vel_integrator_gain={args.vel_integrator_gain}  "
        f"Gear ratio: {
            args.gear_ratio if args.no_load_encoder
            else args.gear_ratio if args.motor_gearbox
            else '1 (direct drive)' if args.motor_encoder
            else 'n/a (load enc)'
        }",
        flush=True,
    )

    try:
        axis.motor.config.current_lim = args.current_limit
        controller.config.pos_gain = args.pos_gain
        controller.config.vel_gain = args.vel_gain
        controller.config.vel_integrator_gain = args.vel_integrator_gain
        controller.config.control_mode = CONTROL_MODE_POSITION_CONTROL
        setup_trap(axis, args.positioning_velocity, args.positioning_velocity * 2)

        # Set input_pos to current position BEFORE entering closed-loop
        # so the controller doesn't lurch on entry.
        controller.input_pos = output_deg_to_input_turns(start_deg, args)

        print(
            f"\nREADY — will sweep output {start_deg:.1f}° → {end_deg:.1f}° "
            f"({args.sweep_degrees:.0f}° range"
            + (
                f", {int(args.sweep_degrees / args.step_degrees)} steps each direction), "
                "then "
                if not args.skip_step_map
                else "), "
            )
            + f"sticktion at {args.sticktion_positions} positions.\n"
            f"Ctrl-C now to abort. Starting in:",
            flush=True,
        )
        for i in range(5, 0, -1):
            print(f"  {i}...", flush=True)
            time.sleep(1.0)
        print("GO\n", flush=True)

        axis.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
        time.sleep(0.3)
        if axis.current_state != AXIS_STATE_CLOSED_LOOP_CONTROL:
            raise RuntimeError("Closed-loop entry failed")
        time.sleep(0.2)

        if not args.skip_step_map:
            # Phase 1a: forward step map
            fwd_steps = run_step_map(device, start_deg, end_deg, args, "FWD")

            # Phase 1b: reverse step map
            rev_steps = run_step_map(device, end_deg, start_deg, args, "REV")

            print_step_summary(fwd_steps, rev_steps)

        # Phase 2: return to start then sticktion sweep
        controller.config.control_mode = CONTROL_MODE_POSITION_CONTROL
        setup_trap(axis, args.positioning_velocity, args.positioning_velocity * 2)
        controller.input_pos = output_deg_to_input_turns(start_deg, args)
        wait_for_position(device, start_deg, 1.5, 20.0, "PRE_STICKTION", args)

        sticktion_results = run_sticktion_sweep(device, start_deg, end_deg, args)

        print_sticktion_summary(sticktion_results)

        if not args.motor_encoder and not args.motor_gearbox:
            # Geared-joint tests return to their initial output pose. A free,
            # direct-drive rotor does not need this extra full-revolution move.
            controller.config.control_mode = CONTROL_MODE_POSITION_CONTROL
            setup_trap(axis, args.positioning_velocity, args.positioning_velocity * 2)
            controller.input_pos = output_deg_to_input_turns(start_deg, args)
            wait_for_position(device, start_deg, 1.5, 20.0, "RETURN_TO_START", args)

        print("\nTEST COMPLETE", flush=True)

    finally:
        axis.requested_state = AXIS_STATE_IDLE
        time.sleep(0.3)
        (
            axis.motor.config.current_lim,
            controller.config.control_mode,
            controller.config.input_mode,
            controller.config.pos_gain,
            controller.config.vel_gain,
            controller.config.vel_integrator_gain,
            controller.config.vel_limit,
            controller.config.enable_current_mode_vel_limit,
            controller.config.enable_overspeed_error,
            axis.trap_traj.config.vel_limit,
            axis.trap_traj.config.accel_limit,
            axis.trap_traj.config.decel_limit,
        ) = old_config
        print(
            f"IDLE  output={read_output_deg(device, args):.2f}deg  "
            f"errors={get_errors(device, uses_axis0_encoder(args))}",
            flush=True,
        )


if __name__ == "__main__":
    main()
