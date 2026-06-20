#!/usr/bin/env python3

import argparse
import math
import os
import sys
import time

# Avoid shadowing the installed odrive package with this repository's
# tools/odrive source tree when this file is launched directly.
script_directory = os.path.dirname(os.path.abspath(__file__))
if sys.path and os.path.abspath(sys.path[0]) == script_directory:
    sys.path.pop(0)

import odrive
from odrive.enums import (
    AXIS_STATE_CLOSED_LOOP_CONTROL,
    AXIS_STATE_IDLE,
    CONTROL_MODE_POSITION_CONTROL,
    INPUT_MODE_PASSTHROUGH,
    INPUT_MODE_POS_FILTER,
    INPUT_MODE_TRAP_TRAJ,
)


def parse_args():
    parser = argparse.ArgumentParser(
        description="Run a guarded hold/motion endurance test on axis0."
    )
    parser.add_argument("--total-seconds", type=float, default=600.0)
    parser.add_argument("--initial-hold-seconds", type=float, default=300.0)
    parser.add_argument("--motion-seconds", type=float, default=60.0)
    parser.add_argument(
        "--motion-cycles",
        type=int,
        default=0,
        help="Stop the motion phase after this many complete zero-to-target-to-zero "
             "cycles. Zero keeps the duration-based behavior.",
    )
    parser.add_argument("--motion-degrees", type=float, default=20.0)
    parser.add_argument("--down-degrees", type=float, default=20.0)
    parser.add_argument("--maximum-travel-degrees", type=float, default=30.0)
    parser.add_argument("--current-limit", type=float, default=15.0)
    parser.add_argument("--velocity-limit", type=float, default=5.0)
    parser.add_argument("--positioning-velocity-limit", type=float, default=2.0)
    parser.add_argument("--release-velocity-limit", type=float, default=1.0)
    parser.add_argument("--fet-temperature-limit", type=float, default=85.0)
    parser.add_argument(
        "--backlash-degrees",
        type=float,
        default=0.0,
        help="Output-shaft backlash dead-zone in degrees. Widens displacement "
             "guards and hold-error tolerances so backlash does not trip them.",
    )
    parser.add_argument(
        "--pos-gain",
        type=float,
        default=20.0,
        help="Position controller gain. Reduce (e.g. 10-20) when backlash "
             "causes hunting/oscillation.",
    )
    parser.add_argument(
        "--motion-settle-degrees",
        type=float,
        default=2.5,
        help="Position error threshold (degrees) to declare a motion target reached.",
    )
    parser.add_argument(
        "--motion-target-timeout",
        type=float,
        default=10.0,
        help="Seconds allowed to reach each motion target before aborting.",
    )
    parser.add_argument(
        "--motion-input-mode",
        choices=("trap", "filter"),
        default="trap",
        help="Motion profile generator. 'filter' uses a critically damped "
             "second-order position filter.",
    )
    parser.add_argument(
        "--input-filter-bandwidth",
        type=float,
        default=6.0,
        help="Position-filter bandwidth in 1/s when --motion-input-mode=filter.",
    )
    parser.add_argument(
        "--trap-vel-limit",
        type=float,
        default=3.0,
        help="Trapezoidal trajectory cruise velocity for the motion phase (motor turns/s).",
    )
    parser.add_argument(
        "--trap-accel-limit",
        type=float,
        default=3.0,
        help="Trapezoidal trajectory acceleration/deceleration limit (motor turns/s²).",
    )
    parser.add_argument(
        "--vel-gain",
        type=float,
        default=0.08,
        help="Velocity proportional gain. Lower values reduce abrupt torque changes.",
    )
    parser.add_argument(
        "--vel-integrator-gain",
        type=float,
        default=0.3,
        help="Velocity integrator gain. Increase (e.g. 1-5) to build torque faster "
             "through gearbox stiction.",
    )
    parser.add_argument("--serial-number")
    return parser.parse_args()


def get_errors(device):
    axis = device.axis0
    load_encoder = device.axis1.encoder
    return (
        axis.error,
        axis.motor.error,
        axis.encoder.error,
        axis.controller.error,
        device.axis1.error,
        load_encoder.error,
    )


def check_common(device, origin, temperature_limit, maximum_travel_degrees, backlash_degrees=0.0):
    axis = device.axis0
    load_encoder = device.axis1.encoder
    errors = get_errors(device)
    if any(errors):
        raise RuntimeError(f"ODrive fault: {errors}")
    if not load_encoder.mt6701_debug_crc_ok:
        raise RuntimeError("MT6701 CRC is invalid")

    temperature = axis.fet_thermistor.temperature
    if not math.isfinite(temperature) or temperature > temperature_limit:
        raise RuntimeError(f"FET temperature limit exceeded: {temperature:.1f} C")

    displacement_degrees = (load_encoder.pos_estimate - origin) * 360.0
    lower_limit = -(5.0 + backlash_degrees)
    if displacement_degrees < lower_limit or displacement_degrees > maximum_travel_degrees:
        raise RuntimeError(
            f"Joint displacement guard exceeded: {displacement_degrees:.2f} degrees "
            f"(limits [{lower_limit:.1f}, {maximum_travel_degrees:.1f}])"
        )

    return displacement_degrees, temperature, errors


def move_to_position(
    device,
    target,
    velocity_limit,
    temperature_limit,
    label,
    timeout=30.0,
):
    axis = device.axis0
    load_encoder = device.axis1.encoder
    controller = axis.controller
    controller.config.vel_limit = velocity_limit
    controller.input_pos = target

    started = time.monotonic()
    next_report = 0.0
    saturation_started = None
    while True:
        time.sleep(0.05)
        elapsed = time.monotonic() - started
        errors = get_errors(device)
        position_error = (load_encoder.pos_estimate - target) * 360.0
        motor_velocity = axis.encoder.vel_estimate
        iq = axis.motor.current_control.Iq_measured
        temperature = axis.fet_thermistor.temperature

        if any(errors):
            raise RuntimeError(f"{label} fault: {errors}")
        if not load_encoder.mt6701_debug_crc_ok:
            raise RuntimeError(f"{label} MT6701 CRC is invalid")
        if not math.isfinite(temperature) or temperature > temperature_limit:
            raise RuntimeError(f"{label} FET temperature limit exceeded")
        if abs(load_encoder.pos_estimate) > 0.35:
            raise RuntimeError(
                f"{label} absolute travel guard exceeded: "
                f"{load_encoder.pos_estimate * 360.0:.2f} degrees"
            )

        if abs(iq) > 0.97 * axis.motor.config.current_lim:
            saturation_started = saturation_started or time.monotonic()
            if time.monotonic() - saturation_started > 5.0:
                raise RuntimeError(f"{label} current saturated for more than 5 seconds")
        else:
            saturation_started = None

        if elapsed >= next_report:
            print(
                f"{label} {elapsed:.1f}s position_error_deg={position_error:.2f} "
                f"motor_vel={motor_velocity:.2f} iq={iq:.2f}A "
                f"fet={temperature:.1f}C errors={errors}",
                flush=True,
            )
            next_report += 1.0

        if abs(position_error) < 1.5 and abs(motor_velocity) < 0.8:
            print(
                f"{label} reached position={load_encoder.pos_estimate:.6f}",
                flush=True,
            )
            return
        if elapsed > timeout:
            raise RuntimeError(
                f"{label} timeout with {position_error:.2f} degrees error"
            )


def monitor_hold(
    device,
    origin,
    target,
    duration,
    label,
    temperature_limit,
    maximum_travel_degrees,
    stats,
    backlash_degrees=0.0,
):
    axis = device.axis0
    load_encoder = device.axis1.encoder
    controller = axis.controller
    controller.input_pos = target

    started = time.monotonic()
    next_report = 0.0
    saturation_started = None

    while True:
        time.sleep(0.05)
        elapsed = time.monotonic() - started
        displacement, temperature, errors = check_common(
            device, origin, temperature_limit, maximum_travel_degrees, backlash_degrees
        )
        iq = axis.motor.current_control.Iq_measured
        position_error = (load_encoder.pos_estimate - target) * 360.0

        stats["peak_iq"] = max(stats["peak_iq"], abs(iq))
        stats["peak_temperature"] = max(stats["peak_temperature"], temperature)

        hold_error_limit = max(10.0, backlash_degrees + 5.0)
        if abs(position_error) > hold_error_limit:
            raise RuntimeError(
                f"{label} position error exceeded {hold_error_limit:.1f} degrees: {position_error:.2f}"
            )

        if abs(iq) > 0.97 * axis.motor.config.current_lim:
            saturation_started = saturation_started or time.monotonic()
            if time.monotonic() - saturation_started > 5.0:
                raise RuntimeError(f"{label} current saturated for more than 5 seconds")
        else:
            saturation_started = None

        if elapsed >= next_report:
            print(
                f"{label} {elapsed:.1f}s "
                f"position_error_deg={position_error:.2f} "
                f"displacement_deg={displacement:.2f} "
                f"iq={iq:.2f}A fet={temperature:.1f}C "
                f"vbus={device.vbus_voltage:.1f}V errors={errors}",
                flush=True,
            )
            next_report += 2.0

        if elapsed >= duration:
            return


def run_motion(
    device,
    origin,
    duration,
    amplitude_degrees,
    temperature_limit,
    maximum_travel_degrees,
    stats,
    backlash_degrees=0.0,
    settle_degrees=1.5,
    target_timeout=10.0,
    trap_vel_limit=2.0,
    trap_accel_limit=5.0,
    requested_cycles=0,
    input_mode="trap",
    input_filter_bandwidth=6.0,
):
    axis = device.axis0
    load_encoder = device.axis1.encoder
    controller = axis.controller
    amplitude_turns = amplitude_degrees / 360.0
    targets = (origin + amplitude_turns, origin)
    target_index = 0
    target_started = time.monotonic()
    phase_started = target_started
    next_report = 0.0
    cycles = 0
    target_settle_times = []
    saturation_started = None

    if input_mode == "filter":
        controller.config.input_filter_bandwidth = input_filter_bandwidth
        controller.config.input_mode = INPUT_MODE_POS_FILTER
    else:
        axis.trap_traj.config.vel_limit = trap_vel_limit
        axis.trap_traj.config.accel_limit = trap_accel_limit
        axis.trap_traj.config.decel_limit = trap_accel_limit
        controller.config.input_mode = INPUT_MODE_TRAP_TRAJ
    controller.config.vel_limit = trap_vel_limit * 1.5
    controller.input_pos = targets[target_index]

    while True:
        time.sleep(0.02)
        now = time.monotonic()
        elapsed = now - phase_started
        target_elapsed = now - target_started
        displacement, temperature, errors = check_common(
            device, origin, temperature_limit, maximum_travel_degrees, backlash_degrees
        )
        iq = axis.motor.current_control.Iq_measured
        motor_velocity = axis.encoder.vel_estimate
        target_error = (load_encoder.pos_estimate - targets[target_index]) * 360.0

        stats["peak_iq"] = max(stats["peak_iq"], abs(iq))
        stats["peak_temperature"] = max(stats["peak_temperature"], temperature)
        stats["peak_velocity"] = max(stats["peak_velocity"], abs(motor_velocity))

        if abs(iq) > 0.97 * axis.motor.config.current_lim:
            saturation_started = saturation_started or now
            if now - saturation_started > 1.0:
                raise RuntimeError("Motion current saturated for more than 1 second")
        else:
            saturation_started = None

        if elapsed >= next_report:
            print(
                f"MOTION {elapsed:.1f}s target_deg="
                f"{(targets[target_index] - origin) * 360.0:.1f} "
                f"displacement_deg={displacement:.2f} "
                f"target_error_deg={target_error:.2f} "
                f"motor_vel={motor_velocity:.2f} iq={iq:.2f}A "
                f"fet={temperature:.1f}C errors={errors}",
                flush=True,
            )
            next_report += 1.0

        if abs(target_error) < settle_degrees and abs(motor_velocity) < 0.8:
            target_settle_times.append(target_elapsed)
            target_index = 1 - target_index
            if target_index == 0:
                cycles += 1
                print(
                    f"MOTION cycle={cycles} "
                    f"down_settle={target_settle_times[-2]:.2f}s "
                    f"zero_settle={target_settle_times[-1]:.2f}s",
                    flush=True,
                )
            controller.input_pos = targets[target_index]
            target_started = now
        elif target_elapsed > target_timeout:
            raise RuntimeError(
                f"Motion target timeout at {displacement:.2f} degrees"
            )

        cycles_complete = requested_cycles > 0 and cycles >= requested_cycles
        duration_complete = requested_cycles == 0 and elapsed >= duration
        if cycles_complete or duration_complete:
            mean_settle = (
                sum(target_settle_times) / len(target_settle_times)
                if target_settle_times
                else 0.0
            )
            print(
                f"MOTION cycles_completed={cycles} "
                f"mean_target_settle={mean_settle:.2f}s",
                flush=True,
            )
            controller.config.input_mode = INPUT_MODE_PASSTHROUGH
            return


def main():
    args = parse_args()
    if args.motion_cycles < 0:
        raise ValueError("--motion-cycles must be zero or greater")
    final_hold_seconds = (
        args.total_seconds - args.initial_hold_seconds - args.motion_seconds
    )
    if final_hold_seconds < 0.0:
        raise ValueError("Phase durations exceed total duration")

    device = odrive.find_any(
        serial_number=args.serial_number,
        timeout=15,
    )
    axis = device.axis0
    load_encoder = device.axis1.encoder
    controller = axis.controller
    origin = 0.0
    down_position = args.down_degrees / 360.0

    old_config = (
        axis.motor.config.current_lim,
        controller.config.control_mode,
        controller.config.input_mode,
        controller.config.pos_gain,
        controller.config.vel_limit,
        controller.config.vel_gain,
        controller.config.vel_integrator_gain,
        controller.config.input_filter_bandwidth,
        axis.trap_traj.config.vel_limit,
        axis.trap_traj.config.accel_limit,
        axis.trap_traj.config.decel_limit,
    )
    stats = {
        "peak_iq": 0.0,
        "peak_temperature": axis.fet_thermistor.temperature,
        "peak_velocity": 0.0,
    }

    test_completed = False
    try:
        if any(get_errors(device)):
            raise RuntimeError(f"Pre-existing errors: {get_errors(device)}")
        if not (
            axis.motor.is_calibrated
            and axis.encoder.is_ready
            and load_encoder.mt6701_debug_crc_ok
        ):
            raise RuntimeError("Motor or encoder feedback is not ready")

        axis.motor.config.current_lim = args.current_limit
        controller.config.control_mode = CONTROL_MODE_POSITION_CONTROL
        controller.config.input_mode = INPUT_MODE_PASSTHROUGH
        controller.config.pos_gain = args.pos_gain
        controller.config.vel_limit = args.velocity_limit
        controller.config.vel_gain = args.vel_gain
        controller.config.vel_integrator_gain = args.vel_integrator_gain

        axis.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
        time.sleep(0.5)
        if axis.current_state != AXIS_STATE_CLOSED_LOOP_CONTROL:
            raise RuntimeError("Closed-loop entry failed")

        move_to_position(
            device,
            origin,
            args.positioning_velocity_limit,
            args.fet_temperature_limit,
            "MOVE_TO_ZERO",
        )
        controller.config.vel_limit = args.velocity_limit
        controller.input_pos = origin
        print(
            f"\nREADY_FOR_WEIGHT — motor holding position {origin:.6f}"
            f"  attach weight now, starting in 5 seconds...\n"
            f"  Plan: initial_hold={args.initial_hold_seconds:.0f}s  "
            f"motion={args.motion_seconds:.0f}s  "
            f"motion_cycles={args.motion_cycles or 'duration-based'}  "
            f"final_hold={final_hold_seconds:.0f}s  "
            f"release_down={args.down_degrees:.1f}deg",
            flush=True,
        )
        for i in range(5, 0, -1):
            print(f"  {i}...", flush=True)
            time.sleep(1.0)
        print("STARTING_TEST", flush=True)

        if args.initial_hold_seconds > 0.0:
            monitor_hold(
                device,
                origin,
                origin,
                args.initial_hold_seconds,
                "HOLD1",
                args.fet_temperature_limit,
                args.maximum_travel_degrees,
                stats,
                args.backlash_degrees,
            )
        if args.motion_seconds > 0.0 or args.motion_cycles > 0:
            run_motion(
                device,
                origin,
                args.motion_seconds,
                args.motion_degrees,
                args.fet_temperature_limit,
                args.maximum_travel_degrees,
                stats,
                args.backlash_degrees,
                args.motion_settle_degrees,
                args.motion_target_timeout,
                args.trap_vel_limit,
                args.trap_accel_limit,
                args.motion_cycles,
                args.motion_input_mode,
                args.input_filter_bandwidth,
            )
            move_to_position(
                device,
                origin,
                args.positioning_velocity_limit,
                args.fet_temperature_limit,
                "RETURN_TO_ZERO",
            )
            controller.config.vel_limit = args.velocity_limit
        if final_hold_seconds > 0.0:
            monitor_hold(
                device,
                origin,
                origin,
                final_hold_seconds,
                "HOLD2",
                args.fet_temperature_limit,
                args.maximum_travel_degrees,
                stats,
                args.backlash_degrees,
            )
        print(
            "TEST_COMPLETE "
            f"peak_iq={stats['peak_iq']:.2f}A "
            f"peak_fet={stats['peak_temperature']:.1f}C "
            f"peak_motor_velocity={stats['peak_velocity']:.2f}turn/s",
            flush=True,
        )
        test_completed = True
    finally:
        if test_completed and axis.current_state == AXIS_STATE_CLOSED_LOOP_CONTROL:
            try:
                print(
                    f"MOVE_DOWN_BEFORE_RELEASE target_deg={args.down_degrees:.1f}",
                    flush=True,
                )
                move_to_position(
                    device,
                    down_position,
                    args.release_velocity_limit,
                    args.fet_temperature_limit,
                    "MOVE_DOWN",
                )
            except Exception as ex:
                print(f"MOVE_DOWN_FAILED {ex}", flush=True)
        axis.requested_state = AXIS_STATE_IDLE
        time.sleep(0.4)
        (
            axis.motor.config.current_lim,
            controller.config.control_mode,
            controller.config.input_mode,
            controller.config.pos_gain,
            controller.config.vel_limit,
            controller.config.vel_gain,
            controller.config.vel_integrator_gain,
            controller.config.input_filter_bandwidth,
            axis.trap_traj.config.vel_limit,
            axis.trap_traj.config.accel_limit,
            axis.trap_traj.config.decel_limit,
        ) = old_config
        print(
            f"IDLE state={axis.current_state} "
            f"position={load_encoder.pos_estimate:.6f} "
            f"errors={get_errors(device)}",
            flush=True,
        )


if __name__ == "__main__":
    main()
