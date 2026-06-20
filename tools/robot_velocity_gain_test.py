#!/usr/bin/env python3

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
    CONTROL_MODE_VELOCITY_CONTROL,
    INPUT_MODE_PASSTHROUGH,
)


def parse_args():
    parser = argparse.ArgumentParser(
        description="Run one guarded motor-velocity step for velocity-loop tuning."
    )
    parser.add_argument("--velocity", type=float, default=4.0)
    parser.add_argument("--duration", type=float, default=2.0)
    parser.add_argument("--vel-gain", type=float, required=True)
    parser.add_argument("--vel-integrator-gain", type=float, required=True)
    parser.add_argument("--current-limit", type=float, default=15.0)
    parser.add_argument("--minimum-degrees", type=float, default=3.0)
    parser.add_argument("--maximum-degrees", type=float, default=47.0)
    parser.add_argument("--fet-temperature-limit", type=float, default=85.0)
    parser.add_argument("--watchdog-timeout", type=float, default=1.0)
    parser.add_argument("--serial-number")
    return parser.parse_args()


def get_errors(device):
    axis = device.axis0
    return (
        axis.error,
        axis.motor.error,
        axis.encoder.error,
        axis.controller.error,
        device.axis1.error,
        device.axis1.encoder.error,
    )


def main():
    args = parse_args()
    device = odrive.find_any(serial_number=args.serial_number, timeout=15)
    axis = device.axis0
    controller = axis.controller
    load_encoder = device.axis1.encoder

    position_degrees = load_encoder.pos_estimate * 360.0
    if not args.minimum_degrees < position_degrees < args.maximum_degrees:
        raise RuntimeError(
            f"Start position {position_degrees:.2f} degrees is outside the "
            f"test range ({args.minimum_degrees}, {args.maximum_degrees})"
        )
    if any(get_errors(device)):
        raise RuntimeError(f"Pre-existing errors: {get_errors(device)}")
    if not load_encoder.mt6701_debug_crc_ok:
        raise RuntimeError("MT6701 CRC is invalid")

    # Negative motor velocity increases the configured load-side position.
    midpoint = 0.5 * (args.minimum_degrees + args.maximum_degrees)
    velocity_command = (
        -abs(args.velocity) if position_degrees <= midpoint else abs(args.velocity)
    )
    old_config = (
        axis.motor.config.current_lim,
        controller.config.control_mode,
        controller.config.input_mode,
        controller.config.vel_gain,
        controller.config.vel_integrator_gain,
        controller.config.vel_limit,
        axis.config.watchdog_timeout,
        axis.config.enable_watchdog,
    )

    samples = []
    peak_current = 0.0
    peak_temperature = axis.fet_thermistor.temperature
    rise_time = None
    started = None
    stop_reason = "duration"
    try:
        axis.motor.config.current_lim = args.current_limit
        controller.config.control_mode = CONTROL_MODE_VELOCITY_CONTROL
        controller.config.input_mode = INPUT_MODE_PASSTHROUGH
        controller.config.vel_gain = args.vel_gain
        controller.config.vel_integrator_gain = args.vel_integrator_gain
        controller.config.vel_limit = max(abs(velocity_command) * 1.5, 1.0)
        controller.input_vel = 0.0
        axis.config.watchdog_timeout = args.watchdog_timeout
        axis.watchdog_feed()
        axis.config.enable_watchdog = True

        axis.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
        time.sleep(0.05)
        axis.watchdog_feed()
        if axis.current_state != AXIS_STATE_CLOSED_LOOP_CONTROL:
            raise RuntimeError("Closed-loop entry failed")

        started = time.monotonic()
        controller.input_vel = velocity_command
        next_report = 0.0
        while True:
            time.sleep(0.05)
            axis.watchdog_feed()
            elapsed = time.monotonic() - started
            position_degrees = load_encoder.pos_estimate * 360.0
            velocity = axis.encoder.vel_estimate
            current = axis.motor.current_control.Iq_measured
            temperature = axis.fet_thermistor.temperature
            errors = get_errors(device)

            if any(errors):
                raise RuntimeError(f"ODrive fault: {errors}")
            if not load_encoder.mt6701_debug_crc_ok:
                raise RuntimeError("MT6701 CRC is invalid")
            if not math.isfinite(temperature) or temperature > args.fet_temperature_limit:
                raise RuntimeError(f"FET temperature limit exceeded: {temperature:.1f} C")
            if not args.minimum_degrees <= position_degrees <= args.maximum_degrees:
                stop_reason = f"travel_guard_{position_degrees:.2f}deg"
                break

            peak_current = max(peak_current, abs(current))
            peak_temperature = max(peak_temperature, temperature)
            samples.append(velocity)
            if rise_time is None and abs(velocity) >= 0.9 * abs(velocity_command):
                rise_time = elapsed

            if elapsed >= next_report:
                print(
                    f"STEP {elapsed:.2f}s load_deg={position_degrees:.2f} "
                    f"motor_vel={velocity:.2f} iq={current:.2f}A "
                    f"fet={temperature:.1f}C errors={errors}",
                    flush=True,
                )
                next_report += 0.25
            if elapsed >= args.duration:
                break
    finally:
        try:
            controller.input_vel = 0.0
            axis.watchdog_feed()
            time.sleep(0.1)
        finally:
            axis.requested_state = AXIS_STATE_IDLE
            time.sleep(0.05)
            axis.config.enable_watchdog = False
            (
                axis.motor.config.current_lim,
                controller.config.control_mode,
                controller.config.input_mode,
                controller.config.vel_gain,
                controller.config.vel_integrator_gain,
                controller.config.vel_limit,
                axis.config.watchdog_timeout,
                axis.config.enable_watchdog,
            ) = old_config

    steady_samples = samples[len(samples) // 2:]
    mean_velocity = statistics.fmean(steady_samples)
    velocity_stddev = (
        statistics.pstdev(steady_samples) if len(steady_samples) > 1 else 0.0
    )
    print(
        f"RESULT command={velocity_command:.2f} "
        f"rise_time={rise_time if rise_time is not None else 'not_reached'} "
        f"mean_velocity={mean_velocity:.2f} stddev={velocity_stddev:.2f} "
        f"peak_iq={peak_current:.2f}A peak_fet={peak_temperature:.1f}C "
        f"stop={stop_reason} "
        f"final_load_deg={load_encoder.pos_estimate * 360.0:.2f} "
        f"errors={get_errors(device)}",
        flush=True,
    )


if __name__ == "__main__":
    main()
