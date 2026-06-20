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
    AXIS_STATE_ENCODER_OFFSET_CALIBRATION,
    AXIS_STATE_IDLE,
    CONTROL_MODE_VELOCITY_CONTROL,
    ENCODER_MODE_HALL,
    ENCODER_MODE_SPI_ABS_AMS,
    INPUT_MODE_PASSTHROUGH,
)


def parse_args():
    parser = argparse.ArgumentParser(
        description="Sequentially compare Hall and onboard AS5047P feedback."
    )
    parser.add_argument("--serial-number")
    parser.add_argument(
        "--only",
        choices=("hall", "as5047p"),
        help="Test only one encoder instead of running the full sequence.",
    )
    parser.add_argument("--speeds", type=float, nargs="+", default=[0.5, 2.0, 5.0])
    parser.add_argument("--duration", type=float, default=2.0)
    parser.add_argument("--settle-time", type=float, default=0.5)
    parser.add_argument("--current-limit", type=float, default=10.0)
    parser.add_argument("--fet-temperature-limit", type=float, default=80.0)
    parser.add_argument("--watchdog-timeout", type=float, default=1.0)
    return parser.parse_args()


def errors(axis):
    return (
        int(axis.error),
        int(axis.motor.error),
        int(axis.encoder.error),
        int(axis.controller.error),
    )


def wait_for_idle(axis, timeout=20.0):
    deadline = time.monotonic() + timeout
    while axis.current_state != AXIS_STATE_IDLE:
        if time.monotonic() >= deadline:
            raise RuntimeError(
                f"Axis did not return to idle; state={axis.current_state}, "
                f"errors={errors(axis)}"
            )
        time.sleep(0.05)


def clear_errors(axis):
    axis.motor.error = 0
    axis.encoder.error = 0
    axis.controller.error = 0
    axis.error = 0
    time.sleep(0.1)


def configure_encoder(axis, mode):
    encoder = axis.encoder
    axis.requested_state = AXIS_STATE_IDLE
    wait_for_idle(axis)
    encoder.config.pre_calibrated = False
    if mode == ENCODER_MODE_HALL:
        encoder.config.cpr = 6 * axis.motor.config.pole_pairs
        encoder.config.enable_phase_interpolation = True
    else:
        encoder.config.cpr = 16384
        encoder.config.abs_spi_cs_gpio_pin = 7
        encoder.config.enable_phase_interpolation = False
    encoder.config.mode = mode
    time.sleep(0.5)
    clear_errors(axis)


def calibrate_encoder(axis):
    axis.requested_state = AXIS_STATE_ENCODER_OFFSET_CALIBRATION
    wait_for_idle(axis, timeout=60.0)
    if any(errors(axis)):
        raise RuntimeError(f"Encoder offset calibration failed: {errors(axis)}")
    if not axis.encoder.is_ready:
        raise RuntimeError("Encoder offset calibration did not mark encoder ready")


def run_step(axis, command, args):
    controller = axis.controller
    samples = []
    started = time.monotonic()
    controller.input_vel = command

    while time.monotonic() - started < args.duration:
        time.sleep(0.02)
        axis.watchdog_feed()
        elapsed = time.monotonic() - started
        temperature = axis.fet_thermistor.temperature
        current = axis.motor.current_control.Iq_measured
        velocity = axis.encoder.vel_estimate
        axis_errors = errors(axis)
        if any(axis_errors):
            raise RuntimeError(f"ODrive fault during step: {axis_errors}")
        if not math.isfinite(temperature) or temperature > args.fet_temperature_limit:
            raise RuntimeError(f"FET temperature limit exceeded: {temperature:.1f} C")
        if elapsed >= args.settle_time:
            samples.append((velocity, current, temperature))

    controller.input_vel = 0.0
    stop_deadline = time.monotonic() + 3.0
    while abs(axis.encoder.vel_estimate) > 0.1:
        axis.watchdog_feed()
        if time.monotonic() >= stop_deadline:
            raise RuntimeError("Motor did not stop within 3 seconds")
        time.sleep(0.02)

    velocities = [sample[0] for sample in samples]
    currents = [sample[1] for sample in samples]
    temperatures = [sample[2] for sample in samples]
    mean_velocity = statistics.fmean(velocities)
    mean_error = statistics.fmean(abs(value - command) for value in velocities)
    return {
        "command": command,
        "mean_velocity": mean_velocity,
        "velocity_stddev": statistics.pstdev(velocities),
        "mean_abs_error": mean_error,
        "mean_abs_iq": statistics.fmean(abs(value) for value in currents),
        "peak_abs_iq": max(abs(value) for value in currents),
        "peak_fet": max(temperatures),
        "spi_error_rate": axis.encoder.spi_error_rate,
    }


def test_encoder(axis, name, mode, args):
    configure_encoder(axis, mode)
    if mode == ENCODER_MODE_SPI_ABS_AMS and axis.encoder.spi_error_rate > 0.005:
        raise RuntimeError(
            f"AS5047P SPI error rate is too high: {axis.encoder.spi_error_rate}"
        )
    calibrate_encoder(axis)
    axis.encoder.config.pre_calibrated = True

    axis.controller.input_vel = 0.0
    axis.config.watchdog_timeout = args.watchdog_timeout
    axis.watchdog_feed()
    axis.config.enable_watchdog = True
    axis.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
    time.sleep(0.1)
    if axis.current_state != AXIS_STATE_CLOSED_LOOP_CONTROL:
        raise RuntimeError(
            f"{name} failed to enter closed loop: state={axis.current_state}, "
            f"errors={errors(axis)}"
        )

    results = []
    fault = None
    try:
        for speed in args.speeds:
            for direction in (1.0, -1.0):
                try:
                    result = run_step(axis, direction * abs(speed), args)
                except RuntimeError as exc:
                    fault = str(exc)
                    print(f"{name:7s} FAULT {fault}", flush=True)
                    break
                results.append(result)
                print(
                    f"{name:7s} cmd={result['command']:6.2f} "
                    f"mean={result['mean_velocity']:7.3f} "
                    f"noise={result['velocity_stddev']:7.4f} "
                    f"mae={result['mean_abs_error']:7.4f} "
                    f"iq_mean={result['mean_abs_iq']:6.3f}A "
                    f"iq_peak={result['peak_abs_iq']:6.3f}A "
                    f"fet={result['peak_fet']:5.1f}C "
                    f"spi_err={result['spi_error_rate']:.6f}",
                    flush=True,
                )
            if fault:
                break
    finally:
        axis.controller.input_vel = 0.0
        axis.watchdog_feed()
        time.sleep(0.2)
        axis.requested_state = AXIS_STATE_IDLE
        wait_for_idle(axis)
        axis.config.enable_watchdog = False
    return results, fault


def summarize(name, results, fault):
    if not results:
        print(f"SUMMARY {name:7s} no completed steps fault={fault}", flush=True)
        return
    print(
        f"SUMMARY {name:7s} "
        f"velocity_noise={statistics.fmean(r['velocity_stddev'] for r in results):.5f} "
        f"tracking_mae={statistics.fmean(r['mean_abs_error'] for r in results):.5f} "
        f"mean_abs_iq={statistics.fmean(r['mean_abs_iq'] for r in results):.4f}A "
        f"peak_abs_iq={max(r['peak_abs_iq'] for r in results):.4f}A "
        f"max_spi_error={max(r['spi_error_rate'] for r in results):.6f} "
        f"fault={fault or 'none'}",
        flush=True,
    )


def run_encoder_test(axis, name, mode, args):
    try:
        return test_encoder(axis, name, mode, args)
    except RuntimeError as exc:
        fault = str(exc)
        print(f"{name:7s} FAULT {fault}", flush=True)
        return [], fault


def main():
    args = parse_args()
    device = odrive.find_any(serial_number=args.serial_number, timeout=15)
    axis = device.axis0
    encoder = axis.encoder
    controller = axis.controller

    if any(speed <= 0 for speed in args.speeds):
        raise ValueError("All speeds must be positive")
    if args.duration <= args.settle_time:
        raise ValueError("--duration must be greater than --settle-time")
    if any(errors(axis)):
        raise RuntimeError(f"Pre-existing ODrive errors: {errors(axis)}")
    if not axis.motor.is_calibrated:
        raise RuntimeError("Motor must already be calibrated")

    original = {
        "encoder_mode": encoder.config.mode,
        "encoder_cpr": encoder.config.cpr,
        "encoder_cs": encoder.config.abs_spi_cs_gpio_pin,
        "encoder_pre_calibrated": encoder.config.pre_calibrated,
        "encoder_interpolation": encoder.config.enable_phase_interpolation,
        "encoder_offset": encoder.config.offset,
        "encoder_offset_float": encoder.config.offset_float,
        "motor_direction": axis.motor.config.direction,
        "current_limit": axis.motor.config.current_lim,
        "control_mode": controller.config.control_mode,
        "input_mode": controller.config.input_mode,
        "vel_limit": controller.config.vel_limit,
        "watchdog_timeout": axis.config.watchdog_timeout,
        "watchdog_enabled": axis.config.enable_watchdog,
    }

    results = {}
    try:
        axis.requested_state = AXIS_STATE_IDLE
        wait_for_idle(axis)
        axis.motor.config.current_lim = min(
            args.current_limit, original["current_limit"]
        )
        controller.config.control_mode = CONTROL_MODE_VELOCITY_CONTROL
        controller.config.input_mode = INPUT_MODE_PASSTHROUGH
        controller.config.vel_limit = max(args.speeds) * 1.5
        if args.only != "as5047p":
            print("Testing Hall feedback...", flush=True)
            results["Hall"] = run_encoder_test(
                axis, "Hall", ENCODER_MODE_HALL, args
            )
        if args.only != "hall":
            print("Testing onboard AS5047P feedback...", flush=True)
            results["AS5047P"] = run_encoder_test(
                axis, "AS5047P", ENCODER_MODE_SPI_ABS_AMS, args
            )
    finally:
        try:
            controller.input_vel = 0.0
            axis.watchdog_feed()
            axis.requested_state = AXIS_STATE_IDLE
            wait_for_idle(axis)
        finally:
            axis.config.enable_watchdog = False
            encoder.config.pre_calibrated = False
            encoder.config.cpr = original["encoder_cpr"]
            encoder.config.abs_spi_cs_gpio_pin = original["encoder_cs"]
            encoder.config.enable_phase_interpolation = original[
                "encoder_interpolation"
            ]
            encoder.config.mode = original["encoder_mode"]
            encoder.config.offset = original["encoder_offset"]
            encoder.config.offset_float = original["encoder_offset_float"]
            encoder.config.pre_calibrated = original["encoder_pre_calibrated"]
            axis.motor.config.direction = original["motor_direction"]
            axis.motor.config.current_lim = original["current_limit"]
            controller.config.control_mode = original["control_mode"]
            controller.config.input_mode = original["input_mode"]
            controller.config.vel_limit = original["vel_limit"]
            axis.config.watchdog_timeout = original["watchdog_timeout"]
            axis.config.enable_watchdog = original["watchdog_enabled"]
            clear_errors(axis)

    print("\nAggregate comparison:")
    for name, (encoder_results, fault) in results.items():
        summarize(name, encoder_results, fault)
    print("Original encoder configuration restored; axis0 is idle. Nothing saved.")


if __name__ == "__main__":
    main()
