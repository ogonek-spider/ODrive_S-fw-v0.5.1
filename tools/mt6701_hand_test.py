#!/usr/bin/env python3

import argparse
import os
import sys
import time

# Avoid shadowing the installed odrive package with tools/odrive.
script_directory = os.path.dirname(os.path.abspath(__file__))
if sys.path and os.path.abspath(sys.path[0]) == script_directory:
    sys.path.pop(0)

import odrive


AXIS_STATE_IDLE = 1
MT6701_MODE = 261
MT6701_CPR = 16384
MT6701_CS_PIN = 6


def parse_args():
    parser = argparse.ArgumentParser(
        description="Configure and test an axis1 MT6701 by rotating it by hand."
    )
    parser.add_argument("--duration", type=float, default=15.0)
    parser.add_argument("--serial-number")
    parser.add_argument(
        "--configure",
        action="store_true",
        help="Apply the expected axis1 MT6701 settings before testing.",
    )
    parser.add_argument(
        "--save",
        action="store_true",
        help="Save configuration after a successful test.",
    )
    return parser.parse_args()


def main():
    args = parse_args()
    if args.duration <= 0:
        raise ValueError("--duration must be positive")
    if args.save and not args.configure:
        raise ValueError("--save requires --configure")

    print("Connecting to ODrive...", flush=True)
    device = odrive.find_any(serial_number=args.serial_number, timeout=15)
    device.axis0.requested_state = AXIS_STATE_IDLE
    device.axis0.config.enable_watchdog = False
    encoder = device.axis1.encoder

    if args.configure:
        # Configure mode last because its setter reinitializes the shared SPI
        # peripheral and immediately enables periodic absolute-encoder reads.
        if encoder.config.cpr != MT6701_CPR:
            encoder.config.cpr = MT6701_CPR
        if encoder.config.abs_spi_cs_gpio_pin != MT6701_CS_PIN:
            encoder.config.abs_spi_cs_gpio_pin = MT6701_CS_PIN
        if encoder.config.enable_phase_interpolation:
            encoder.config.enable_phase_interpolation = False
        if encoder.config.mode != MT6701_MODE:
            encoder.config.mode = MT6701_MODE
        if not encoder.config.pre_calibrated:
            encoder.config.pre_calibrated = True
        time.sleep(0.5)

    print(
        "config:"
        f" mode={encoder.config.mode}"
        f" cpr={encoder.config.cpr}"
        f" cs={encoder.config.abs_spi_cs_gpio_pin}"
        f" interpolation={encoder.config.enable_phase_interpolation}"
        f" pre_calibrated={encoder.config.pre_calibrated}"
        f" ready={encoder.is_ready}",
        flush=True,
    )
    print(
        f"errors: axis0={device.axis0.error} axis1={device.axis1.error}"
        f" encoder={encoder.error}",
        flush=True,
    )

    start_samples = int(encoder.mt6701_debug_sample_count)
    start_bad_crc = int(encoder.mt6701_debug_bad_crc_count)
    start_position = float(encoder.pos_estimate)
    low_position = start_position
    high_position = start_position
    start_raw_position = int(encoder.mt6701_debug_pos)
    low_raw_position = start_raw_position
    high_raw_position = start_raw_position

    print(
        f"Rotate the encoder by hand now for {args.duration:.0f} seconds.",
        flush=True,
    )
    started = time.monotonic()
    next_report = started
    while time.monotonic() - started < args.duration:
        position = float(encoder.pos_estimate)
        low_position = min(low_position, position)
        high_position = max(high_position, position)
        raw_position = int(encoder.mt6701_debug_pos)
        low_raw_position = min(low_raw_position, raw_position)
        high_raw_position = max(high_raw_position, raw_position)
        now = time.monotonic()
        if now >= next_report:
            print(
                f"  {now - started:5.1f}s:"
                f" position={position * 360.0:+9.2f} deg"
                f" raw={raw_position * 360.0 / MT6701_CPR:7.2f} deg"
                f" velocity={float(encoder.vel_estimate) * 360.0:+8.2f} deg/s"
                f" crc={'OK' if encoder.mt6701_debug_crc_ok else 'BAD'}",
                flush=True,
            )
            next_report = now + 1.0
        time.sleep(0.02)

    sample_delta = int(encoder.mt6701_debug_sample_count) - start_samples
    bad_crc_delta = int(encoder.mt6701_debug_bad_crc_count) - start_bad_crc
    movement_degrees = (high_position - low_position) * 360.0
    raw_movement_degrees = (
        high_raw_position - low_raw_position
    ) * 360.0 / MT6701_CPR
    passed = (
        encoder.config.mode == MT6701_MODE
        and encoder.config.cpr == MT6701_CPR
        and encoder.config.abs_spi_cs_gpio_pin == MT6701_CS_PIN
        and bool(encoder.config.pre_calibrated)
        and bool(encoder.is_ready)
        and bool(encoder.mt6701_debug_crc_ok)
        and sample_delta > 100
        and bad_crc_delta == 0
        and movement_degrees >= 20.0
        and int(encoder.error) == 0
    )

    print(
        f"result: {'PASS' if passed else 'FAIL'}"
        f" movement={movement_degrees:.2f} deg"
        f" raw_movement={raw_movement_degrees:.2f} deg"
        f" samples={sample_delta} bad_crc={bad_crc_delta}"
        f" end={float(encoder.pos_estimate) * 360.0:+.2f} deg",
        flush=True,
    )

    if args.save:
        if not passed:
            raise RuntimeError("Refusing to save because the hand test failed")
        print("Saving configuration; USB will reconnect...", flush=True)
        device.save_configuration()

    if not passed:
        raise SystemExit(1)


if __name__ == "__main__":
    main()
