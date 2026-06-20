#!/usr/bin/env python3

import argparse
import os
import sys
import time

script_directory = os.path.dirname(os.path.abspath(__file__))
if sys.path and os.path.abspath(sys.path[0]) == script_directory:
    sys.path.pop(0)

import odrive
from odrive.enums import AXIS_STATE_IDLE, ENCODER_MODE_SPI_ABS_AMS


def parse_args():
    parser = argparse.ArgumentParser(
        description="Monitor the onboard AS5047P while rotating a magnet by hand."
    )
    parser.add_argument("--serial-number")
    parser.add_argument("--interval", type=float, default=0.1)
    return parser.parse_args()


def clear_errors(axis):
    axis.motor.error = 0
    axis.encoder.error = 0
    axis.controller.error = 0
    axis.error = 0


def main():
    args = parse_args()
    if args.interval <= 0:
        raise ValueError("--interval must be positive")

    device = odrive.find_any(serial_number=args.serial_number, timeout=15)
    axis = device.axis0
    encoder = axis.encoder

    original = {
        "mode": encoder.config.mode,
        "cpr": encoder.config.cpr,
        "cs": encoder.config.abs_spi_cs_gpio_pin,
        "pre_calibrated": encoder.config.pre_calibrated,
        "interpolation": encoder.config.enable_phase_interpolation,
        "offset": encoder.config.offset,
        "offset_float": encoder.config.offset_float,
    }

    axis.requested_state = AXIS_STATE_IDLE
    time.sleep(0.3)
    if axis.current_state != AXIS_STATE_IDLE:
        raise RuntimeError(f"axis0 did not enter idle; state={axis.current_state}")

    clear_errors(axis)
    encoder.config.pre_calibrated = False
    encoder.config.cpr = 16384
    encoder.config.abs_spi_cs_gpio_pin = 7
    encoder.config.enable_phase_interpolation = False
    encoder.config.mode = ENCODER_MODE_SPI_ABS_AMS
    time.sleep(1.0)

    start_count = int(encoder.shadow_count)
    minimum = start_count
    maximum = start_count
    last_count = start_count

    print("AS5047P hand test active. The motor will not move.")
    print("Hold a diametric magnet centered over the sensor and rotate it.")
    print("A working setup should sweep through about 16384 counts per turn.")
    print("Press Ctrl+C to stop and restore the original encoder configuration.")
    print()

    try:
        while True:
            count = int(encoder.shadow_count)
            count_in_cpr = int(encoder.count_in_cpr)
            minimum = min(minimum, count)
            maximum = max(maximum, count)
            delta = count - last_count
            total_delta = count - start_count
            angle = (count_in_cpr % 16384) * 360.0 / 16384.0
            print(
                f"angle={angle:7.2f} deg  "
                f"count={count_in_cpr:6d}  "
                f"step={delta:+6d}  total={total_delta:+8d}  "
                f"span={maximum - minimum:6d}  "
                f"spi_error={float(encoder.spi_error_rate):.6f}  "
                f"errors=({int(axis.error)}, {int(encoder.error)})",
                flush=True,
            )
            last_count = count
            time.sleep(args.interval)
    except KeyboardInterrupt:
        print("\nStopping...")
    finally:
        axis.requested_state = AXIS_STATE_IDLE
        time.sleep(0.2)
        clear_errors(axis)
        encoder.config.pre_calibrated = False
        encoder.config.cpr = original["cpr"]
        encoder.config.abs_spi_cs_gpio_pin = original["cs"]
        encoder.config.enable_phase_interpolation = original["interpolation"]
        encoder.config.mode = original["mode"]
        encoder.config.offset = original["offset"]
        encoder.config.offset_float = original["offset_float"]
        encoder.config.pre_calibrated = original["pre_calibrated"]
        time.sleep(0.3)
        clear_errors(axis)
        print(
            f"Restored mode={encoder.config.mode}, cpr={encoder.config.cpr}; "
            "axis0 is idle. Nothing was saved."
        )


if __name__ == "__main__":
    main()
