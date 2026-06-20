#!/usr/bin/env python3

import argparse
import struct
import time

import serial

from robot_can_velocity_test import frame, parse, request


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--port", default="/dev/cu.usbmodem101")
    parser.add_argument("--target-degrees", type=float, required=True)
    parser.add_argument("--timeout", type=float, default=10.0)
    parser.add_argument("--settle-degrees", type=float, default=2.5)
    parser.add_argument("--settle-seconds", type=float, default=0.2)
    parser.add_argument("--minimum-degrees", type=float, default=-3.0)
    parser.add_argument("--maximum-degrees", type=float, default=55.0)
    parser.add_argument("--traj-velocity", type=float, default=3.0)
    parser.add_argument("--traj-acceleration", type=float, default=3.0)
    args = parser.parse_args()

    bus = serial.Serial(args.port, 250000, timeout=0.02)
    bus.reset_input_buffer()

    target_turns = args.target_degrees / 360.0
    target_payload = struct.pack("<fhh", target_turns, 0, 0)
    latest_load = None
    latest_velocity = None
    peak_iq = 0.0
    axis_error = 0
    minimum_seen = float("inf")
    maximum_seen = float("-inf")
    settled_since = None

    bus.write(frame(0x018))
    bus.write(frame(0x00B, struct.pack("<ii", 3, 5)))
    bus.write(frame(0x011, struct.pack("<f", args.traj_velocity)))
    bus.write(
        frame(
            0x012,
            struct.pack("<ff", args.traj_acceleration, args.traj_acceleration),
        )
    )
    bus.write(frame(0x007, struct.pack("<I", 8)))
    time.sleep(0.05)
    bus.write(frame(0x00C, target_payload))

    started = time.monotonic()
    next_report = 0.0
    try:
        while True:
            now = time.monotonic()
            elapsed = now - started
            bus.write(request(0x029))
            bus.write(request(0x009))
            bus.write(request(0x014))

            deadline = time.monotonic() + 0.04
            while time.monotonic() < deadline:
                parsed = parse(
                    bus.read_until(b"\r").strip().decode(errors="ignore")
                )
                if not parsed:
                    continue
                can_id, data = parsed
                if can_id == 0x001 and len(data) == 8:
                    axis_error, _ = struct.unpack("<II", data)
                elif can_id == 0x029 and len(data) == 8:
                    latest_load, _ = struct.unpack("<ff", data)
                elif can_id == 0x009 and len(data) == 8:
                    _, latest_velocity = struct.unpack("<ff", data)
                elif can_id == 0x014 and len(data) == 8:
                    _, iq = struct.unpack("<ff", data)
                    peak_iq = max(peak_iq, abs(iq))

            if axis_error and not (axis_error == 0x800 and elapsed < 0.25):
                raise RuntimeError(f"axis error 0x{axis_error:x}")

            if latest_load is not None:
                load_degrees = latest_load * 360.0
                minimum_seen = min(minimum_seen, load_degrees)
                maximum_seen = max(maximum_seen, load_degrees)
                if not args.minimum_degrees <= load_degrees <= args.maximum_degrees:
                    raise RuntimeError(f"travel guard at {load_degrees:.2f} degrees")

                position_ok = (
                    abs(load_degrees - args.target_degrees) <= args.settle_degrees
                )
                velocity_ok = (
                    latest_velocity is not None and abs(latest_velocity) < 0.8
                )
                if position_ok and velocity_ok:
                    settled_since = settled_since or now
                    if now - settled_since >= args.settle_seconds:
                        break
                else:
                    settled_since = None

            if elapsed >= next_report:
                print(
                    f"CAN_POS {elapsed:.2f}s target={args.target_degrees:.1f} "
                    f"load={latest_load * 360.0 if latest_load is not None else float('nan'):.2f} "
                    f"motor_vel={latest_velocity if latest_velocity is not None else float('nan'):.2f} "
                    f"peak_iq={peak_iq:.2f}A error=0x{axis_error:x}",
                    flush=True,
                )
                next_report += 0.25

            if elapsed >= args.timeout:
                raise RuntimeError("position settle timeout")
    finally:
        bus.write(frame(0x00C, target_payload))
        time.sleep(0.05)
        bus.write(frame(0x007, struct.pack("<I", 1)))
        time.sleep(0.1)

    elapsed = time.monotonic() - started
    print(
        f"CAN_POS_RESULT settle={elapsed:.2f}s "
        f"final={latest_load * 360.0:.2f}deg "
        f"min={minimum_seen:.2f}deg max={maximum_seen:.2f}deg "
        f"peak_iq={peak_iq:.2f}A",
        flush=True,
    )


if __name__ == "__main__":
    main()
