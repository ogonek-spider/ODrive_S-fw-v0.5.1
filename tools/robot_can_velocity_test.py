#!/usr/bin/env python3

import argparse
import struct
import time

import serial


def frame(can_id, payload=b""):
    return f"t{can_id:03X}{len(payload):X}{payload.hex().upper()}\n".encode()


def request(can_id):
    return f"r{can_id:03X}0\n".encode()


def parse(line):
    if len(line) < 5 or line[0] != "t":
        return None
    try:
        can_id = int(line[1:4], 16)
        length = int(line[4], 16)
        payload = line[5:5 + length * 2]
        if len(payload) != length * 2:
            return None
        return can_id, bytes.fromhex(payload)
    except ValueError:
        return None


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--port", default="/dev/cu.usbmodem101")
    parser.add_argument("--velocity", type=float, default=2.0)
    parser.add_argument("--duration", type=float, default=0.7)
    parser.add_argument("--minimum-degrees", type=float, default=-3.0)
    parser.add_argument("--maximum-degrees", type=float, default=50.0)
    args = parser.parse_args()

    bus = serial.Serial(args.port, 250000, timeout=0.02)
    bus.reset_input_buffer()

    latest_load = None
    latest_velocity = None
    peak_iq = 0.0
    axis_error = 0

    def send_velocity(value):
        bus.write(frame(0x00D, struct.pack("<ff", value, 0.0)))

    # Clear an idle watchdog latch, configure velocity control, then enter closed loop.
    bus.write(frame(0x018))
    bus.write(frame(0x00B, struct.pack("<ii", 2, 1)))
    send_velocity(0.0)
    bus.write(frame(0x007, struct.pack("<I", 8)))
    time.sleep(0.05)

    started = time.monotonic()
    next_report = 0.0
    try:
        while time.monotonic() - started < args.duration:
            elapsed = time.monotonic() - started
            send_velocity(-abs(args.velocity))
            bus.write(request(0x029))  # axis1 MT6701 position
            bus.write(request(0x009))  # axis0 Hall velocity
            bus.write(request(0x014))  # axis0 current

            deadline = time.monotonic() + 0.04
            while time.monotonic() < deadline:
                raw = bus.read_until(b"\r").strip().decode(errors="ignore")
                parsed = parse(raw)
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
                if not args.minimum_degrees <= load_degrees <= args.maximum_degrees:
                    raise RuntimeError(f"travel guard at {load_degrees:.2f} degrees")
            if elapsed >= next_report:
                print(
                    f"CAN_STEP {elapsed:.2f}s load_deg="
                    f"{latest_load * 360.0 if latest_load is not None else float('nan'):.2f} "
                    f"motor_vel={latest_velocity if latest_velocity is not None else float('nan'):.2f} "
                    f"peak_iq={peak_iq:.2f}A error=0x{axis_error:x}",
                    flush=True,
                )
                next_report += 0.1
    finally:
        for _ in range(5):
            send_velocity(0.0)
            time.sleep(0.02)
        bus.write(frame(0x007, struct.pack("<I", 1)))
        time.sleep(0.1)

    print(
        f"CAN_RESULT load_deg={latest_load * 360.0:.2f} "
        f"motor_vel={latest_velocity:.2f} peak_iq={peak_iq:.2f}A",
        flush=True,
    )


if __name__ == "__main__":
    main()
