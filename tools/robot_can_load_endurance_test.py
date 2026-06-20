#!/usr/bin/env python3

import argparse
import math
import struct
import time

import serial

from robot_can_velocity_test import frame, parse, request


AXIS_STATE_IDLE = 1
AXIS_STATE_CLOSED_LOOP_CONTROL = 8
CONTROL_MODE_POSITION_CONTROL = 3
INPUT_MODE_TRAP_TRAJ = 5

AXIS0_HEARTBEAT = 0x001
AXIS0_MOTOR_ERROR = 0x003
AXIS0_ENCODER_ERROR = 0x004
AXIS0_ENCODER_ESTIMATES = 0x009
AXIS0_SET_STATE = 0x007
AXIS0_SET_MODES = 0x00B
AXIS0_SET_INPUT_POS = 0x00C
AXIS0_SET_VEL_LIMIT = 0x00F
AXIS0_SET_TRAJ_VEL_LIMIT = 0x011
AXIS0_SET_TRAJ_ACCEL_LIMITS = 0x012
AXIS0_GET_IQ = 0x014
AXIS0_CLEAR_ERRORS = 0x018
AXIS1_ENCODER_ERROR = 0x024
AXIS1_ENCODER_ESTIMATES = 0x029


def parse_args():
    parser = argparse.ArgumentParser(
        description="Cycle a geared joint between zero and a load-side target over CAN."
    )
    parser.add_argument("--port", default="/dev/cu.usbmodem101")
    parser.add_argument("--duration", type=float, default=300.0)
    parser.add_argument("--target-degrees", type=float, default=114.0)
    parser.add_argument("--minimum-degrees", type=float, default=-5.0)
    parser.add_argument("--maximum-degrees", type=float, default=120.0)
    parser.add_argument("--velocity-limit", type=float, default=8.0)
    parser.add_argument("--acceleration-limit", type=float, default=8.0)
    parser.add_argument("--positioning-velocity-limit", type=float, default=2.0)
    parser.add_argument("--settle-degrees", type=float, default=2.5)
    parser.add_argument("--target-timeout", type=float, default=12.0)
    parser.add_argument("--current-limit", type=float, default=15.0)
    parser.add_argument("--release-velocity-limit", type=float, default=2.0)
    return parser.parse_args()


class CanJoint:
    def __init__(self, port):
        self.bus = serial.Serial(port, 250000, timeout=0.02)
        self.bus.reset_input_buffer()
        self.axis_error = None
        self.axis_state = None
        self.motor_error = None
        self.encoder_error = None
        self.load_encoder_error = None
        self.load_position = None
        self.motor_velocity = None
        self.iq = None
        self.last_feedback = None

    def close(self):
        self.bus.close()

    def send(self, can_id, payload=b""):
        self.bus.write(frame(can_id, payload))

    def set_state(self, state):
        self.send(AXIS0_SET_STATE, struct.pack("<I", state))

    def set_target(self, turns):
        self.send(AXIS0_SET_INPUT_POS, struct.pack("<fhh", turns, 0, 0))

    def configure_trajectory(self, velocity, acceleration):
        self.send(AXIS0_SET_VEL_LIMIT, struct.pack("<f", velocity * 1.5))
        self.send(AXIS0_SET_TRAJ_VEL_LIMIT, struct.pack("<f", velocity))
        self.send(
            AXIS0_SET_TRAJ_ACCEL_LIMITS,
            struct.pack("<ff", acceleration, acceleration),
        )

    def poll(self):
        for can_id in (
            AXIS0_MOTOR_ERROR,
            AXIS0_ENCODER_ERROR,
            AXIS0_ENCODER_ESTIMATES,
            AXIS0_GET_IQ,
            AXIS1_ENCODER_ERROR,
            AXIS1_ENCODER_ESTIMATES,
        ):
            self.bus.write(request(can_id))

        deadline = time.monotonic() + 0.06
        while time.monotonic() < deadline:
            parsed = parse(self.bus.read_until(b"\r").strip().decode(errors="ignore"))
            if not parsed:
                continue
            can_id, data = parsed
            self.last_feedback = time.monotonic()
            if can_id == AXIS0_HEARTBEAT and len(data) == 8:
                self.axis_error, self.axis_state = struct.unpack("<II", data)
            elif can_id == AXIS0_MOTOR_ERROR and len(data) >= 4:
                self.motor_error = struct.unpack("<I", data[:4])[0]
            elif can_id == AXIS0_ENCODER_ERROR and len(data) >= 4:
                self.encoder_error = struct.unpack("<I", data[:4])[0]
            elif can_id == AXIS0_ENCODER_ESTIMATES and len(data) == 8:
                _, self.motor_velocity = struct.unpack("<ff", data)
            elif can_id == AXIS0_GET_IQ and len(data) == 8:
                _, self.iq = struct.unpack("<ff", data)
            elif can_id == AXIS1_ENCODER_ERROR and len(data) >= 4:
                self.load_encoder_error = struct.unpack("<I", data[:4])[0]
            elif can_id == AXIS1_ENCODER_ESTIMATES and len(data) == 8:
                self.load_position, _ = struct.unpack("<ff", data)

    def check(self, minimum_degrees, maximum_degrees):
        if self.last_feedback is None:
            raise RuntimeError("no CAN feedback")
        if time.monotonic() - self.last_feedback > 0.5:
            raise RuntimeError("CAN feedback timeout")

        errors = (
            self.axis_error,
            self.motor_error,
            self.encoder_error,
            self.load_encoder_error,
        )
        if any(error not in (None, 0) for error in errors):
            raise RuntimeError(
                "ODrive fault: "
                + ", ".join(
                    "unknown" if error is None else f"0x{error:x}" for error in errors
                )
            )
        if self.load_position is None or not math.isfinite(self.load_position):
            raise RuntimeError("invalid load position")

        load_degrees = self.load_position * 360.0
        if not minimum_degrees <= load_degrees <= maximum_degrees:
            raise RuntimeError(f"travel guard at {load_degrees:.2f} degrees")
        return load_degrees

    def wait_until_ready(self, timeout=2.0):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            self.poll()
            if (
                self.load_position is not None
                and self.motor_velocity is not None
                and self.iq is not None
            ):
                return
        raise RuntimeError("incomplete CAN telemetry")


def wait_for_target(
    joint,
    target_degrees,
    timeout,
    settle_degrees,
    minimum_degrees,
    maximum_degrees,
    current_limit,
    label,
    phase_deadline=None,
):
    target_turns = target_degrees / 360.0
    joint.set_target(target_turns)
    started = time.monotonic()
    next_report = 0.0
    saturation_started = None
    peak_iq = 0.0
    peak_velocity = 0.0

    while True:
        joint.poll()
        now = time.monotonic()
        elapsed = now - started
        load_degrees = joint.check(minimum_degrees, maximum_degrees)
        position_error = load_degrees - target_degrees
        velocity = joint.motor_velocity
        iq = joint.iq

        if iq is not None:
            peak_iq = max(peak_iq, abs(iq))
            if abs(iq) > 0.97 * current_limit:
                saturation_started = saturation_started or now
                if now - saturation_started > 1.0:
                    raise RuntimeError(f"{label} current saturated for over 1 second")
            else:
                saturation_started = None
        if velocity is not None:
            peak_velocity = max(peak_velocity, abs(velocity))

        if elapsed >= next_report:
            print(
                f"{label} {elapsed:.1f}s target={target_degrees:.1f}deg "
                f"load={load_degrees:.2f}deg error={position_error:.2f}deg "
                f"motor_vel={velocity if velocity is not None else float('nan'):.2f} "
                f"iq={iq if iq is not None else float('nan'):.2f}A "
                f"axis_error=0x{joint.axis_error or 0:x}",
                flush=True,
            )
            next_report += 1.0

        if (
            abs(position_error) <= settle_degrees
            and velocity is not None
            and abs(velocity) < 0.8
        ):
            return True, elapsed, peak_iq, peak_velocity
        if phase_deadline is not None and now >= phase_deadline:
            return False, elapsed, peak_iq, peak_velocity
        if elapsed > timeout:
            raise RuntimeError(
                f"{label} timeout with {position_error:.2f} degrees error"
            )


def main():
    args = parse_args()
    joint = CanJoint(args.port)
    test_completed = False
    peak_iq = 0.0
    peak_velocity = 0.0
    cycles = 0

    try:
        joint.wait_until_ready()
        initial_degrees = joint.check(args.minimum_degrees, args.maximum_degrees)
        print(f"CAN_READY initial_position={initial_degrees:.2f}deg", flush=True)

        joint.send(AXIS0_CLEAR_ERRORS)
        joint.send(
            AXIS0_SET_MODES,
            struct.pack("<ii", CONTROL_MODE_POSITION_CONTROL, INPUT_MODE_TRAP_TRAJ),
        )
        joint.configure_trajectory(
            args.positioning_velocity_limit, args.acceleration_limit
        )
        joint.set_target(0.0)
        joint.set_state(AXIS_STATE_CLOSED_LOOP_CONTROL)
        time.sleep(0.1)

        _, _, move_iq, move_velocity = wait_for_target(
            joint,
            0.0,
            args.target_timeout,
            args.settle_degrees,
            args.minimum_degrees,
            args.maximum_degrees,
            args.current_limit,
            "MOVE_TO_ZERO",
        )
        peak_iq = max(peak_iq, move_iq)
        peak_velocity = max(peak_velocity, move_velocity)

        joint.configure_trajectory(args.velocity_limit, args.acceleration_limit)
        started = time.monotonic()
        phase_deadline = started + args.duration
        next_target = args.target_degrees
        print(
            f"STARTING_CAN_TEST duration={args.duration:.0f}s "
            f"range=0..{args.target_degrees:.1f}deg "
            f"velocity_limit={args.velocity_limit:.1f}turn/s",
            flush=True,
        )

        while time.monotonic() < phase_deadline:
            reached, _, target_iq, target_velocity = wait_for_target(
                joint,
                next_target,
                args.target_timeout,
                args.settle_degrees,
                args.minimum_degrees,
                args.maximum_degrees,
                args.current_limit,
                "MOTION",
                phase_deadline,
            )
            peak_iq = max(peak_iq, target_iq)
            peak_velocity = max(peak_velocity, target_velocity)
            if not reached:
                break
            if next_target == 0.0:
                cycles += 1
                print(f"MOTION cycle={cycles}", flush=True)
                next_target = args.target_degrees
            else:
                next_target = 0.0

        test_completed = True
        print(
            f"CAN_TEST_COMPLETE cycles={cycles} peak_iq={peak_iq:.2f}A "
            f"peak_motor_velocity={peak_velocity:.2f}turn/s",
            flush=True,
        )
    finally:
        try:
            if test_completed:
                joint.configure_trajectory(
                    args.release_velocity_limit, args.acceleration_limit
                )
                wait_for_target(
                    joint,
                    args.target_degrees,
                    args.target_timeout,
                    args.settle_degrees,
                    args.minimum_degrees,
                    args.maximum_degrees,
                    args.current_limit,
                    "MOVE_TO_SAFE_POSITION",
                )
        finally:
            for _ in range(3):
                joint.set_state(AXIS_STATE_IDLE)
                time.sleep(0.03)
            try:
                joint.poll()
                final_degrees = joint.check(
                    args.minimum_degrees, args.maximum_degrees
                )
                print(
                    f"CAN_IDLE position={final_degrees:.2f}deg "
                    f"axis_error=0x{joint.axis_error or 0:x}",
                    flush=True,
                )
            finally:
                joint.close()


if __name__ == "__main__":
    main()
