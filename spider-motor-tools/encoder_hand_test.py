#!/usr/bin/env python3
"""
Encoder signal-integrity check by hand-spinning the motor shaft.

The axis is held in IDLE (de-energised) so the rotor turns freely. Spin the
shaft by hand through several full turns, both directions, for the whole
window. The script samples the absolute encoder and reports:

  - spi_error_rate   : SPI link integrity (must be 0.0)
  - glitches         : implausible per-sample jumps (loose magnet / dropout)
  - span / turns     : how much you actually rotated
  - max step         : largest clean per-sample change

A healthy magnetic encoder: spi_error_rate == 0, zero glitches, smooth angle.

This is read-only on encoder config (assumes the encoder is already configured
and ready). For first-time bringup / re-configuration use the firmware repo's
tools/as5047p_hand_test.py or tools/mt6701_hand_test.py instead.
"""

import argparse
import os
import sys
import time

sys.path = [p for p in sys.path if p not in ("", ".", os.getcwd())]

import odrive
from odrive.enums import AXIS_STATE_IDLE


def parse_args():
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--motor-id", help="Motor number/label for this unit (e.g. 10).")
    p.add_argument("--serial-number")
    p.add_argument("--axis", type=int, default=0, choices=(0, 1))
    p.add_argument("--duration", type=float, default=25.0)
    p.add_argument("--rate", type=float, default=100.0, help="Samples per second.")
    p.add_argument("--glitch-counts", type=int, default=1800,
                   help="Per-sample jump (counts) above which a step is a glitch.")
    return p.parse_args()


def main():
    args = parse_args()
    dev = odrive.find_any(serial_number=args.serial_number, timeout=20)
    axis = getattr(dev, f"axis{args.axis}")
    e = axis.encoder
    cpr = int(e.config.cpr)

    axis.requested_state = AXIS_STATE_IDLE
    time.sleep(0.3)
    axis.error = 0
    e.error = 0

    if args.motor_id is not None:
        print(f"MOTOR #{args.motor_id}", flush=True)
    print(f"axis{args.axis} encoder: mode={e.config.mode} cpr={cpr} "
          f"cs_pin={getattr(e.config, 'abs_spi_cs_gpio_pin', '-')}", flush=True)
    print(f">>> SPIN THE MOTOR BY HAND NOW for {args.duration:.0f}s "
          f"(several full turns, both directions) <<<", flush=True)

    dt = 1.0 / args.rate
    half = cpr // 2
    samples = []
    last = int(e.count_in_cpr)
    total = 0
    max_step = 0
    glitches = 0
    spi_max = 0.0
    t0 = time.monotonic()
    while time.monotonic() - t0 < args.duration:
        c = int(e.count_in_cpr)
        d = c - last
        if d > half:
            d -= cpr
        if d < -half:
            d += cpr
        total += abs(d)
        max_step = max(max_step, abs(d))
        if abs(d) > args.glitch_counts:
            glitches += 1
        spi_max = max(spi_max, float(e.spi_error_rate))
        samples.append(c)
        last = c
        time.sleep(dt)

    span = max(samples) - min(samples)
    print(f"\nsamples={len(samples)}  span={span}/{cpr} "
          f"({span / cpr * 100:.0f}% of one turn)", flush=True)
    print(f"total swept = {total} counts = {total / cpr:.2f} full turns", flush=True)
    print(f"max per-sample step = {max_step} counts ({max_step / cpr * 360:.1f} deg)", flush=True)
    print(f"glitch events (>{args.glitch_counts} counts/sample) = {glitches}", flush=True)
    print(f"max spi_error_rate = {spi_max:.6f}", flush=True)
    print(f"encoder.error={int(e.error)} axis.error={int(axis.error)}", flush=True)

    ok = (spi_max == 0.0 and glitches == 0 and int(e.error) == 0)
    if total < cpr:
        print("RESULT: INCONCLUSIVE - rotate more next time (swept < 1 turn)", flush=True)
    else:
        print(f"RESULT: {'PASS - encoder signal clean' if ok else 'FAIL - see spi/glitch/error above'}",
              flush=True)


if __name__ == "__main__":
    main()
