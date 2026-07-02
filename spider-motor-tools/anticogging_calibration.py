#!/usr/bin/env python3
"""
Anti-cogging calibration for an ODrive S joint motor (MKS ODrive-S, bench).

Cogging torque is the position-dependent detent torque of a PM motor. It is the
dominant cause of low-speed roughness / velocity ripple -- the harmonic-comp
work showed the low-speed "encoder error" was mostly cogging, not a sensor
defect (see AGENTS.md). Anti-cogging maps the holding torque at every position
over one motor turn and feeds it forward.

Firmware (v0.5.1): start_anticogging_calibration() steps the position controller
through 3600 points across one turn of the controller's position feedback, waits
for pos/vel error to settle within calib_pos_threshold/calib_vel_threshold
(counts), records the wound-up integrator torque into a 3600-entry cogging_map
indexed by axis0.encoder.pos_estimate, then feeds it forward.

This board's hard-won quirks (encode them or suffer):
  * DIRECT DRIVE ONLY. The map replays indexed by THIS axis' (motor) encoder,
    but calibration drives input_pos through the controller's position feedback.
    Refuses to run if split feedback (load_encoder_axis != axis) is active; the
    motor-indexed map stays valid after switching to split feedback later.
  * START NEAR pos 0. Calibration steps input_pos 0..1 turn; if the linear
    pos_estimate is far from 0 the first command slams the motor back many turns
    and faults. Run right after a cold power-up (absolute encoder ~within 1 turn).
  * GENTLE GAINS. The saved pos_gain may be tuned for a soft geared load-side
    loop; on a stiff direct-drive motor it overspeeds on the settle step. We use
    a low pos_gain + raised vel_limit + overspeed disabled during calibration.
  * QUIET THE SIBLING ABS ENCODER. If the other axis is in an absolute-SPI mode
    for a device that is absent on the bench (e.g. axis1 MT6701), its failed
    transactions stall the shared SPI bus and trip CONTROL_DEADLINE_MISSED during
    the long sweep. We set it to mode 0 (incremental) for the run -- mode 0 does
    NOT re-init the SPI peripheral, so axis0 stays healthy. It is restored to its
    original mode at the very end, BEFORE save, so flash keeps the real config.
  * SPI IS FRAGILE TO RE-INIT. A soft reboot or setting an abs mode re-inits the
    shared SPI and tends to wedge it at ~100% error (ABS_SPI_COM_FAIL); only a
    COLD POWER CYCLE recovers it. So: never reboot to retry -- power-cycle. After
    a successful save here, restoring the sibling to its abs mode may wedge the
    encoder; that is fine (we are done) -- power-cycle before using the board.

calib_anticogging is read-only: the firmware clears it only when the 3600-point
sweep completes. An aborted run leaves it set; since reboot wedges the encoder,
the only clean reset is a power cycle.

Config is RAM-only except the final save (cogging map + enable/pre_calibrated +
the restored sibling mode), unless --no-save.

NOTE: strips cwd from sys.path so a local ./odrive dir cannot shadow the package.
"""

import argparse
import os
import sys
import time

sys.path = [p for p in sys.path if p not in ("", ".", os.getcwd())]

import numpy as np
import odrive
from odrive.enums import (
    AXIS_STATE_CLOSED_LOOP_CONTROL,
    AXIS_STATE_IDLE,
    CONTROL_MODE_POSITION_CONTROL,
    CONTROL_MODE_VELOCITY_CONTROL,
    INPUT_MODE_PASSTHROUGH,
)


def parse_args():
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--motor-id")
    p.add_argument("--serial-number")
    p.add_argument("--axis", type=int, default=0, choices=(0, 1))
    p.add_argument("--current-limit", type=float, default=12.0)
    p.add_argument("--calib-pos-gain", type=float, default=15.0,
                   help="pos_gain during calibration (gentle; default 15).")
    p.add_argument("--calib-vel-limit", type=float, default=10.0,
                   help="vel_limit during calibration, motor t/s (default 10).")
    p.add_argument("--pos-threshold", type=float, default=1.0)
    p.add_argument("--vel-threshold", type=float, default=1.0)
    p.add_argument("--stall-timeout", type=float, default=10.0,
                   help="Loosen thresholds x1.5 if the index stalls this long (s).")
    p.add_argument("--max-minutes", type=float, default=20.0)
    p.add_argument("--compare-speed", type=float, default=1.0,
                   help="Low speed (motor t/s) for before/after ripple. 0 to skip.")
    p.add_argument("--keep-sibling-abs", action="store_true",
                   help="Do NOT quiet the other axis' abs encoder during the run.")
    p.add_argument("--no-save", action="store_true")
    return p.parse_args()


def err_tuple(axis):
    return (int(axis.error), int(axis.motor.error),
            int(axis.encoder.error), int(axis.controller.error))


def clear_errors(axis):
    axis.error = 0
    axis.motor.error = 0
    axis.encoder.error = 0
    axis.controller.error = 0


def measure_ripple(axis, m, c, e, speed, seconds=4.0):
    saved = (c.config.control_mode, c.config.input_mode, c.config.vel_limit)
    vels, iqs = [], []
    try:
        clear_errors(axis)
        c.config.vel_limit = abs(speed) * 1.5
        c.config.control_mode = CONTROL_MODE_VELOCITY_CONTROL
        c.config.input_mode = INPUT_MODE_PASSTHROUGH
        c.input_vel = 0.0
        axis.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
        time.sleep(0.3)
        if axis.current_state != AXIS_STATE_CLOSED_LOOP_CONTROL:
            return None
        c.input_vel = speed
        time.sleep(1.5)
        t0 = time.monotonic()
        while time.monotonic() - t0 < seconds:
            vels.append(float(e.vel_estimate))
            iqs.append(float(m.current_control.Iq_measured))
            if int(m.error):
                break
            time.sleep(0.003)
    finally:
        c.input_vel = 0.0
        axis.requested_state = AXIS_STATE_IDLE
        time.sleep(0.3)
        (c.config.control_mode, c.config.input_mode, c.config.vel_limit) = saved
    if len(vels) < 20:
        return None
    return (float(np.std(vels)), float(np.std(iqs)), float(np.mean(np.abs(iqs))))


def main():
    args = parse_args()
    print("Connecting to ODrive...", flush=True)
    dev = odrive.find_any(serial_number=args.serial_number, timeout=20)
    axis = getattr(dev, f"axis{args.axis}")
    m, e, c = axis.motor, axis.encoder, axis.controller
    ac = c.config.anticogging
    sib = getattr(dev, f"axis{1 - args.axis}").encoder

    # Cold-start health checks.
    if not e.is_ready or float(e.spi_error_rate) > 0.01:
        print(f"REFUSING TO RUN: axis{args.axis} encoder not healthy "
              f"(ready={e.is_ready}, spi_error_rate={float(e.spi_error_rate):.3f}). "
              f"Power-cycle the board (not a soft reboot) and run immediately.",
              flush=True)
        sys.exit(2)
    if c.config.load_encoder_axis != args.axis or c.config.vel_encoder_axis != args.axis:
        print("REFUSING TO RUN: split feedback active; calibrate in motor-feedback "
              "mode (load/vel_encoder_axis = this axis).", flush=True)
        sys.exit(2)
    if bool(ac.calib_anticogging):
        print("WARNING: calib_anticogging already set (a prior run aborted). The "
              "firmware can only clear it by completing; if this run also aborts, "
              "power-cycle before retrying.", flush=True)
    pe = float(e.pos_estimate)
    if abs(pe) > 2.0:
        print(f"REFUSING TO RUN: pos_estimate={pe:.1f} turns far from 0; would slam "
              f"the motor to 0. Power-cycle and run immediately.", flush=True)
        sys.exit(2)

    print(f"serial={format(dev.serial_number,'x')} axis{args.axis} "
          f"cpr={e.config.cpr} pole_pairs={m.config.pole_pairs} pos={pe:.3f}", flush=True)

    sib_mode = int(sib.config.mode)
    quiet_sibling = (not args.keep_sibling_abs) and (sib_mode & 0x100)
    if quiet_sibling:
        # mode 0 (incremental) does NOT re-init the shared SPI -> axis0 stays up.
        sib.config.mode = 0
        time.sleep(0.3)
        print(f"quieted sibling axis abs encoder (mode {sib_mode} -> 0); "
              f"axis{args.axis} spi_error_rate now {float(e.spi_error_rate):.3f}",
              flush=True)
        if float(e.spi_error_rate) > 0.01:
            print("WARNING: encoder errored after quieting sibling.", flush=True)

    before = None
    if args.compare_speed > 0:
        ac.anticogging_enabled = False
        before = measure_ripple(axis, m, c, e, args.compare_speed)
        if before:
            print(f"BEFORE @ {args.compare_speed:g} t/s: vel_std={before[0]:.4f} t/s  "
                  f"Iq_std={before[1]:.3f} A  |Iq|={before[2]:.3f} A", flush=True)

    saved = (m.config.current_lim, c.config.control_mode, c.config.input_mode,
             c.config.pos_gain, c.config.vel_limit, c.config.enable_overspeed_error,
             ac.calib_pos_threshold, ac.calib_vel_threshold)
    pos_thr, vel_thr = args.pos_threshold, args.vel_threshold
    stop_reason = "ok"
    t0 = time.monotonic()
    try:
        clear_errors(axis)
        m.config.current_lim = args.current_limit
        c.config.pos_gain = args.calib_pos_gain
        c.config.vel_limit = args.calib_vel_limit
        c.config.enable_overspeed_error = False
        ac.calib_pos_threshold = pos_thr
        ac.calib_vel_threshold = vel_thr
        c.config.control_mode = CONTROL_MODE_POSITION_CONTROL
        c.config.input_mode = INPUT_MODE_PASSTHROUGH
        c.input_pos = float(e.pos_estimate)
        axis.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
        time.sleep(0.4)
        if axis.current_state != AXIS_STATE_CLOSED_LOOP_CONTROL:
            stop_reason = f"closed-loop entry failed errs={err_tuple(axis)}"
            print("FAIL:", stop_reason, flush=True)
            return
        if not bool(ac.calib_anticogging):
            c.start_anticogging_calibration()
        print("calibrating 3600 points (motor turns ~1 rev slowly)...", flush=True)
        deadline = t0 + args.max_minutes * 60.0
        last_index, last_progress_t, last_print = -1, time.monotonic(), -360
        while True:
            now = time.monotonic()
            idx = int(ac.index)
            if (not bool(ac.calib_anticogging)) and bool(c.anticogging_valid):
                stop_reason = "ok"
                break
            errs = err_tuple(axis)
            if any(errs):
                stop_reason = f"axis fault errs={errs} at index {idx}"
                break
            if axis.current_state != AXIS_STATE_CLOSED_LOOP_CONTROL:
                stop_reason = f"left closed-loop (state={axis.current_state}) at index {idx}"
                break
            if idx != last_index:
                last_index, last_progress_t = idx, now
                if idx - last_print >= 360:
                    last_print = idx
                    print(f"  index {idx}/3600 (pos_thr={pos_thr:.1f} vel_thr={vel_thr:.1f})",
                          flush=True)
            elif now - last_progress_t > args.stall_timeout:
                pos_thr *= 1.5
                vel_thr *= 1.5
                ac.calib_pos_threshold = pos_thr
                ac.calib_vel_threshold = vel_thr
                last_progress_t = now
                print(f"  STALL at index {idx}: thresholds -> pos={pos_thr:.2f} "
                      f"vel={vel_thr:.2f}", flush=True)
                if pos_thr > 60.0:
                    stop_reason = f"stalled at index {idx}"
                    break
            if now >= deadline:
                stop_reason = f"time cap at index {idx}"
                break
            time.sleep(0.05)
    finally:
        axis.requested_state = AXIS_STATE_IDLE
        time.sleep(0.3)
        (m.config.current_lim, c.config.control_mode, c.config.input_mode,
         c.config.pos_gain, c.config.vel_limit, c.config.enable_overspeed_error,
         ac.calib_pos_threshold, ac.calib_vel_threshold) = saved

    print(f"\nstop_reason: {stop_reason}  (anticogging_valid={bool(c.anticogging_valid)})",
          flush=True)
    if stop_reason != "ok" or not bool(c.anticogging_valid):
        if quiet_sibling:
            sib.config.mode = sib_mode  # may wedge SPI; we are not saving
        print("Anti-cogging did NOT complete; nothing saved. If calib_anticogging "
              "is still set, POWER-CYCLE before retrying.", flush=True)
        sys.exit(1)

    ac.anticogging_enabled = True
    ac.pre_calibrated = True

    # Measure AFTER while the sibling is still quiet and the encoder is healthy.
    after = None
    if before is not None:
        after = measure_ripple(axis, m, c, e, args.compare_speed)

    # Restore the sibling's real mode BEFORE saving (re-init may wedge the SPI,
    # but we are finished reading the encoder now).
    if quiet_sibling:
        sib.config.mode = sib_mode

    if not args.no_save:
        dev.save_configuration()
        print("cogging map + flags saved to flash. POWER-CYCLE before using the "
              "board (restoring the sibling abs mode can wedge the shared SPI).",
              flush=True)
    else:
        print("cogging map valid in RAM (not saved; --no-save).", flush=True)

    if before is not None and after is not None:
        print(f"\nBEFORE @ {args.compare_speed:g} t/s: vel_std={before[0]:.4f} t/s  "
              f"Iq_std={before[1]:.3f} A", flush=True)
        print(f"AFTER  @ {args.compare_speed:g} t/s: vel_std={after[0]:.4f} t/s  "
              f"Iq_std={after[1]:.3f} A", flush=True)
        print("=== ANTI-COGGING BEFORE -> AFTER ===", flush=True)
        print(f"  velocity ripple (std): {before[0]:.4f} -> {after[0]:.4f} t/s  "
              f"({100*(after[0]-before[0])/max(before[0],1e-6):+.0f}%)", flush=True)
        print(f"  Iq ripple (std)      : {before[1]:.3f} -> {after[1]:.3f} A  "
              f"({100*(after[1]-before[1])/max(before[1],1e-6):+.0f}%)", flush=True)


if __name__ == "__main__":
    main()
