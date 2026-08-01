#!/usr/bin/env python3
"""Tune a split-feedback joint's position gain with guarded steps, over USB.

    joint_step_tune.py --position 6-3

Sequence:

  1. configure split feedback (load_encoder_axis=1, vel_encoder_axis=0) and
     re-seed the joint encoder's linear position
  2. SIGN CHECK: one small step at a low gain, aborted the instant the joint
     moves the WRONG way. position_direction is an assumption about how the leg
     was assembled, and a wrong sign is a runaway on the first arm -- so it is
     established before anything larger is commanded.
  3. GAIN SWEEP: a step per candidate pos_gain, reporting settle time,
     residual error, overshoot and peak current; stops early once a gain rings
  4. report the best gain (nothing is saved unless --save)

Every armed section is watchdogged: the axis is disarmed on any error, on a
runaway past --abort-deg, and on Ctrl+C.

Sampling is 8 Hz over two properties (~16 reads/s). Do NOT raise it: hammering
USB in closed loop starves the 8 kHz control loop and trips
CONTROL_DEADLINE_MISSED -- measured on this hardware at 15 Hz x 2.

NOTE: strips cwd from sys.path so ./odrive does not shadow the installed pkg.
"""
import argparse
import os
import sys
import time

sys.path = [p for p in sys.path if p not in ("", ".", os.getcwd())]

import odrive

AXIS_STATE_IDLE = 1
AXIS_STATE_CLOSED_LOOP = 8
CONTROL_MODE_POSITION = 3
INPUT_MODE_PASSTHROUGH = 1

JOINT_NAMES = {1: "coxa", 2: "femur", 3: "knee"}

# USB reads in closed loop compete with the 8 kHz control loop, and this board
# runs BOTH absolute encoders on the shared SPI bus, which leaves the loop less
# headroom than a single-encoder joint. MEASURED here: 18 reads/s and even
# ~12 reads/s tripped MOTOR_ERROR_CONTROL_DEADLINE_MISSED. So the hot loop reads
# exactly ONE property -- joint position, which every abort test needs -- and
# everything else is sampled sparsely or only at the ends of the step.
SAMPLE_HZ = 5.0        # 5 reads/s in the hot loop
SPARSE_EVERY = 10      # Iq and the error tripwire: ~0.5 reads/s each
SETTLE_TOL_DEG = 0.5     # inside this counts as arrived
RESIDUAL_WINDOW = 0.5    # s of tail averaged for the steady-state error


def parse_args():
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--position", help="Joint label <leg>-<joint>, e.g. 6-3.")
    p.add_argument("--serial-number")
    p.add_argument("--step-deg", type=float, default=5.0,
                   help="Step size in JOINT degrees (default 5).")
    p.add_argument("--sign-step-deg", type=float, default=2.0,
                   help="Step size for the sign check (default 2).")
    p.add_argument("--settle-seconds", type=float, default=5.0,
                   help="How long to watch each step (default 5).")
    p.add_argument("--gains", type=float, nargs="+",
                   default=[60.0, 140.0, 300.0, 600.0],
                   help="pos_gain candidates, JOINT-side units (default 60 140 300 600).")
    p.add_argument("--sign-gains", type=float, nargs="+", default=[60.0, 200.0, 600.0],
                   help="pos_gain ladder for the sign check (default 60 200 600). Starts "
                        "soft so a wrong sign is gentle, then escalates: the gain is in "
                        "joint-side units, so a high-ratio joint needs hundreds before it "
                        "moves at all.")
    p.add_argument("--position-direction", type=int, choices=(1, -1),
                   help="Skip the sign search and force this sign.")
    p.add_argument("--vel-limit", type=float, default=2.0,
                   help="Motor-side velocity limit, turns/s (default 2). On a low-ratio "
                        "joint a few degrees of error already saturates this, so the "
                        "position loop runs bang-bang and overshoots; lower it to make "
                        "the approach controlled.")
    p.add_argument("--vel-gain", type=float,
                   help="Motor-side velocity P gain (default: leave as configured).")
    p.add_argument("--vel-integrator-gain", type=float,
                   help="Motor-side velocity I gain (default: leave as configured). "
                        "Winds up while a sticking joint refuses to move, then throws it "
                        "past the target -- lower it to cure stick-slip overshoot.")
    p.add_argument("--current-lim", type=float,
                   help="Temporarily cap motor current for the test.")
    p.add_argument("--abort-deg", type=float, default=20.0,
                   help="Disarm if the joint strays this far from the target (default 20).")
    p.add_argument("--save", action="store_true",
                   help="Write the winning gain to flash (default: report only).")
    return p.parse_args()


class Joint:
    """Armed-axis helper that always leaves the axis IDLE."""

    def __init__(self, dev):
        self.dev = dev
        self.ax = dev.axis0
        self.c = dev.axis0.controller
        self.load = dev.axis1.encoder

    def pos_deg(self):
        return self.load.pos_estimate * 360.0

    def motor_turn(self):
        """Motor-shaft position. Tracked alongside the joint because the two
        together separate 'gain too soft' (neither moves) from 'decoupled'
        (motor moves, joint does not) without ever leaving the position loop."""
        return self.dev.axis0.encoder.pos_estimate

    def tripped(self):
        """One-read fault tripwire for the hot loop.

        A motor, encoder or controller fault also raises MOTOR_FAILED /
        ENCODER_FAILED / CONTROLLER_FAILED on the axis, so this single read
        catches everything the five-read version did -- at a fifth of the USB
        traffic, which is itself what trips CONTROL_DEADLINE_MISSED.
        """
        return self.ax.error

    def errors(self):
        """Full detail. Only call once something has already tripped."""
        return (self.ax.error, self.ax.motor.error, self.ax.encoder.error,
                self.load.error, self.c.error)

    def clear_errors(self):
        for ax in (self.dev.axis0, self.dev.axis1):
            ax.encoder.error = 0
            ax.motor.error = 0
            ax.controller.error = 0
            ax.error = 0
        time.sleep(0.3)

    def disarm(self):
        try:
            self.ax.requested_state = AXIS_STATE_IDLE
        except Exception:
            pass
        time.sleep(0.3)

    def arm(self):
        """IDLE first, then CLOSED_LOOP. Commanding CLOSED_LOOP from UNDEFINED
        silently fails -- the axis just stays where it was.

        Latched errors are cleared first: an axis refuses to arm while any are
        set, and the previous step in a sweep may legitimately have left one
        (the caller has already reported it)."""
        self.clear_errors()
        if self.ax.current_state != AXIS_STATE_IDLE:
            self.ax.requested_state = AXIS_STATE_IDLE
            for _ in range(20):
                time.sleep(0.1)
                if self.ax.current_state == AXIS_STATE_IDLE:
                    break
            else:
                raise SystemExit(f"axis will not enter IDLE (state "
                                 f"{self.ax.current_state}); clear errors first.")
        # Match the setpoint to where the joint actually is, or arming snaps it
        # to the stale setpoint.
        self.c.input_pos = self.load.pos_estimate
        time.sleep(0.1)
        self.ax.requested_state = AXIS_STATE_CLOSED_LOOP
        for _ in range(20):
            time.sleep(0.1)
            if self.ax.current_state == AXIS_STATE_CLOSED_LOOP:
                return
        errs = "/".join(hex(e) for e in self.errors())
        self.disarm()
        raise SystemExit(f"axis did not reach CLOSED_LOOP (state "
                         f"{self.ax.current_state}, errors {errs}).")

    def step(self, target_deg, seconds, abort_deg, expect_sign=None):
        """Command a step and watch it. Returns a trace dict.

        expect_sign, when given, aborts as soon as the joint has moved more than
        a tolerance in the WRONG direction -- that is the runaway signature.
        """
        start = self.pos_deg()
        motor_start = self.motor_turn()
        self.c.input_pos = target_deg / 360.0
        dt = 1.0 / SAMPLE_HZ
        t0 = time.monotonic()
        trace, peak_i, aborted = [], 0.0, None
        k = 0
        while time.monotonic() - t0 < seconds:
            t = time.monotonic() - t0
            pos = self.pos_deg()          # the ONE read every pass
            trace.append((t, pos))
            k += 1
            moved = pos - start
            # The position-based aborts are the safety-critical ones, so they run
            # every pass; the reads they would cost do not.
            if expect_sign is not None and moved * expect_sign < -1.0:
                aborted = (f"moving the WRONG WAY: commanded {target_deg - start:+.1f} deg, "
                           f"joint went {moved:+.1f} deg")
                break
            if abs(pos - target_deg) > abort_deg:
                aborted = (f"strayed {abs(pos - target_deg):.1f} deg from the target "
                           f"(limit {abort_deg:.0f})")
                break
            if k % SPARSE_EVERY == 0:
                peak_i = max(peak_i, abs(self.ax.motor.current_control.Iq_measured))
            elif k % SPARSE_EVERY == 5 and self.tripped():
                aborted = "axis raised an error: " + "/".join(hex(e) for e in self.errors())
                break
            time.sleep(dt)
        # Motor travel from the endpoints only: enough to tell a decoupled joint
        # from a soft gain, and it costs one read instead of one per pass.
        motor_moved = abs(self.motor_turn() - motor_start)
        if aborted:
            self.disarm()
            return {"aborted": aborted, "trace": trace, "peak_i": peak_i,
                    "start": start, "target": target_deg,
                    "motor_moved": motor_moved}

        # settle time: first sample inside tolerance that stays inside
        settle = None
        for k, (t, pos) in enumerate(trace):
            if all(abs(p - target_deg) <= SETTLE_TOL_DEG for _, p in trace[k:]):
                settle = t
                break
        tail = [p for t, p in trace if t >= trace[-1][0] - RESIDUAL_WINDOW]
        residual = sum(p - target_deg for p in tail) / len(tail) if tail else float("nan")
        travel = target_deg - start
        if abs(travel) > 1e-6:
            excursions = [(p - start) / travel for _, p in trace]
            overshoot = 100.0 * (max(excursions) - 1.0)
        else:
            overshoot = 0.0
        return {"aborted": None, "trace": trace, "peak_i": peak_i, "start": start,
                "target": target_deg, "settle": settle, "residual": residual,
                "overshoot": max(overshoot, 0.0), "motor_moved": motor_moved}


def main():
    args = parse_args()
    label = args.position or "joint"

    dev = odrive.find_any(serial_number=args.serial_number.upper()
                          if args.serial_number else None, timeout=30)
    j = Joint(dev)
    print(f"board {format(dev.serial_number, 'x')}  "
          f"fw {dev.fw_version_major}.{dev.fw_version_minor}.{dev.fw_version_revision}  "
          f"node {dev.axis0.config.can_node_id}  vbus {dev.vbus_voltage:.1f} V")

    if dev.axis1.encoder.config.mode == 0:
        raise SystemExit("axis1 joint encoder is disabled -- split feedback needs it.")
    for ax in (dev.axis0, dev.axis1):
        ax.encoder.error = 0
        ax.motor.error = 0
        ax.controller.error = 0
        ax.error = 0
    time.sleep(0.5)

    # Re-seed the linear accumulator before the loop closes on it: pos_estimate
    # is seeded once at startup and can sit a whole turn out after dropped
    # samples, which would put the setpoint 360 deg from the real joint angle.
    dev.axis1.encoder.config.zero_offset = dev.axis1.encoder.config.zero_offset
    time.sleep(0.3)

    c = dev.axis0.controller.config
    c.load_encoder_axis = 1
    c.vel_encoder_axis = 0
    c.control_mode = CONTROL_MODE_POSITION
    c.input_mode = INPUT_MODE_PASSTHROUGH
    c.vel_limit = args.vel_limit
    if args.vel_gain is not None:
        c.vel_gain = args.vel_gain
    if args.vel_integrator_gain is not None:
        c.vel_integrator_gain = args.vel_integrator_gain
    saved_current_lim = dev.axis0.motor.config.current_lim
    if args.current_lim:
        dev.axis0.motor.config.current_lim = args.current_lim
    time.sleep(0.3)
    print(f"split feedback: load=axis1 (joint MT6701), vel=axis0 (motor)  "
          f"vel_limit {c.vel_limit:.2f} t/s  vel_gain {c.vel_gain:.4f}  "
          f"vel_i {c.vel_integrator_gain:.4f}  current_lim "
          f"{dev.axis0.motor.config.current_lim:.1f} A")
    print(f"joint now at {j.pos_deg():+.2f} deg\n")

    results = []
    best = None
    try:
        # ---- sign check ----------------------------------------------------
        if args.position_direction is not None:
            sign = args.position_direction
            print(f"position_direction forced to {sign:+d} (sign check skipped)")
        else:
            sign = None
            attempts = []
            sign_gain = args.sign_gains[0]
            for sign_gain in args.sign_gains:
                if sign is not None:
                    break
                for candidate in (1, -1):
                    print(f"--- sign check: position_direction {candidate:+d}, "
                          f"pos_gain {sign_gain:g}, {args.sign_step_deg:+.1f} deg step")
                    c.position_direction = candidate
                    c.pos_gain = sign_gain
                    time.sleep(0.2)
                    j.arm()
                    start = j.pos_deg()
                    target = start + args.sign_step_deg
                    r = j.step(target, 3.0, args.abort_deg, expect_sign=1.0)
                    if r["aborted"]:
                        print(f"    ABORTED -- {r['aborted']}")
                        continue
                    moved = r["trace"][-1][1] - start
                    print(f"    joint {start:+.2f} -> {r['trace'][-1][1]:+.2f} deg "
                          f"(moved {moved:+.2f}, wanted {args.sign_step_deg:+.2f}), "
                          f"motor moved {r['motor_moved']:.4f} turn, "
                          f"peak {r['peak_i']:.2f} A")
                    attempts.append((candidate, r, moved))
                    if (moved * args.sign_step_deg > 0
                            and abs(moved) > 0.3 * abs(args.sign_step_deg)):
                        sign = candidate
                        print(f"    OK: position_direction {sign:+d} tracks the command.")
                        # put it back where it started
                        j.step(start, 2.5, args.abort_deg)
                        break
                    print("    did not track; trying the other sign.")
                    j.disarm()
            if sign is None:
                j.disarm()
                # Which failure this is, is decided by the MOTOR trace, not the
                # joint trace -- they look identical from the joint side.
                motor_max = max((r["motor_moved"] for _, r, _ in attempts), default=0.0)
                i_max = max((r["peak_i"] for _, r, _ in attempts), default=0.0)
                lim = dev.axis0.motor.config.current_lim
                print("\nNeither position_direction tracked the step. Diagnosis from the "
                      "motor trace:")
                if motor_max > 0.05:
                    print(f"  the MOTOR turned {motor_max:.3f} turn but the joint did not "
                          "follow.")
                    print("  -> DECOUPLED: broken coupling, stripped gear or slipping "
                          "output. Not a")
                    print("     tuning problem; no gain will fix it.")
                elif i_max > 0.8 * lim:
                    print(f"  the motor did not move ({motor_max:.4f} turn) while pulling "
                          f"{i_max:.1f} A of a {lim:.0f} A limit.")
                    print("  -> SEIZED or jammed: it is pushing hard and going nowhere. "
                          "Turn the")
                    print("     joint by hand with the axis idle before commanding it again.")
                else:
                    print(f"  the motor barely moved ({motor_max:.4f} turn) and only drew "
                          f"{i_max:.1f} A of a {lim:.0f} A limit --")
                    print("  it never really pushed.")
                    print(f"  -> pos_gain is TOO SOFT for this joint, up to the "
                          f"{max(args.sign_gains):g} tried. The gain is in")
                    print("     JOINT-side units, so it scales with the gearbox ratio: a "
                          "high-ratio")
                    print("     joint needs hundreds. Re-run with a larger --sign-gain "
                          "(and/or a")
                    print("     bigger --sign-step-deg) before concluding anything "
                          "mechanical.")
                return 1
        c.position_direction = sign

        # ---- gain sweep ----------------------------------------------------
        print(f"\n--- gain sweep, {args.step_deg:+.1f} deg steps "
              f"(settle tol {SETTLE_TOL_DEG} deg)\n")
        print(f"  {'pos_gain':>9}  {'settle':>8}  {'residual':>9}  "
              f"{'overshoot':>9}  {'peak I':>7}")
        if j.ax.current_state != AXIS_STATE_CLOSED_LOOP:
            j.arm()
        for gain in args.gains:
            c.pos_gain = gain
            time.sleep(0.2)
            if j.ax.current_state != AXIS_STATE_CLOSED_LOOP:
                j.arm()
            home = j.pos_deg()
            r = j.step(home + args.step_deg, args.settle_seconds, args.abort_deg)
            if r["aborted"]:
                print(f"  {gain:9.0f}  ABORTED -- {r['aborted']}")
                break
            settle_txt = f"{r['settle']:.2f} s" if r["settle"] is not None else "  never"
            print(f"  {gain:9.0f}  {settle_txt:>8}  {r['residual']:+8.2f} deg  "
                  f"{r['overshoot']:8.0f}%  {r['peak_i']:6.2f} A")
            results.append((gain, r))
            # return to the starting pose before the next candidate
            back = j.step(home, args.settle_seconds, args.abort_deg)
            if back["aborted"]:
                print(f"           (return ABORTED -- {back['aborted']})")
                break
            # Only bail out on ringing once something has actually WORKED. A
            # sticking joint overshoots at every gain, and stopping on the first
            # overshoot would end the sweep before any candidate settled.
            if r["overshoot"] > 40.0 and any(
                    x["settle"] is not None for _, x in results[:-1]):
                print("           overshoot is climbing past a gain that already "
                      "settled -- stopping.")
                break
    except KeyboardInterrupt:
        print("\ninterrupted -- disarming.")
    finally:
        j.disarm()
        if args.current_lim:
            dev.axis0.motor.config.current_lim = saved_current_lim
        print(f"\naxis IDLE (state {dev.axis0.current_state}), "
              f"current_lim {dev.axis0.motor.config.current_lim:.1f} A")

    # ---- pick ---------------------------------------------------------------
    settled = [(g, r) for g, r in results
               if r["settle"] is not None and r["overshoot"] <= 25.0]
    if settled:
        best = min(settled, key=lambda gr: (gr[1]["settle"], abs(gr[1]["residual"])))
        print(f"\nBEST: pos_gain {best[0]:.0f} -- settled in {best[1]['settle']:.2f} s, "
              f"residual {best[1]['residual']:+.2f} deg, "
              f"overshoot {best[1]['overshoot']:.0f}%")
        print(f"      position_direction {sign:+d}, load_encoder_axis 1, vel_encoder_axis 0")
    elif results:
        print("\nNo candidate settled inside tolerance without ringing. Widen the "
              "gain list\nor check the joint mechanically -- backlash sets a floor "
              "on the residual.")
    if best and args.save:
        c.pos_gain = best[0]
        time.sleep(0.2)
        try:
            dev.save_configuration()
        except Exception as exc:
            print(f"  save_configuration raised {type(exc).__name__} "
                  "(expected -- USB transport resets)")
        print(f"SAVED pos_gain {best[0]:.0f} to flash.")
    elif best:
        print(f"\nNothing was saved. To keep it:  --save  "
              f"(or set pos_gain {best[0]:.0f} by hand)")
    return 0


if __name__ == "__main__":
    sys.exit(main())
