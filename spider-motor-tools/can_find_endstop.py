#!/usr/bin/env python3
"""Walk one ODrive joint into a mechanical endstop over CAN and report where it is.

Used during per-joint bring-up to find the real travel limits before setting
`encoder.config.min_position` / `max_position`. It creeps the position setpoint
in one direction in small steps and stops as soon as the joint stops following.

Every step the setpoint is RE-ANCHORED to where the joint actually is
(`cmd = pos + step`). That matters on a backdrivable geared leg: descending under
gravity the planetary lets go in stick-slip jerks and the joint overshoots the
setpoint by tens of motor degrees. A fixed ramp treats that overshoot as lag and
calls a false endstop, and, worse, lets the controller wind up against a real
stop. Re-anchoring bounds the push to one step past wherever the joint is, and
makes the endstop signal simply: the joint stopped ADVANCING.

So a stop is declared when actual progress stays under `--min-progress` of a step
for `--stall-steps` steps in a row. |Iq| over the cap is the second, faster trip.

On detection the setpoint is pulled back to where the joint ACTUALLY is (so the
controller stops pressing into the stop), then the axis goes IDLE. The endstop is
reported but nothing is written to the board -- picking zero_offset and the
usable limits is a separate, deliberate step.

Units are whatever this node reports over Get_Encoder_Estimates: joint/output
degrees on a split-feedback joint (load_encoder_axis != this axis), motor-shaft
degrees on a plain one. The tool does not know the gear ratio and does not care.

CAUTION: a foot touching the floor, a cable snagging, or the leg meeting another
limb looks exactly like a mechanical endstop. Confirm visually before treating
the reported angle as the joint's true travel limit.
"""
import argparse
import struct
import time

from can_goto import (Bridge, STATE_NAMES, find_bridge,
                      CMD_SET_AXIS_STATE, CMD_SET_INPUT_POS,
                      CMD_SET_CONTROLLER_MODES, CMD_CLEAR_ERRORS,
                      AXIS_STATE_IDLE, AXIS_STATE_CLOSED_LOOP,
                      CONTROL_MODE_POSITION, INPUT_MODE_PASSTHROUGH)


def ramp_to(br, frm, to, rate_deg_s, iq_cap, step_dt=0.05):
    """Slew the setpoint frm->to at rate_deg_s. Returns (ok, reason)."""
    step = (rate_deg_s / 360.0) * step_dt
    cmd = frm
    while True:
        cmd = to if abs(to - cmd) <= step else cmd + (step if to > cmd else -step)
        br.send(CMD_SET_INPUT_POS, struct.pack("<fhh", cmd, 0, 0))
        time.sleep(step_dt)
        err, _ = br.hb(timeout=0.06)
        if err:
            return False, "fault 0x%X" % err
        iq = br.get_iq(timeout=0.08)
        if iq is not None and abs(iq) > iq_cap:
            return False, "Iq %.2f A over cap" % iq
        if cmd == to:
            return True, ""


def find(port, node, direction, step_deg, rate, iq_cap, max_travel_deg,
         min_progress, stall_steps, settle_s, backoff_deg):
    br = Bridge(port, node)
    sign = 1.0 if direction > 0 else -1.0
    hit = None
    try:
        err, state = br.hb(timeout=3.0)
        if err is None:
            print("  NO HEARTBEAT -> abort", flush=True)
            return None
        print("  heartbeat: err=0x%X state=%s" % (err, STATE_NAMES.get(state, "?")), flush=True)
        if err:
            print("  axis error set -> abort (clear the fault first)", flush=True)
            return None

        start, _ = br.get_pos()
        if start is None:
            print("  no encoder estimate -> abort", flush=True)
            return None
        print("  start %.2f deg, creeping %s in %.1f deg steps (max %.0f deg)"
              % (start * 360, "+" if sign > 0 else "-", step_deg, max_travel_deg), flush=True)

        br.send(CMD_CLEAR_ERRORS)
        time.sleep(0.2)
        br.send(CMD_SET_CONTROLLER_MODES,
                struct.pack("<ii", CONTROL_MODE_POSITION, INPUT_MODE_PASSTHROUGH))
        br.send(CMD_SET_INPUT_POS, struct.pack("<fhh", start, 0, 0))
        time.sleep(0.15)
        br.send(CMD_SET_AXIS_STATE, struct.pack("<i", AXIS_STATE_CLOSED_LOOP))

        entered = False
        end = time.time() + 3.0
        while time.time() < end:
            err, state = br.hb(timeout=0.5)
            if err:
                print("  err=0x%X on entry -> abort" % err, flush=True)
                return None
            if state == AXIS_STATE_CLOSED_LOOP:
                entered = True
                break
        if not entered:
            print("  did not reach CLOSED_LOOP -> abort", flush=True)
            return None

        step_turn = sign * step_deg / 360.0
        cmd = start
        pos = start
        stalled = 0
        reason = "max travel reached"
        while abs(pos - start) * 360 < max_travel_deg:
            prev_pos = pos
            # Re-anchor on the joint, not on the last setpoint.
            target = pos + step_turn
            ok, why = ramp_to(br, cmd, target, rate, iq_cap)
            cmd = target
            if not ok:
                reason = why
                break
            time.sleep(settle_s)
            pos, _ = br.get_pos(timeout=0.3)
            iq = br.get_iq(timeout=0.2)
            if pos is None:
                reason = "lost encoder estimate"
                break
            progress = sign * (pos - prev_pos) * 360
            stalled = stalled + 1 if progress < min_progress * step_deg else 0
            print("    cmd=%8.2f  pos=%8.2f  step progress=%6.2f deg  Iq=%5.2f A%s"
                  % (cmd * 360, pos * 360, progress,
                     iq if iq is not None else float("nan"),
                     "  (stalled %d)" % stalled if stalled else ""), flush=True)
            if stalled >= stall_steps:
                reason = ("no progress for %d steps -> ENDSTOP" % stalled)
                break
            err, _ = br.hb(timeout=0.06)
            if err:
                reason = "fault 0x%X" % err
                break

        pos, _ = br.get_pos(timeout=0.4)
        iq = br.get_iq(timeout=0.3)
        print("  stopped: %s" % reason, flush=True)
        print("  at pos=%.2f deg (%.4f turn), travelled %.2f deg, Iq=%.2f A"
              % (pos * 360, pos, (pos - start) * 360, iq if iq is not None else float("nan")),
              flush=True)
        hit = pos

        # Stop pressing into whatever stopped us: pull the setpoint back to the
        # actual position (plus a small back-off) BEFORE releasing.
        back = pos - sign * backoff_deg / 360.0
        ramp_to(br, cmd, back, max(rate, 5.0), iq_cap + 4.0)
        time.sleep(0.3)
        rest, _ = br.get_pos(timeout=0.4)
        if rest is not None:
            print("  backed off to %.2f deg" % (rest * 360), flush=True)
        return hit
    except BaseException as e:
        print("  exiting on %s" % type(e).__name__, flush=True)
        raise
    finally:
        br.send(CMD_SET_AXIS_STATE, struct.pack("<i", AXIS_STATE_IDLE))
        print("  -> IDLE", flush=True)
        time.sleep(0.2)
        br.close()


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--port", default=None)
    p.add_argument("--node", type=int, required=True)
    p.add_argument("--direction", type=int, required=True, choices=(1, -1),
                   help="+1 or -1, in reported position units")
    p.add_argument("--step", type=float, default=5.0, help="setpoint step per iteration, deg")
    p.add_argument("--rate", type=float, default=10.0, help="slew rate deg/s")
    # Deliberately kept at 6 A while can_jog/can_goto moved to 12: this tool
    # drives INTO a hard mechanical stop on purpose, and the current rise IS the
    # detection signal. A high cap here just means pushing harder into the stop.
    p.add_argument("--iq-cap", type=float, default=6.0)
    p.add_argument("--max-travel", type=float, default=180.0, help="give up after this much, deg")
    p.add_argument("--min-progress", type=float, default=0.25,
                   help="a step counts as stalled below this fraction of --step")
    p.add_argument("--stall-steps", type=int, default=2,
                   help="consecutive stalled steps that declare an endstop")
    p.add_argument("--settle", type=float, default=1.0, help="seconds to converge before measuring")
    p.add_argument("--backoff", type=float, default=2.0, help="deg to retreat from the stop")
    a = p.parse_args()
    port = a.port or find_bridge()
    print("bridge port:", port, flush=True)
    print("node %d: hunting endstop" % a.node, flush=True)
    hit = find(port, a.node, a.direction, a.step, a.rate, a.iq_cap,
               a.max_travel, a.min_progress, a.stall_steps, a.settle, a.backoff)
    raise SystemExit(0 if hit is not None else 1)


if __name__ == "__main__":
    main()
