#!/usr/bin/env python3
"""Drive one ODrive joint to a target angle over CAN and HOLD it until told to stop.

Unlike can_goto.py (which releases to IDLE), this keeps the joint in closed loop
indefinitely -- for a geared leg that means CONTINUOUS current against gravity,
which is the duty that thermally destroyed motor #10 (~30 W was enough). Most of
these boards have no motor thermistor, so nothing protects the winding. Hence:

  * a HARD max-hold timeout (--max-hold, default 600 s) that always fires
  * a stop file (--stop-file) so a supervisor can end the hold cleanly
  * continuous Iq telemetry with an estimated copper-loss readout
  * abort -> IDLE on any axis fault or on |Iq| over the cap
  * IDLE on EVERY exit path, including KeyboardInterrupt / exception

Telemetry is CAN-only on purpose: polling the board over USB during closed loop
starves the real-time control loop and trips CONTROL_DEADLINE_MISSED.
"""
import argparse
import os
import struct
import time

from can_goto import (Bridge, STATE_NAMES, CMD_SET_AXIS_STATE, CMD_SET_INPUT_POS,
                      CMD_SET_CONTROLLER_MODES, CMD_CLEAR_ERRORS,
                      AXIS_STATE_IDLE, AXIS_STATE_CLOSED_LOOP,
                      CONTROL_MODE_POSITION, INPUT_MODE_PASSTHROUGH)


def release(br, why):
    br.send(CMD_SET_AXIS_STATE, struct.pack("<i", AXIS_STATE_IDLE))
    print("  -> IDLE (%s)" % why, flush=True)


def hold(port, node, target_turn, rate_deg_s, iq_cap, max_hold, stop_file,
         phase_r, report_dt):
    br = Bridge(port, node)
    try:
        err, state = br.hb(timeout=3.0)
        if err is None:
            print("  NO HEARTBEAT -> abort", flush=True)
            return False
        print("  heartbeat: err=0x%X state=%s" % (err, STATE_NAMES.get(state, "?")), flush=True)
        if err:
            print("  axis error set -> abort", flush=True)
            return False

        pos, _ = br.get_pos()
        if pos is None:
            print("  no encoder estimate -> abort", flush=True)
            return False
        print("  start %.2f deg -> target %.2f deg" % (pos * 360, target_turn * 360), flush=True)

        br.send(CMD_CLEAR_ERRORS)
        time.sleep(0.2)
        br.send(CMD_SET_CONTROLLER_MODES,
                struct.pack("<ii", CONTROL_MODE_POSITION, INPUT_MODE_PASSTHROUGH))
        br.send(CMD_SET_INPUT_POS, struct.pack("<fhh", pos, 0, 0))
        time.sleep(0.15)
        br.send(CMD_SET_AXIS_STATE, struct.pack("<i", AXIS_STATE_CLOSED_LOOP))

        entered = False
        end = time.time() + 3.0
        while time.time() < end:
            err, state = br.hb(timeout=0.5)
            if err:
                release(br, "fault 0x%X on entry" % err)
                return False
            if state == AXIS_STATE_CLOSED_LOOP:
                entered = True
                break
        if not entered:
            release(br, "did not reach CLOSED_LOOP")
            return False

        # Ramp to target.
        step_dt = 0.05
        step = (rate_deg_s / 360.0) * step_dt
        cmd = pos
        while True:
            cmd = target_turn if abs(target_turn - cmd) <= step else \
                cmd + (step if target_turn > cmd else -step)
            br.send(CMD_SET_INPUT_POS, struct.pack("<fhh", cmd, 0, 0))
            time.sleep(step_dt)
            err, _ = br.hb(timeout=0.06)
            if err:
                release(br, "fault 0x%X while ramping" % err)
                return False
            iq = br.get_iq(timeout=0.08)
            if iq is not None and abs(iq) > iq_cap:
                release(br, "Iq %.1f A over cap %.1f A while ramping" % (iq, iq_cap))
                return False
            if cmd == target_turn:
                break

        print("  HOLDING. stop with:  touch %s" % stop_file, flush=True)
        print("  hard timeout %.0f s. Iq/loss reported every %.0f s." % (max_hold, report_dt),
              flush=True)
        t0 = time.time()
        nxt = 0.0
        peak = 0.0
        while True:
            now = time.time() - t0
            if stop_file and os.path.exists(stop_file):
                release(br, "stop file after %.0f s" % now)
                return True
            if now >= max_hold:
                release(br, "HARD TIMEOUT %.0f s reached" % max_hold)
                return True
            err, state = br.hb(timeout=0.2)
            if err:
                release(br, "fault 0x%X during hold" % err)
                return False
            if state is not None and state != AXIS_STATE_CLOSED_LOOP:
                print("  left CLOSED_LOOP (state=%s)" % STATE_NAMES.get(state, "?"), flush=True)
                release(br, "unexpected state")
                return False
            iq = br.get_iq(timeout=0.2)
            if iq is not None:
                peak = max(peak, abs(iq))
                if abs(iq) > iq_cap:
                    release(br, "Iq %.1f A over cap %.1f A during hold" % (iq, iq_cap))
                    return False
            if now >= nxt:
                p, _ = br.get_pos(timeout=0.3)
                # FOC copper loss ~ 1.5 * Iq^2 * R_phase
                loss = 1.5 * (iq or 0.0) ** 2 * phase_r
                print("  t=%5.0fs  pos=%6.2f deg  Iq=%5.2f A  ~%4.0f W copper  (peak %.1f A)"
                      % (now, (p or 0) * 360, iq or float('nan'), loss, peak), flush=True)
                nxt = now + report_dt
            time.sleep(0.05)
    except BaseException as e:
        release(br, "exiting on %s" % type(e).__name__)
        raise
    finally:
        br.close()


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--port", required=True)
    p.add_argument("--node", type=int, required=True)
    p.add_argument("--target", type=float, required=True, help="target joint pos in turns")
    p.add_argument("--rate", type=float, default=3.0, help="ramp rate deg/s")
    p.add_argument("--iq-cap", type=float, default=24.0)
    p.add_argument("--max-hold", type=float, default=600.0, help="hard timeout seconds")
    p.add_argument("--stop-file", default=None)
    p.add_argument("--phase-r", type=float, default=0.24, help="phase resistance for loss estimate")
    p.add_argument("--report", type=float, default=5.0, help="telemetry interval s")
    a = p.parse_args()
    ok = hold(a.port, a.node, a.target, a.rate, a.iq_cap, a.max_hold,
              a.stop_file, a.phase_r, a.report)
    raise SystemExit(0 if ok else 1)


if __name__ == "__main__":
    main()
