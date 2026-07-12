# Spider CAN node ID map

Per-joint CAN addressing for the hexapod (6 legs × 3 motors = 18 joints).

## Conventions

- **Position label:** `<leg>-<motor>` (e.g. `2-3` = leg 2, knee).
- **Leg number 1..6:** clockwise, starting at the **front-left** leg.
- **Motor number 1..3:** `1` = top, `2` = middle, `3` = bottom (knee).
- **`can_node_id = leg*10 + motor`** — readable decimal (id `23` = leg 2, motor 3).
- ODrive CAN Simple uses a 6-bit node field → range **0–63**. Leg 6 motor 3 = `63`
  is exactly the ceiling; extending past it needs extended (29-bit) frames.
- `can_node_id = 0` is the erased/default value, so no real joint uses it — an
  un-configured board is instantly visible on the bus.

Rows are keyed by leg position. `Motor #` (physical build/characterization order)
and `Serial` are filled in as boards get mounted and assigned.

## Assignment

| Leg | Motor | Location    | `can_node_id` | Motor # | Serial       |
|:---:|:-----:|:------------|:-------------:|:-------:|:-------------|
| 1   | 1     | top         | **11**        |         |              |
| 1   | 2     | middle      | **12**        |         |              |
| 1   | 3     | bottom/knee | **13**        | 2       | 367836893335 |
| 2   | 1     | top         | **21**        |         |              |
| 2   | 2     | middle      | **22**        |         |              |
| 2   | 3     | bottom/knee | **23**        | 3       | 367c365e3335 |
| 3   | 1     | top         | **31**        |         |              |
| 3   | 2     | middle      | **32**        |         |              |
| 3   | 3     | bottom/knee | **33**        |         |              |
| 4   | 1     | top         | **41**        |         |              |
| 4   | 2     | middle      | **42**        |         |              |
| 4   | 3     | bottom/knee | **43**        |         |              |
| 5   | 1     | top         | **51**        |         |              |
| 5   | 2     | middle      | **52**        |         |              |
| 5   | 3     | bottom/knee | **53**        |         |              |
| 6   | 1     | top         | **61**        |         |              |
| 6   | 2     | middle      | **62**        |         |              |
| 6   | 3     | bottom/knee | **63**        |         |              |

## Setting it on a board (single-axis → use axis0)

```python
odrv0.axis0.config.can_node_id = 13          # leg 1, bottom/knee
odrv0.axis0.config.can_node_id_extended = False
odrv0.save_configuration()
```

Current bench board **serial `367836893335` (physical motor #2) → leg 1, bottom/knee → `can_node_id = 13`** (applied + saved).

Bench board **serial `367c365e3335` (physical motor #3, bare motor) → leg 2, bottom/knee → `can_node_id = 23`** (applied + saved 2026-07-12). This **supersedes** motor #8 (`3482345a3034`) at node 23 — motor #8 was bench testing only; reassign it a new slot before mounting.

## Known problems to revisit

Per-motor issues found on the bench, to fix before final assembly.

### Motor #2 — board `367836893335` (leg 1, knee) — new electric box, 2026-07-12

- **Motor thermistor shorted (GPIO4).** Reads a false **151 °C**; raw ADC on
  GPIO4 ≈ **0.002 V** (pulled to GND) — sense line shorted, or NTC leads
  shorted, in the new box. At room temp this pin should sit ~1–1.6 V. This
  latches `AXIS_ERROR_OVER_TEMP` (0x40000) on axis0 every control cycle
  ([thermistor.cpp:45](../Firmware/MotorControl/thermistor.cpp#L45)), so the
  axis cannot stay in closed loop. **Blocks all spins (USB or CAN).** Fix the
  wiring, or as a stopgap disable `axis0.motor_thermistor.config.enabled`
  (removes over-temp protection — only OK for gentle no-load bench runs).
- **CAN `node_id`** now reads **13** again after the reboot on 2026-07-12 (an
  earlier live read showed 0 — likely a not-fully-loaded config that session).
  Re-verify it survives a cold power cycle.
- **Motor won't commutate in the new box (no rotation under current).** Over
  pure CAN the full control path works — heartbeat, clear-errors, set
  velocity/passthrough mode, CLOSED_LOOP entry, `Set_Input_Vel`, and Iq/encoder
  telemetry all respond. But commanding +2 t/s makes the velocity integrator
  wind current up (Iq_setpoint −0.9 → −2.5 A) while `pos_estimate` stays frozen
  and `vel ≈ 0`, no axis error. Current flows, zero torque → **commutation is
  broken**: almost certainly a stale saved encoder offset and/or a swapped motor
  phase order from rewiring into the new box (the onboard AS5047P offset is only
  valid for the exact phase wiring + rotor-to-sensor mounting it was calibrated
  on). **Also rule out a missing/loose onboard AS5047P magnet on the motor
  shaft** (the external MT6701 is knowingly magnet-less; the *internal* one must
  have its magnet for commutation).

  **Diagnosed 2026-07-12 (magnet IS fitted; not the cause):**
  - Motor phases good: motor cal passes, R≈0.244 Ω, L≈0.5 mH.
  - Motor spins **freely** under open-loop LOCK-IN at 8 A (0.78 turn, Iq≈−10 A,
    no error) — commutation hardware, encoder-under-motion, and the gearbox
    output are all fine; nothing is blocked.
  - **Root cause: commutation offset can't be recalibrated through the 1:36
    gearbox.** ODrive's `run_offset_calibration` drives the scan with a fixed
    **voltage** vector (`voltage_magnitude = calibration_current·R`,
    [encoder.cpp:229](../Firmware/MotorControl/encoder.cpp#L229), with a literal
    `// TODO: Do the scan with current, not voltage!`) that **steps to full
    `calib_scan_omega` instantly** (no speed ramp, unlike lock-in). Through the
    gearbox's reflected inertia/cogging the rotor can't lock onto the already-
    spinning field, so it pole-slips in place: 10 A flows, **zero net rotation**,
    at every scan speed tried (omega 12.5 → 1.0) and current (8→12 A). Errors
    alternate NO_RESPONSE / CPR_POLEPAIRS_MISMATCH; measured scan response
    ~250–840 counts vs ~8738 expected.
  - The original motor #2 commutation cal was done **before** the gearbox was
    attached; that's why it worked then.
  - **Fix options (pick one):** (A) decouple the motor from the gearbox, run
    offset cal, recouple, save; (B) confirm the U/V/W phase leads went back into
    the new box in the **same order** as the original build → the old saved
    offset is still valid, just restore it (no recal); (C) patch the firmware
    offset cal to ramp `calib_scan_omega` from 0 (or use a current-controlled
    scan) so it can calibrate assembled geared joints — a permanent fix for all
    18 joints. Nothing was saved to flash during this diagnosis except the
    thermistor-disable; RAM cal tweaks revert on reboot.

  **Phase-order restore attempt failed (2026-07-12).** User rewired the motor
  phases back toward the original order instead of decoupling. Retested: in
  closed-loop velocity control the loop delivered **7.6 A with zero rotation**
  (0.008 turn), while open-loop lock-in at the same 8 A spins the motor 0.78
  turn. Same current, opposite result ⇒ the closed-loop commutation angle
  (encoder offset 3399) still doesn't match the wiring. Note `Id≈0` in torque
  mode is NOT evidence of good commutation — the current controller always
  drives Id→0 in its assumed frame. Also note the closed-loop tests were
  repeatedly starved of current by clamps unrelated to commutation:
  `enable_current_mode_vel_limit` caps torque-mode current to
  `(vel_limit−vel)·vel_gain` ([controller.cpp:318](../Firmware/MotorControl/controller.cpp#L318)),
  and soft velocity gains under-drive the geared load. Still unresolved: needs a
  correct offset via decouple-and-cal (A) or the firmware scan-ramp patch (C).
  Split-feedback is configured (`load_encoder_axis=1` → the magnet-less MT6701),
  which will also need attention before position control, but velocity/torque
  tests use `vel_encoder_axis=0` (good AS5047P) so it didn't affect these.
