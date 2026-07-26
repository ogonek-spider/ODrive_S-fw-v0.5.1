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
| 1   | 1     | top         | **11**        | 12      | 366f36533335 |
| 1   | 2     | middle      | **12**        | 4       | 3680366e3335 |
| 1   | 3     | bottom/knee | **13**        | 8       | 3482345a3034 |
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

Bench board **serial `3482345a3034` (physical motor #8) → leg 1, bottom/knee → `can_node_id = 13`** (applied + saved 2026-07-13). This **replaces** physical motor #2 (`367836893335`) in this slot — motor #2's new electric box has the unresolved thermistor-short + commutation faults documented below, so motor #8 (which is healthy and geared) takes leg 1 knee. Motor #8 was previously node 23 (bench testing); that slot is now motor #3.

Bench board **serial `366f36533335` (physical motor #12) → leg 1, top = COXA → `can_node_id = 11`** (applied + saved 2026-07-26). Mounted in robot with **1:6 gearbox** (NOT the 34:1 used on knees; verify per-joint ratio). Motor **not** free to spin → no motor recal done; commutation from bench setup (offset 8399, Kt 0.260) — **verified working through the gearbox** (2026-07-26): smooth current, no faults, drives both directions. **Sign: +motor velocity → +joint** (toward max); `position_direction = +1`. + is the higher-friction direction (needs more current to break friction — fine within 15 A). Joint-side MT6701 on `axis1` configured (mode 261, cpr 16384, CS6, `pre_calibrated`), split feedback `load_encoder_axis=1`/`vel_encoder_axis=0`. Joint encoder healthy (CRC bad ~0.003%, no slip over a full hand-sweep). **Joint coordinate:** `direction=1`, `zero_offset=1660` (raw at min), away-from-min = positive. **Range:** min = 0° (raw 1660); physical hard max ≈ **+154°** (raw ~8677) the short way. NB: the coxa can also swing the *other* way past min (explored to −214°), so total mobility > 180° — keep operation on the one 0..+ arc. **OPERATING MAX CAPPED at +140°** (raw ~8032) — host must never command above this: the abs-encoder linear `pos_estimate` boots **1 turn low** above raw **8192** (= +143.6°); ≤140° boots clean everywhere (`pos_cpr` circular is always correct; only linear `pos_estimate` wraps). See [[abs-encoder-boot-wrap-halfturn-2026-07-26]]. **TODO:** `pos_gain`/tune when first driven closed-loop position; measured gearbox ratio came out garbage (~3×) from coarse endpoint sampling — do a clean synchronized-sweep ratio check to confirm 1:6.

Bench board **serial `3680366e3335` (physical motor #4) → leg 1, middle = FEMUR → `can_node_id = 12`** (applied + saved 2026-07-26). Flashed **fw 0.5.4** (position-limit / min-max endstop patch) — first board to carry it; base config restored from backup (Kt 0.2435, offset 19181, mode 257/CS7). Mounted with **1:18 compound gearbox** (1:6 + 1:3 planetary; ratio re-confirmed on-robot ≈18.3:1 from motor-turns/output-turns). Motor **not** free to spin → no motor recal; commutation carried from bench. **Verified working through the gearbox** (2026-07-26): 3× full-range 0↔155° closed-loop cycles, ±0.8° tracking, no faults, motor turns repeatable (no slip). **Sign: +motor velocity → +joint (leg UP, away from ground)**; `position_direction = +1`. Joint-side MT6701 on `axis1` (mode 261, cpr 16384, CS6, `pre_calibrated`), split feedback `load_encoder_axis=1`/`vel_encoder_axis=0`. Joint encoder healthy (CRC-miss ~2.5% scattered under PWM EMI, no consecutive-miss fault, no slip). **Joint coordinate:** `direction=1`, `zero_offset=7887` (raw at ground) — **min = 0° = leg lowered to ground**; physical hard max ≈ **+165°** (raw ~15386). **Software endstops ENABLED:** `min_position=0`, `max_position=+160°` (`enable_position_limit=True`); clamp verified (commanded +200° held at ~157°). Tuned `pos_gain=140`, `vel_gain=0.2`, `vel_integrator_gain=0.8` (softer stalled on small up-steps against gravity+planetary stiction). **Boot-wrap:** ground rest (raw 7887) is below the raw-8192 boundary so it **boots clean at 0**; only a power-up while held raised (raw>8192) would boot 1 turn low — a dangling leg rests at ground, so this is safe. See [[abs-encoder-boot-wrap-halfturn-2026-07-26]].

Bench board **serial `367c365e3335` (physical motor #3, bare motor) → leg 2, bottom/knee → `can_node_id = 23`** (applied + saved 2026-07-12).

Displaced: physical motor #2 (`367836893335`) no longer holds a slot — reassign it once its box faults are fixed.

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
