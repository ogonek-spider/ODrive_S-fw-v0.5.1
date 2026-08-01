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

## Gearbox ratio by position (production build)

Same motor everywhere, different boxes:

| Position | Gearbox | Joint-side `pos_gain` |
|:---------|:--------|:---------------------:|
| 1 coxa   | 1:6         | ~140 (derived)  |
| 2 femur  | 6×6×3 = **108:1** | **2500** (step-tested, leg 6) |
| 3 knee   | 1:6         | ~140 (derived)  |

With split feedback (`load_encoder_axis = 1`) the position error is in **output**
turns, so `pos_gain` is a joint-side gain and **scales with the ratio**
(≈ `23 × ratio`). Never carry a femur gain onto a coxa/knee — 2500 on a 1:6
joint is ~18× too stiff. These values live in `JOINT_TUNING` in
[robot_joint_setup.py](robot_joint_setup.py), which writes and verifies them.

⚠️ **Leg 1 is an early prototype and does not follow this table** — its coxa is
1:6, its femur **1:18**, and its knee measures 5.6–7.4:1 varying with angle.

## Assignment

| Leg | Motor | Location    | `can_node_id` | Motor # | Serial       |
|:---:|:-----:|:------------|:-------------:|:-------:|:-------------|
| 1   | 1     | top         | **11**        | 12      | 366f36533335 |
| 1   | 2     | middle      | **12**        | 4       | 3680366e3335 |
| 1   | 3     | bottom/knee | **13**        | 1       | 367b36793335 |
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
| 6   | 2     | middle      | **62**        | 9       | 318136883335 |
| 6   | 3     | bottom/knee | **63**        | 13      | 367036483335 |

## Setting it on a board (single-axis → use axis0)

```python
odrv0.axis0.config.can_node_id = 13          # leg 1, bottom/knee
odrv0.axis0.config.can_node_id_extended = False
odrv0.save_configuration()
```

Bench board **serial `3482345a3034` (physical motor #8) → leg 1, bottom/knee → `can_node_id = 13`** (applied + saved 2026-07-13). This **replaces** physical motor #2 (`367836893335`) in this slot — motor #2's new electric box has the unresolved thermistor-short + commutation faults documented below, so motor #8 (which is healthy and geared) takes leg 1 knee. Motor #8 was previously node 23 (bench testing); that slot is now motor #3.

Bench board **serial `366f36533335` (physical motor #12) → leg 1, top = COXA → `can_node_id = 11`** (applied + saved 2026-07-26). Mounted in robot with **1:6 gearbox** (NOT the 34:1 used on knees; verify per-joint ratio). Motor **not** free to spin → no motor recal done; commutation from bench setup (offset 8399, Kt 0.260) — **verified working through the gearbox** (2026-07-26): smooth current, no faults, drives both directions. **Sign: +motor velocity → +joint** (toward max); `position_direction = +1`. + is the higher-friction direction (needs more current to break friction — fine within 15 A). Joint-side MT6701 on `axis1` configured (mode 261, cpr 16384, CS6, `pre_calibrated`), split feedback `load_encoder_axis=1`/`vel_encoder_axis=0`. Joint encoder healthy (CRC bad ~0.003%, no slip over a full hand-sweep). **Joint coordinate:** `direction=1`, `zero_offset=1660` (raw at min), away-from-min = positive. **Range:** min = 0° (raw 1660); physical hard max ≈ **+154°** (raw ~8677) the short way. NB: the coxa can also swing the *other* way past min (explored to −214°), so total mobility > 180° — keep operation on the one 0..+ arc. **OPERATING MAX CAPPED at +140°** (raw ~8032) — host must never command above this: the abs-encoder linear `pos_estimate` boots **1 turn low** above raw **8192** (= +143.6°); ≤140° boots clean everywhere (`pos_cpr` circular is always correct; only linear `pos_estimate` wraps). See [[abs-encoder-boot-wrap-halfturn-2026-07-26]]. **TODO:** `pos_gain`/tune when first driven closed-loop position; measured gearbox ratio came out garbage (~3×) from coarse endpoint sampling — do a clean synchronized-sweep ratio check to confirm 1:6.

Bench board **serial `3680366e3335` (physical motor #4) → leg 1, middle = FEMUR → `can_node_id = 12`** (applied + saved 2026-07-26). Flashed **fw 0.5.4** (position-limit / min-max endstop patch) — first board to carry it; base config restored from backup (Kt 0.2435, offset 19181, mode 257/CS7). Mounted with **1:18 compound gearbox** (1:6 + 1:3 planetary; ratio re-confirmed on-robot ≈18.3:1 from motor-turns/output-turns). Motor **not** free to spin → no motor recal; commutation carried from bench. **Verified working through the gearbox** (2026-07-26): 3× full-range 0↔155° closed-loop cycles, ±0.8° tracking, no faults, motor turns repeatable (no slip). **Sign: +motor velocity → +joint (leg UP, away from ground)**; `position_direction = +1`. Joint-side MT6701 on `axis1` (mode 261, cpr 16384, CS6, `pre_calibrated`), split feedback `load_encoder_axis=1`/`vel_encoder_axis=0`. Joint encoder healthy (CRC-miss ~2.5% scattered under PWM EMI, no consecutive-miss fault, no slip). **Joint coordinate:** `direction=1`, `zero_offset=7887` (raw at ground) — **min = 0° = leg lowered to ground**; physical hard max ≈ **+165°** (raw ~15386). **Software endstops ENABLED:** `min_position=0`, `max_position=+160°` (`enable_position_limit=True`); clamp verified (commanded +200° held at ~157°). Tuned `pos_gain=140`, `vel_gain=0.2`, `vel_integrator_gain=0.8` (softer stalled on small up-steps against gravity+planetary stiction). **Boot-wrap:** ground rest (raw 7887) is below the raw-8192 boundary so it **boots clean at 0**; only a power-up while held raised (raw>8192) would boot 1 turn low — a dangling leg rests at ground, so this is safe. See [[abs-encoder-boot-wrap-halfturn-2026-07-26]].

Board **serial `367b36793335` (physical motor #1) → leg 1, bottom/knee = KNEE →
`can_node_id = 13`** (applied + saved 2026-07-27). **This replaces motor #8
(`3482345a3034`) in this slot** — the row above is updated; reassign #8 elsewhere.
Flashed **fw 0.5.4** (0.5.1 before); config restored intact (Kt 0.253, offset
17926, R 0.240 Ω, L 0.595 mH, pp15, AS5047P mode 257 / CS7, brake 2.0 Ω armed;
**no motor thermistor**). Joint-side **MT6701 on `axis1`
configured**: mode 261, cpr 16384, CS6, `pre_calibrated=True` — verified healthy
(**0 bad CRC in 440k samples**, 0-count spread at rest, errors 0 across a reboot).

**First motion + joint coordinates, 2026-07-28.** Motor armed cleanly on the
robot (the bench commutation offset is correct): held position at ±0.1 A, no
faults. **`+` motor = `+` joint = leg UP** (shin folds), verified with a 4° test
move.

- **Joint zero captured at the LOWER travel point:** `axis1.encoder.config`
  `direction = +1`, **`zero_offset = 6186`** — saved and verified across a
  reconnect. Bottom = 0°, up positive. This also resolves the old ±180°-seam
  warning: the whole travel now sits on one side of the seam with ~18° to spare.
- **Travel:** 0° (bottom) → +82.3° (pose the leg rests in) → **upper physical
  stop ≈ +162°, NOT yet measured** (user's estimate of +80° above the rest pose).
- **No lower hard stop** within 43° of powered travel — the bottom was set by
  hand. Descending has to be PUSHED (gravity pulls the shin back toward hanging)
  until it goes **over-centre past horizontal**, after which gravity takes over
  and the descent runs away in stick-slip bursts (a 30° step overshot to 65.7°).
- **Split feedback configured + saved:** `load_encoder_axis = 1`,
  `vel_encoder_axis = 0`, `position_direction = +1`, **`pos_gain = 130`**.
  Verified that CAN `Get_Encoder_Estimates` now reports the **joint** angle
  (94.2° over CAN == 94.2° over USB) → **`can_goto.py --target` for node 13 is in
  JOINT turns from here on, not motor turns.**
- **`min_position = 0°`; `max_position = 140°` is PROVISIONAL and
  `enable_position_limit = False`** — do not enable until the top is measured.

🔴 **Gearbox is NOT a constant 1:6 — the ratio VARIES with joint angle:**
measured **7.39 / 7.38** (bottom +1…+39°, taken in opposite directions, agreeing
to 0.1% — so not backlash), **6.30** (mid +38…+89°), **5.64** (top +76…+86°).
Consistent with a linkage drive of changing lever arm. **Never convert motor
angle to joint angle with a fixed factor** — that is what `load_encoder_axis = 1`
is for. `pos_gain = 130` (= motor-side 20 × ratio) keeps the effective motor-side
gain between 17.6 and 23 across the whole travel.

🔴 **`pos_estimate` can silently lose a WHOLE TURN.** It is a linear accumulator,
seeded once from `wrap_pm(count_in_cpr - zero_offset, cpr/2)` and thereafter only
integrating deltas. During this session's stick-slip falls it drifted to
**−265.78° while the true angle was +94.22°** (exactly −360°); `count_in_cpr`
stayed correct throughout. **Re-sync without rebooting by writing `zero_offset`
back onto itself** — its setter calls `Encoder::reset_user_position()`:

```python
e = odrv0.axis1.encoder
e.config.zero_offset = e.config.zero_offset   # re-seeds from count_in_cpr
```

**Always verify `pos_estimate == wrap_pm(count_in_cpr - zero_offset)` before
enabling split feedback or endstops** — otherwise the controller reads the joint
a full turn away, pins the setpoint at `min_position`, and tries to drive 360° of
joint travel to "correct" it.

**STILL TODO on this joint:** measure the upper physical stop, set
`max_position` with seam margin and enable `enable_position_limit`, tune
`pos_gain` under real load, re-verify across a power cycle.

**CAN hazard fixed on every leg-1 board:** `axis1` defaults to
`can_node_id = 1` with a 100 ms heartbeat, and `ODriveCAN::send_heartbeat`
transmits whenever the rate is > 0 without checking whether the axis is used —
so every board on the bus would emit heartbeats claiming node 1. All three leg-1
boards now have `axis1.config.can_heartbeat_rate_ms = 0`. **Do this on every new
board, in the same session as `can_node_id`.**

**Femur update 2026-07-27 — tibia fitted (+4 kg), leg now stands on soft ground.**
The 2026-07-26 numbers above were measured with a *lighter* leg and no tibia; two
of them are superseded:

- **`current_lim` 15 → 25 A (saved).** With the tibia the joint could not move at
  all at 15 A: an up-move stalled at ~14 A after 2.5°. It needed **16.6 A** to
  break away. Note this board has **no motor thermistor** — nothing protects the
  winding.
- **`min_position` 0° → +58.21° (saved).** "0° = leg lowered to ground" is now
  **unreachable**: the foot bottoms out on the ground at ~58.2°, and released
  poses sink back to it. The ground is **soft**, so this rest angle drifts.
- **`max_position` is still 160° and is NOT re-derived** — treat it as fiction
  until measured with the tibia fitted.

**Holding cost measured** (instrumented hold, `vel_integrator_torque` logged):
holding **+78.6°** settles at **3.20 Nm motor / 13.1 A ≈ 40 Nm at the joint**,
reached after ~38 s of integrator fill (it converges — it is not runaway windup).
That is **~62 W copper**, vs the ~30 W that thermally destroyed motor #10. The
first ~10 s of any hold stick-slips ±3.5° with 14.6 A peaks. Sustained holding
near horizontal needs mechanical help (counterbalance / brake) or a 200 mm
aluminium radiator; it cannot be tuned away.

Board **serial `318136883335` (physical motor #9) → leg 6, middle = FEMUR →
`can_node_id = 62`** (fw **0.5.6**). Position loop tuned over USB 2026-08-01.

🔴 **Gearbox is 6 × 6 × 3 = 108:1 — NOT the 1:18 that leg 1's femur uses.**
Measured independently before the build was confirmed: a monotonic motor ramp of
**+1.4 motor turns produced +4.09° of joint travel**, instantaneous ratio stable
at **113–146:1** across the whole sweep, second-half fit **112.9:1**, sign **+1**
(+motor → +joint). Do not assume femurs share a ratio between legs.

- **Split feedback SAVED:** `load_encoder_axis = 1`, `vel_encoder_axis = 0`,
  `position_direction = +1`, **`pos_gain = 2500`**, `vel_gain = 0.167`,
  `vel_integrator_gain = 0.333` (the last two left at defaults). Verified across
  the `save_configuration()` reboot. Since `load_encoder_axis = 1`, CAN
  `Get_Encoder_Estimates` on node 62 reports the **joint** angle → `can_goto.py
  --target` is in JOINT turns for this node.
- **Why 2500:** with `load_encoder_axis = 1` the gain is joint-side, so it must
  carry the ratio — `2500 / 108 ≈ 23`, i.e. the usual motor-side ~20. Sweep with
  ±5° joint steps: gain 300 settled in 3.3 s with a 0.32° residual, 700 and 1500
  were also sluggish (3.7–3.9 s, ~0.5° off), **2500 settled in 0.27 s with
  −0.04° residual**, and 5000 bought nothing while growing overshoot. The
  gravity-assisted downward step consistently overshoots more (+0.67°) and draws
  ~8 A vs ~5 A upward.
- **Joint-side MT6701 on `axis1` is healthy:** mode 261, cpr 16384, CS 6,
  **0 bad CRC in 96 218 samples**, 12-count spread at rest, and
  `pos_estimate == wrap_pm(count_in_cpr − zero_offset)` exactly (no lost turn).
- **Backlash ≈ 1.5° at the joint.** After a ramp out and back the motor returned
  exactly to its start while the joint stayed **+1.56°** away. That is the
  accuracy floor here — the ±0.5° step residuals sit inside it, so do not chase
  them with gain.

**Holding cost — this is the argument for the 108:1 box.** A 50 s static hold at
18.4° settled at **Iq ≈ 2.4–2.9 A ≈ 0.61 Nm motor ≈ 37–46 Nm at the joint**
(η 0.57–0.70), for **~2–3 W of copper**, with drift under 0.10°. Compare leg 1's
1:18 femur holding a comparable **40 Nm at 13.1 A ≈ 62 W** — same joint torque
for **~4.5× less current and ~20× less heat**, because motor torque scales as
1/(N·η) and copper loss as I². Against a project whose documented killer is
thermal death during continuous gravity holds (motor #10 at ~30 W, the LA8308
burnt winding), **108:1 is the right choice for a femur**; it also holds pose at
near-zero current instead of needing a counterbalance or brake. The costs are the
1.5° backlash, ~6× less joint speed (`vel_limit = 10` motor t/s → only 33°/s at
the joint), and shock loads landing on gear teeth instead of being absorbed.
NB the hold current was still creeping upward at 50 s (0.8 → 2.9 A as the
integrator filled), so treat ~3 A as a lower bound, and note the pose at 18.4° is
not necessarily the worst-case gravity moment.

**STILL TODO on this joint:** joint `zero_offset`/`direction` are still `0`/`+1`
(the working pose reads 18.4°, an arbitrary origin) — capture a real zero;
measure the travel limits and set `min_position`/`max_position` +
`enable_position_limit` (currently **disabled**, so nothing stops this joint);
re-verify the tuning over CAN rather than USB.

⚠️ **`axis1.config.can_heartbeat_rate_ms = 0` is already set** (the node-1 mute).
⚠️ Motor thermistor is enabled and its filter settles slowly after a reboot —
it read −3 °C immediately after the save-reboot, 6 → 16 °C during the hold, and
20 °C once settled. Do not trust a reading taken right after boot.

⚠️ **USB telemetry trips `MOTOR_ERROR_CONTROL_DEADLINE_MISSED` (0x10) on this
board.** Polling ~8 endpoints per 50 ms faulted within seconds and 15 Hz × 2
reads still faulted repeatedly; **8 Hz with 2 reads per sample, and 3 Hz with 1
read, ran clean**. This is the known "runtime telemetry on CAN, not USB" rule —
if you must tune over USB, keep it under ~30 endpoint reads/s.

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

Board **serial `367036483335` (physical motor #13) → leg 6, bottom/knee = KNEE →
`can_node_id = 63`** (applied + saved 2026-07-31). Note `63` is the **6-bit
ceiling** of CAN Simple's node field — this is the last standard-frame id
available, and it is the only slot that cannot be typo'd upward.

Flashed **fw 0.5.3 → 0.5.6** (the live-CAN-configuration patch: `MSG_CONFIG_ACCESS
0x01C` / `MSG_CONFIG_COMMIT 0x01D`). The NVM `config_version` went `0x0004 →
0x0005` across those revisions, so the flash invalidated the saved config as
expected; it was backed up to
`configs/motor13-367036483335-before-0.5.6-flash-2026-07-31.json` and restored +
verified field-by-field afterwards (Kt **0.259**, encoder offset **14241**,
R 0.240 Ω, L 0.604 mH, pp15, AS5047P mode 257 / CS7 / cpr 16384,
`pre_calibrated`, brake 2.0 Ω). **Harmonic compensation restored ON** with the
bench 18 t/s coefficients (cos1 −59.13 / sin1 13.80 / cos2 −1.10 / sin2 −0.54).
**Motor thermistor is wired and enabled** on this board (GPIO4, 5k-divider
coeffs) — reads 27.3 °C stable. That makes #13 one of the few robot joints with
real winding protection.

CAN verified across a power-cycle: `axis0` node **63** @ 100 ms heartbeat,
250 kbaud, `axis1.config.can_heartbeat_rate_ms = 0` (the node-1 mute, applied in
the same USB session per the standing rule), 0 errors, axis0 boots to IDLE.

🔴 **Joint-side MT6701 on `axis1` is NOT responding — not yet usable.**
Configured (mode 261, cpr 16384, CS 6, `pre_calibrated`) **in RAM only, not
saved**, and it reads **100.0 % bad CRC over 120 353 samples** with
`word0 = word1 = 0x0000`, `raw24 = 0x000000`, `count_in_cpr` pinned at 0,
`error 0x80 ABS_SPI_COM_FAIL`. Per the triage rule, 100 % bad CRC = **wiring**
(vs. CRC-OK-but-wandering = missing magnet, CRC-OK + zero spread = healthy).
All-zero is rejected deliberately — an all-zero frame has a self-consistent CRC
of 0, so [encoder.cpp:489](../Firmware/MotorControl/encoder.cpp#L489) guards it
with `(raw24 != 0) && (crc_calc == crc_recv)`.

The board side is proven good: the **`axis0` AS5047P shares the same SCK/MISO**
and reads cleanly on CS 7 during the same session (count 882, spread 5, error 0),
and the control loop really is clocking the MT6701 at ~8 kHz (120 k samples in
15 s). So SPI, MISO and the firmware path all work — nothing is driving MISO
while CS 6 is asserted. Check `DO → MISO`, `CLK → SCK`, `CSN → IO6`,
`VCC → 3.3 V`, `GND → GND`.

⚠️ Note `mt6701_debug_sample()` does **not** force a transfer — it only bumps
`mt6701_debug_request_count_` and records the mode
([encoder.cpp:51](../Firmware/MotorControl/encoder.cpp#L51)); the values read back
are just the latest control-loop sample. Likewise
`mt6701_debug_start_ok_count` / `start_fail_count` are declared but **never
incremented** (always 0) — do not read them as evidence.

**TODO on this joint:** fix MT6701 wiring → re-verify CRC + at-rest spread →
`save_configuration()` → joint `direction`/`zero_offset` → split feedback
(`load_encoder_axis=1`, `vel_encoder_axis=0`, `position_direction`) → measure
travel → endstops → `pos_gain`.
