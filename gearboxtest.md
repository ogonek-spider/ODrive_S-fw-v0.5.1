# Gearbox Sticktion Test — Findings

**Date:** 2026-06-07  
**Gearbox:** 3D-printed PETG, two stages, 1:36 total reduction  
**Configuration:** Full robot arm attached, motor on axis0, load encoder (MT6701) on axis1  
**Test script:** `tools/robot_sticktion_test.py`

---

## Test 1 — Full assembly (arm + gearbox)

**Parameters:**
```
--sweep-degrees 20
--current-limit 8
--max-test-torque 0.3
--sticktion-positions 6
--step-velocity 0.4
--torque-ramp-rate 0.03
```

### Step map — peak current per 1° step

**Forward (0 → 20°):**

| Position | Peak Iq | Note |
|----------|---------|------|
| +0.1° | 2.06A | |
| +0.9° | 2.50A | |
| +2.2° | 0.98A | |
| +2.6° | 2.01A | |
| +3.8° | 1.01A | |
| +4.6° | **2.91A** | elevated |
| +6.3° | 1.52A | |
| +6.6° | **2.96A** | elevated |
| +8.8° | 1.19A | |
| +9.6° | **3.05A** | elevated |
| +12.1° | 2.12A | |
| +12.8° | 1.70A | |
| +13.6° | 1.41A | |
| +17.1° | **3.28A** | elevated |
| +17.5° | 2.58A | |
| +19.0° | 1.02A | |

**Reverse (20 → 0°):**

| Position | Peak Iq | Note |
|----------|---------|------|
| +18.5° | 2.41A | |
| +17.2° | 2.79A | |
| +15.6° | 3.29A | |
| +15.4° | **3.54A** | |
| +13.8° | 2.89A | |
| +13.1° | **3.67A** | |
| +12.1° | 2.05A | |
| +11.3° | **4.59A** | major stuck point |
| +9.4° | 2.48A | |
| **+5.6°** | **4.95A** | worst stuck point |
| +5.2° | **3.62A** | same zone |
| +4.0° | 2.50A | |
| +3.2° | 2.96A | |
| +2.0° | **3.85A** | |
| +1.1° | 1.96A | |

**Top 5 worst steps (both directions):**

| Direction | Position | Peak Iq |
|-----------|----------|---------|
| REV | +5.6° | **4.95A** |
| REV | +11.3° | **4.59A** |
| REV | +2.0° | 3.85A |
| REV | +13.1° | 3.67A |
| REV | +5.2° | 3.62A |

### Sticktion sweep — breakaway current (torque ramp)

| Position | FWD breakaway | REV breakaway | FWD torque cmd | Asymmetry |
|----------|--------------|--------------|----------------|-----------|
| 0.0° | **STUCK** (>0.3Nm) | 1.389A | — | — |
| 2.6° | 2.424A | 1.651A | 0.094Nm | 0.77A |
| 9.1° | 3.026A | ~0A (0.004A) | 0.116Nm | 3.02A |
| 13.3° | 2.366A | ~0A (0.048A) | 0.136Nm | 2.32A |
| 16.4° | 2.795A | ~0A (0.109A) | 0.112Nm | 2.69A |
| 18.5° | 2.007A | ~0A (0.163A) | 0.104Nm | 1.84A |

Mean breakaway (all valid): **1.45A** · Max: **3.03A** · Min: **~0A**

---

## Interpretation

### Gravity asymmetry

REV breakaway is near zero (0.004–0.16A) at most positions because **gravity pulls the arm in the reverse direction**. The torque ramp test effectively measures "extra torque beyond gravity" needed to break away. FWD (against gravity) is the meaningful sticktion number: **2–3A consistently**.

At 0.0° the arm could not break away FWD within 0.3Nm — true hard stuck point at the starting position, typical of PETG gearboxes binding at rest.

### Gear mesh periodicity

The two biggest stuck points in the step map are at **~5.6°** and **~11.3°** (REV direction), roughly 2× apart. This suggests a repeating mesh defect with a period of ~5–6° on the output shaft. With a two-stage 1:36 gearbox, this likely corresponds to a bad tooth or layer artifact on one gear in **stage 1** (the faster-spinning stage — where PETG layer lines are most harmful due to higher RPM and surface stress).

If stage 1 ratio is 6:1: one full revolution of the intermediate shaft = 360°/6 = 60° output travel. A ~5.6° binding period → ~10 gear teeth engaged per intermediate shaft revolution. If the intermediate gear has ~10 teeth, that fits a single-tooth defect. More likely it's a layer-line ridge on a tooth flank that repeats.

### Summary

| Finding | Evidence |
|---------|----------|
| High overall sticktion (PETG, tight tolerances) | FWD breakaway 2–3A uniformly across range |
| Worst binding at 5–6° and 11–12° output positions | Step map peaks 4.6–4.95A REV |
| Binding is periodic → single tooth or layer-line defect | ~5.6° repeat period |
| Stage 1 is the likely culprit | Faster-spinning stage, highest surface stress |
| Arm start position (0°) has worst FWD sticktion | STUCK at 0.3Nm limit |
| REV direction dominated by gravity (not gearbox drag) | ~0A breakaway with arm attached |

---

---

## Test 2 — Gearbox only (arm removed, load encoder still on output shaft)

**Date:** 2026-06-07  
**Config:** Arm disconnected, gearbox free to rotate, load encoder (MT6701) still connected on output shaft  
**Starting position:** -168.4° (output shaft rotated to a new position when arm removed)

### Sticktion sweep result (partial — test aborted at position 3 due to position timeout)

| Position | FWD breakaway | REV breakaway |
|----------|--------------|--------------|
| -168.1° | 0.144A | 0.106A |
| -165.9° | 0.063A | 0.082A |

**Step map:** Unreliable — position controller oscillated without arm load. Without the arm, the gearbox output has very low inertia and almost no damping, causing the shaft to bounce past each 1° target.

### Key finding: **Gearbox sticktion alone is nearly zero**

Breakaway current without arm: **0.06–0.14A**  
Breakaway current with arm: **2.0–3.0A FWD**  

The difference is ~2–3A — almost entirely explained by the arm weight creating a gravity load.  
**The gearbox itself is not the source of the sticktion problem.**

### Revised interpretation

| Finding | Conclusion |
|---------|------------|
| Near-zero breakaway without arm | Gearbox friction is minimal — PETG mesh is OK |
| 2–3A breakaway with arm | Sticktion is gravity-load driven, not gearbox-driven |
| Step map stuck points (Test 1: 4–5A REV) | Likely gravity × poor gains interaction, not real mesh binding |
| True fix | Lower vel_integrator_gain to prevent hunting; or increase current headroom |

### What this means for the robot

The gearbox is probably fine mechanically. The "sticktion" felt in operation is the combination of:
1. Arm weight creating gravity torque at the joint
2. Position controller integrator winding up against sticktion, then lurching when it breaks away
3. The 1:36 gear ratio meaning a small arm torque requires significant motor effort

---

## Next tests planned

| Step | Change | Goal |
|------|--------|------|
| Test 3 | Remove stage 2, test stage 1 alone | Confirm whether any real mesh binding exists |
| Test 4 | Tune position controller gains with arm attached | Fix hunting/oscillation in operation |

---

## Test 3 — Gearbox only, onboard motor encoder

**Date:** 2026-06-08  
**Config:** 1:36 gearbox connected, no arm/load, no load-side encoder  
**Feedback:** Onboard AS5047P on `axis0`; motor position divided by 36  
**Current ceiling:** 4 A phase current, 5 A motor current limit  
**Breakaway threshold:** 0.03° output (~1.1° motor)

| Output position | FWD breakaway | REV breakaway |
|-----------------|---------------|---------------|
| 4.05° | 1.325 A | 0.600 A |
| 5.43° | 2.198 A | 0.453 A |

Mean breakaway across both directions: **1.144 A**.

The requested second position was 9.0°, but position control only soft-settled
at 5.43°. Therefore this is a local measurement around 4–5.4°, not a complete
5° sweep.

Compared with the bare motor test (FWD 0.27–0.31 A, REV 0.43–0.50 A), the
gearbox adds substantial and direction-dependent resistance. Forward
breakaway increased by roughly 1.0–1.9 A, while reverse stayed near the bare
motor range.

The test completed in idle with all axis, motor, encoder, and controller error
fields at zero.

---

## Multi-Rotation Exercise

**Date:** 2026-06-08  
**Feedback:** Onboard AS5047P, output position calculated with 1:36 ratio  
**Load:** Gearbox only, no arm/load  
**Current limit:** 5 A

The gearbox moved approximately **2.15 output rotations forward** across two
guarded attempts.

- At 2 motor turns/s, a bind raised measured phase current to 3.29 A and
  stopped motion before the controller faulted during the release transient.
- At 1.5 motor turns/s, repeated stick-slip was observed. Peak measured phase
  current reached 3.77 A, followed by sudden acceleration to 4.05 motor
  turns/s.
- The second attempt was stopped by the independent software velocity cutoff.
- Final state was idle with zero axis, motor, encoder, and controller errors.
- FET temperature remained below 36°C.

The gearbox can rotate, but it has significant periodic binding and releases
stored torque abruptly. Continuous multi-rotation operation is not considered
safe at the current mechanical condition and controller settings.

A later high-current release attempt confirmed a hard mechanical lock:

- Reverse command with a 15 A motor current limit reached 14.33 A measured
  phase current with effectively zero output movement.
- The command was stopped after 0.5 seconds at high current.
- Final state was idle with zero errors; FET temperature was 36.8°C.

The lock is not caused by the earlier 5 A current ceiling. The gearbox needs
mechanical inspection before it can complete continuous rotations.

---

## Pre-Load Baseline After Gearbox Release

**Date:** 2026-06-09  
**Config:** Gearbox only, no external load, no load-side encoder  
**Feedback:** Onboard AS5047P on `axis0`, scaled by 1:36  
**Current limit:** 15 A  
**Test ceiling:** 12 A phase current  
**Breakaway threshold:** 0.03° output

| Settled output position | FWD breakaway | REV breakaway |
|-------------------------|---------------|---------------|
| -3.76° | 1.057 A | 0.419 A |
| -3.74° | 0.581 A | 0.418 A |
| -3.74° | 0.675 A | 0.406 A |
| -3.75° | 0.754 A | 0.427 A |
| -3.20° | 1.429 A | 0.239 A |

Across all ten measurements:

- Mean breakaway current: **0.640 A**
- Maximum breakaway current: **1.429 A**
- Minimum breakaway current: **0.239 A**
- Forward range: **0.581–1.429 A**
- Reverse range: **0.239–0.427 A**

The requested sweep was -3.8° to -1.8°, but position control could not reach
most requested positions. Four samples remained near -3.75°, and the final
sample reached -3.20°. This is therefore a local pre-load baseline, not a full
gearbox rotation map.

The test completed in idle with zero axis, motor, encoder, and controller
errors.

---

## Full-Rotation Baseline After Conditioning

**Date:** 2026-06-09  
**Config:** Gearbox only, no external load, no load-side encoder  
**Feedback:** Onboard AS5047P on `axis0`, scaled by 1:36

Before measuring, the gearbox completed:

- 3.05 output rotations forward at 2 motor turns/s, peak current 2.03 A
- 3.12 output rotations reverse at 2 motor turns/s, peak current 1.63 A
- Maximum FET temperature below 33°C

The stiction map then sampled seven positions over one complete output
rotation:

| Output position | FWD breakaway | REV breakaway | Asymmetry |
|-----------------|---------------|---------------|-----------|
| -27.53° | 0.608 A | 1.026 A | 0.418 A |
| 32.55° | 0.907 A | 0.944 A | 0.037 A |
| 92.47° | 0.938 A | 1.024 A | 0.086 A |
| 152.63° | 1.115 A | 1.033 A | 0.082 A |
| 212.36° | 0.920 A | 0.976 A | 0.057 A |
| 272.44° | 0.956 A | 0.519 A | 0.437 A |
| 332.41° | 0.907 A | 0.884 A | 0.022 A |

Summary across all fourteen measurements:

- Mean breakaway current: **0.911 A**
- Maximum breakaway current: **1.115 A**
- Minimum breakaway current: **0.519 A**
- All seven requested positions were reached accurately

After conditioning, the gearbox rotates continuously and its breakaway current
is mostly uniform and directionally symmetric. The two larger asymmetries occur
near -27.5° and 272.4°, but neither direction exceeds 1.12 A.

The test completed in idle with zero axis, motor, encoder, and controller
errors.

---

## Notes

- All tests run in position control (trap traj) for step map; torque control (ramp) for sticktion measurement
- Load encoder = MT6701 magnetic on output shaft (axis1)
- Motor encoder = ODrive built-in (axis0)
- Settle threshold in step map: 1.5° / 0.5 turns·s⁻¹ (necessary due to high sticktion)
- `robot_sticktion_test.py` restores all ODrive config on exit
