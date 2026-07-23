# Kinetic Sculpture Hexapod — Design Notes & Plan

Status: **design discussion, not started.** Nothing connected/calibrated yet.
Captured 2026-07-23 to resume later.

## Context / Pivot

The robot won't be ready for the event. Instead we build a **kinetic sculpture**:
a **6-legged hexapod, 3 joints per leg = 18 ODrive nodes**, that "dangles its legs
in the air" with a **walking-like gait (tripod gait)**. It is a **living
installation running for hours**, unattended.

Legs hang on a **complex body at various heights and angles**. Legs are **in the
air** — no ground contact, so gait is aesthetic, not load-bearing.

### Decisions locked in this discussion
- **No ROS.** Single host, single CAN bus, single process. ROS2 adds a node
  graph / DDS / lifecycle = more things to fail unattended at 3am. A structured
  single process (Python asyncio or C++) is more robust for this. ROS only buys
  live tuning/viz (dev convenience), not runtime value.
- **Motion model = IK** (user chose the "honest" Cartesian gait over pure
  joint-space choreography). Foot trajectory (swing/stance ellipse) per leg in
  leg frame → inverse kinematics → 3 joint angles. Needs segment lengths +
  mounting transforms.
- **Transport = SocketCAN + python-can** on the Orange Pi (replace the ESP32
  slcan serial bridge used on the macOS bench). Firmware "will be changed."
- Runtime host = **Orange Pi**, currently running "latest Ubuntu" (to be
  reconsidered — see OS section).

## Hardware / power facts
- 18 ODrive boards, single-axis each (one motor per board), on **one CAN bus**.
- **No battery.** Powered from **mains PSU**. Each motor has its own **2 Ω brake
  resistor** (board-level on ODrive → 1 resistor per board, maps cleanly).
- Gearbox is **NOT non-backdrivable** — it does *not* hold pose at zero current.
  But holding torque is now small: the 5 kg @ 1 m load that destroyed motor #10
  (~98 Nm) is **gone**; now only the leg's own weight (~3–10 Nm near horizontal
  → ~1–2 A hold). Thermal is manageable but not free.
- **~70% of motors have a thermistor; ~30% do not.** The no-thermistor motors
  are a blind thermal spot for an hours-long run.
- **PSU: 48 V / 25 A = 1200 W** for 18 motors ≈ 67 W/motor avg (fine for a gentle
  dangle gait). All 18 boards share ONE 48 V rail from this one PSU.
- ⚠️ **48 V on a v3.6-56V board = only ~6–8 V regen headroom** before the FET
  limit. Set `dc_bus_overvoltage_trip_level ≈ 54 V`. The brake window (48→54 V)
  is TIGHT → any hard decel overshoots 54 V and trips `DC_BUS_OVER_VOLTAGE`.
  **Gait MUST be smooth** (bounded accel/decel, velocity cap, no gravity drops).
- **Shared rail helps regen:** braking legs raise the common rail, accelerating
  legs sink it → tripod phasing balances source/sink, net regen is small. 18
  brake resistors on the shared rail ≈ 18×2 Ω ∥ ≈ 0.11 Ω dump capacity.
- Brake resistor dissipation: `54²/2 ≈ 1.5 kW` peak per resistor during a chop
  pulse (duty-cycled). **Verify brake-resistor wattage rating** (stock ODrive 2 Ω
  ≈ 50 W — likely OK for gentle gait, confirm).
- **Inrush:** no battery buffer; 25 A PSU sees surges directly. Cold-start of 18
  boards (bulk caps) will exceed 25 A for ms → **staged power-up (groups) or
  inrush limiter (NTC / precharge relay) is MANDATORY.**

## The four/five subsystems & their gotchas

### 1. Thermal (main risk, but de-risked vs the robot)
- Motor #10 died from **continuous gravity-hold under 5 kg @ 1 m**; that load is
  gone. Now it's leg self-weight only.
- Keep the gait **dynamic and low-current**; design so **no leg statically holds
  near horizontal** (max gravity moment) — pass through it, don't dwell.
- Insert **rest phases** (variety in the choreography) to let motors cool.
- **Wire the remaining ~30% thermistors** (preferred), or assign those joints to
  low-moment roles (e.g. tibia, not the weight-bearing femur).

### 2. Power / regeneration (new subsystem — no battery)
- Battery used to **absorb regen** (`dc_max_negative_current=-10 / max_regen=10`
  each powerup). Mains PSU **cannot sink current** → every decel / gravity-driven
  move pushes energy into the DC bus → `DC_BUS_OVER_VOLTAGE` or dead PSU.
- The **2 Ω brake resistors are the sink.** Per-board config (18×):
  ```python
  odrv0.config.brake_resistance = 2.0
  odrv0.config.enable_brake_resistor = True
  odrv0.config.dc_max_negative_current = -1.0   # do NOT push current into mains PSU
  odrv0.config.dc_bus_overvoltage_trip_level = 54.0  # 48V supply, 56V board → tight window
  odrv0.save_configuration()   # applies at boot
  ```
- After reboot **verify `odrv0.brake_resistor_armed == True`**. A **disarmed brake
  resistor on mains = overvoltage risk** → supervisor treats as CRITICAL, safe-stop.
- Brake resistors **get hot** (2 Ω @ ~24 V → up to ~288 W peak while chopping,
  duty-cycled). 18 of them add to the enclosure thermal budget — space them out.
- **Gait must be smooth** (bounded accel/decel, no gravity "drops") to keep regen
  within what 2 Ω can dissipate. Power + thermal both push toward slow motion.
- **PSU sizing:** no battery buffer → PSU sees surges directly. Tripod phasing
  helps (only half the legs in heavy swing at once). Watch **inrush at power-on**
  of 18 boards (staged power-up / inrush limiter).
- **E-stop:** don't cut mains mid-motion (motors generate into nothing). Do a
  controlled stop: IDLE all axes → then remove power.

### 3. CAN bandwidth (18 nodes on one bus)
- Old bench pacing: 4 ms between frames (ESP32 bridge drops back-to-back frames).
  18 nodes × 4 ms = **72 ms/tick → ~13 Hz max**. Too slow for smooth gait.
- Fixes:
  - Flash the **ESP32 bridge back-to-back-drop fix** (memory: "FIXED, NOT YET
    FLASHED") — or better, drop the bridge entirely for SocketCAN.
  - **SocketCAN + USB-CAN adapter** (CANable/candleLight `gs_usb`, or Orange Pi
    native controller): kernel TX queue, no manual pacing, real 500k–1M bitrate,
    `candump`/`cansend` debug, and **it surfaces bus error frames** (see diag).
  - **Stream setpoints at 15–20 Hz and enable `INPUT_MODE_POS_FILTER`** on the
    ODrive (2nd-order, `input_filter_bandwidth`) so the controller smooths
    between sparse setpoints → smooth motion at low CAN trafiic. **Key trick for
    18 axes.**

### 4. Diagnostics (mandatory for unattended hours)
Two distinct error domains — collect **both**.

**A) ODrive application faults** — what's available over CAN in THIS fork
(`Firmware/communication/can_simple.hpp`, IDs sequential from 0x000):

| ID | Message | Type | Gives |
|----|---------|------|-------|
| 0x001 | Heartbeat | broadcast | **axis error (32-bit) + state** — free, passive |
| 0x003 | Get_Motor_Error | RTR | motor error |
| 0x004 | Get_Encoder_Error | RTR | encoder error |
| 0x005 | Get_Sensorless_Error | RTR | sensorless error |
| 0x014 | Get_Iq | RTR | Iq setpoint/measured |
| 0x017 | Get_Vbus_Voltage | RTR | bus voltage (per board) |

  - Passive heartbeat capture on all 18 = baseline (axis err + state).
  - Round-robin RTR poll for specific cause + Vbus + Iq.
  - Decode bitfields via **version-matched** `tools/odrive/enums.py` (NOT memory).

**B) CAN transport errors** (separate layer, easy to miss): bus-off,
error-passive/warning, ACK/CRC/form/stuff, lost arbitration, TX-queue overrun.
  - `python-can` gives **error frames** (`msg.is_error_frame`).
  - `ip -s -d link show can0` → RX/TX errors, bus-off count, restart count, state.
  - Set `restart-ms` for auto bus recovery.
  - **This ESP32 bridge exposes bus state via the slcan `F` status byte**
    (error-warning/passive, overrun, arb-lost, bus-off) — PROBED working. The
    kernel slcan driver won't surface it in `ip link`, so **poll `F` in our own
    diagnostics layer** (works on both the Mac slcan backend and Linux).

**Two gaps in this fork — decide:**
  1. **No `controller.error` over CAN** (no Get_Controller_Error). You see
     `AXIS_ERROR_CONTROLLER_FAILED` in heartbeat but not the cause code (USB only).
     Usually tolerable.
  2. **No temperature over CAN at all** (no Get_Temperature; 0x015 is
     Sensorless_Estimates). Thermal is risk #1 → you'd only learn of overheat
     *after* the thermistor trips it into a motor/axis error; **no temperature
     trend in the log.** Options: live with trip-only detection, OR
     **PATCH THE FIRMWARE to add a CAN temperature message** (broadcast or RTR).
     We already patch CAN Simple (the `Get_Encoder_Estimates` split-feedback
     patch in AGENTS.md is the same pattern). **Recommendation: do the patch** —
     continuous temp log of 18 motors is worth it, especially with 30% lacking a
     thermistor.

Log to a **rotating structured store** (jsonl or sqlite): node, timestamp, axis
err, motor/enc/sensorless err, Iq, Vbus, (temp if patched), CAN bus state. On any
axis error, snapshot the node's full sub-error set + recent command history.

### 5. Supervisor / watchdog (it's a supervisor, not a script)
Runs for hours → one-shot scripts that abort on first fault won't do. Needs:
- Watchdog over heartbeat + temperature on 18 nodes.
- Auto clear / re-home / resume after USB/CAN drop or recoverable fault.
- CRITICAL faults (brake_resistor disarmed, overvoltage, overtemp) → safe-stop.
- Motion variety + rest periods (avoid hypnotic repeat; let motors cool).
- Run as a **systemd service, `Restart=always`**, HW/SW watchdog.

## Engine architecture (layers)
```
┌ choreography / sequencer ─ gait patterns, pauses, variety, motor "rest"
├ gait generator ─────────── tripod/wave/ripple phasing → foot trajectory (swing/stance ellipse), bounded accel
├ inverse kinematics ─────── foot(x,y,z) → (coxa, femur, tibia), per-leg mounting transform
├ joint mapper ───────────── joint angle → output-turns w/ per-node zero/direction/limits (18×)
├ CAN transport (SocketCAN)─ SET_INPUT_POS @15–20 Hz + POS_FILTER smoothing; python-can
├ diagnostics ────────────── fault collector + CAN-bus health + correlator → rotating log
└ supervisor / watchdog ──── heartbeat+thermal, auto recover, safe-stop, systemd
```

## OS on Orange Pi — board CONFIRMED: **Orange Pi 5 Pro, RK3588S, 16 GB**
- Compute is massively overkill (8 cores 4×A76+4×A55 / 16 GB LPDDR5); IK+gait for
  18 joints is trivial. Focus everything on **reliability + jitter**, not perf.
- **OS:** Joshua Riek `ubuntu-rockchip` **Ubuntu 22.04 Server** (best-maintained
  RK3588 image, hardware works OOB) — or Armbian Jammy Server (BSP 6.1). Headless.
  Jammy (22.04) over 24.04: Rockchip BSP kernel is better-baked on 22.04. Pin the
  version, **`unattended-upgrades` DISABLED**.
- CPU governor `performance`, PM off, **dedicate one A76 core** to the control
  loop (`isolcpus` + `taskset` + `SCHED_FIFO`). PREEMPT_RT not needed — jitter
  floor is CAN, not CPU.

### CAN transport — KEEP the user's ESP32 board, one code on Mac + Linux
The user's own CAN board (ESP32 bridge) **already speaks slcan/LAWICEL ASCII**:
the existing scripts send `t%03X%X…` / `r%03X%X` = standard slcan frames
(`tIIILDD`, `r`=RTR). So **`python-can` talks to the same board on both
platforms — no bridge protocol rewrite:**
- **macOS** (no SocketCAN): `can.Bus(interface="slcan",
  channel="/dev/cu.usbmodemXXXX", bitrate=1_000_000)` — slcan backend over serial.
- **Orange Pi (Linux):** preferred = attach via `slcand` → real `can0`
  (kernel bus-off recovery, `candump`, error frames):
  ```bash
  sudo slcand -o -c -s8 -S 1000000 /dev/ttyACM0 can0   # s8 = 1 Mbit
  sudo ip link set can0 up type can
  sudo ip link set can0 txqueuelen 1000
  ```
  then `can.Bus(interface="socketcan", channel="can0")`. (Or same `slcan`
  backend on Linux if not using slcand.)
- Transport abstracted behind `python-can`; a factory picks backend by
  `sys.platform`. Gait/diag code is backend-agnostic.

**Bridge PROBED & VALIDATED 2026-07-23** (port `/dev/cu.usbmodem31101`):
- Full standard slcan/LAWICEL impl — all control cmds ack with CR: `V`→`V1050`,
  `N`→`NF001`, `F`→`F00` (status flags), `C`/`S8`/`O`/`C` all `\r` ok. So
  **`slcand` (Linux) + `python-can` slcan (Mac) attach cleanly — NO firmware
  change needed for control commands.** (The earlier "add O/C/S/F" touch-up is
  already done.)
- Bridge does NOT auto-stream; needs `O` to forward frames (standard slcan).
- **Bus-error diagnostics via `F` polling:** the `F` status byte encodes RX/TX
  overflow, error-warning, **error-passive, arbitration-lost, bus-off**. The
  kernel `slcan` driver is dumb and does NOT poll `F` (so `ip link` state won't
  reflect bus-off for a slcan iface) → **our diagnostics layer polls `F` itself**.
  Since `F` works, the transport-error visibility gap is solved.

**ESP32 firmware — only remaining item:**
- Flash the **back-to-back-drop fix** (memory: FIXED, not yet flashed) → then drop
  the 4 ms frame pacing. Independent of the above; flash when convenient.
- Serial-speed: **CONFIRMED native USB, no bottleneck.** Board enumerates as
  Espressif `0x303A` "USB JTAG_serial debug unit" (ESP32-S3 built-in USB-Serial-
  JTAG), **Full-Speed 12 Mbit** → baud nominal; the ~550 frames/s UART concern is
  moot. Port on the Mac bench: `/dev/cu.usbmodem31101`. (Nuance: USB-Serial-JTAG
  has a small TX buffer and can stall if the host stops reading; for max
  robustness later, move the bridge to USB-OTG CDC / TinyUSB. Not urgent.)

**DECISION: keep the ESP32 slcan board** (above), used via `slcand`→`socketcan`
on the Orange Pi and the `slcan` backend on macOS. One `python-can` codebase.

Future option (not now): **native RK3588S CAN-FD** — lowest latency, drops the
USB layer, but needs a **device-tree overlay** (CAN TX/RX are muxed on the 5 Pro
40-pin header) + an **external 3.3 V transceiver** (SN65HVD230 / TJA1050); check
the 5 Pro schematic/pinout for pins/conflicts. Because transport is abstracted
via `python-can`, moving to native CAN later = no gait/diag code change.

Either way: `can0` @ **1 Mbit**, `restart-ms 100` (auto bus-off recovery), larger
`txqueuelen`, `can-utils` for debug.

## Phased build plan
1. **Transport layer:** SocketCAN + python-can; heartbeat + thermal telemetry
   across 18 nodes (verifiable before any IK).
2. **18-joint bring-up** (the bulk of the work): per-motor runbook (Kt, offset,
   harmonic, thermistor) + per-joint mechanical zero/direction/limits +
   brake-resistor config. Build a config table (18 nodes).
3. **Kinematic model:** URDF-like config (segment lengths coxa/femur/tibia +
   mounting pose per leg) + IK solver; verify on ONE leg.
4. **Gait generator + tripod phasing:** dry-run in the air on 2 legs, then 6.
5. **Supervisor + choreography + diagnostics** + autostart as a service.

## OPEN QUESTIONS (needed to proceed)
1. ~~Which Orange Pi model~~ **ANSWERED: Orange Pi 5 Pro, RK3588S, 16 GB.** →
   Ubuntu 22.04 Server (ubuntu-rockchip/Armbian); native CAN-FD exists but needs
   DT overlay + transceiver — recommend USB-CAN start. (see OS section)
2. ~~CAN adapter~~ **ANSWERED: keep the user's ESP32 slcan board** — works on both
   Mac (python-can `slcan`) and Orange Pi (`slcand`→`socketcan`), no protocol
   rewrite. Small firmware touch-ups: slcan control cmds `O/C/S/F`, error frames
   (for bus diag), back-to-back-drop fix, serial-speed check.
3. **Kinematics:** do segment lengths (coxa/femur/tibia) + body mounting geometry
   exist (CAD/drawing)? IK can't be designed without them.
4. **Joint map:** which node = coxa/femur/tibia per leg, node IDs, which lack a
   thermistor.
5. **Left/right mirroring:** same joint sign convention on all legs or mirrored?
6. ~~PSU voltage & current rating~~ **ANSWERED: 48 V / 25 A / 1200 W, shared rail.**
   → overvoltage trip ~54 V (tight 6–8 V regen headroom on 56V board), staged
   power-up mandatory, verify brake-resistor wattage. (see Power section)
7. **Firmware:** patch in a CAN **temperature** message? (recommend yes)

## Existing assets to reuse
- `spider-motor-tools/can_sync_cycle.py` — multi-node closed-loop entry, synchronized
  streaming, heartbeat/Iq fault handling, safe idle. Good safety skeleton.
- `spider-motor-tools/can_goto.py`, `can_diag_move.py` — single-joint moves.
- `tools/odrive/enums.py` — version-matched error/state/mode decode.
- Per-motor bring-up runbook + tools (`measure_kt.py`, `pin_calibration.py`,
  `health_check`, harmonic calibration) — see MEMORY.md.
- `Firmware/communication/can_simple.{hpp,cpp}` — CAN command table + patch site
  (split-feedback `Get_Encoder_Estimates` patch is the template for a temp msg).
```
