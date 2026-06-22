#!/usr/bin/env python3
"""Measure phase R & L of the LA8308 (no encoder needed) and report them.

MOTOR_CALIBRATION injects DC + a voltage step to measure resistance and
inductance; the rotor may twitch but does not spin. Safe on the bench with no
load. Does NOT need an encoder. Run with the motor wired and the bus at 36 V.
"""

import sys, time
import odrive
from odrive.enums import AXIS_STATE_MOTOR_CALIBRATION, AXIS_STATE_IDLE

POLE_PAIRS = 20            # 40 magnets / 2
CAL_CURRENT = 8.0         # a few A; fine for an 83 mm motor

d = odrive.find_any(timeout=20)
a = d.axis0
print(f"connected {hex(d.serial_number)}  vbus={float(d.vbus_voltage):.1f} V")
if float(d.vbus_voltage) < 20:
    sys.exit("Bus voltage too low -- power the ODrive at 36 V first.")
if any((int(a.error), int(a.motor.error))):
    print(f"clearing prior errors a0={int(a.error)} motor={int(a.motor.error)}")
    try: d.clear_errors()
    except Exception: pass

a.motor.config.pole_pairs = POLE_PAIRS
a.motor.config.calibration_current = CAL_CURRENT
print(f"pole_pairs={POLE_PAIRS} calibration_current={CAL_CURRENT} A -- calibrating...")

a.requested_state = AXIS_STATE_MOTOR_CALIBRATION
time.sleep(0.5)
deadline = time.monotonic() + 15
while a.current_state != AXIS_STATE_IDLE and time.monotonic() < deadline:
    time.sleep(0.2)

print("\n=== RESULT ===")
print(f"is_calibrated     : {a.motor.is_calibrated}")
print(f"phase_resistance  : {float(a.motor.config.phase_resistance):.5f} ohm")
print(f"phase_inductance  : {float(a.motor.config.phase_inductance)*1e6:.1f} uH")
print(f"motor.error       : {int(a.motor.error)}")
if a.motor.is_calibrated and int(a.motor.error) == 0:
    print("\nOK -- send me phase_resistance and I'll compute the safe hold current.")
    # persist so the measured values survive a reboot
    try:
        d.save_configuration()
        print("configuration saved.")
    except Exception as e:
        print(f"(save_configuration skipped: {e})")
else:
    print("\nCalibration did NOT succeed -- check wiring / bus voltage / decode motor.error.")
