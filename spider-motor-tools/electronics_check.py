#!/usr/bin/env python3
"""Bench electronics check for a newly-boxed motor axis (no load, free spin).

Probes, without moving the motor:
  - board link / serial / vbus
  - onboard AS5047P (motor-shaft absolute encoder, SPI)
  - external MT6701 (joint absolute encoder, SPI) -- SPI/CRC health even with
    NO MAGNET fitted (angle will be meaningless, but the SSI frame + CRC must
    still be valid to prove wiring/CS/clocking are good)
  - motor thermistor (GPIO4)
  - CAN interface config/state

Read-only: does NOT arm, calibrate, or spin the motor.
"""
import time
import odrive


def connect(serial=None, timeout=15, attempts=8):
    # Bare find + our own case-insensitive serial compare (modern find_any
    # serial filter is case-sensitive and the board reports uppercase).
    want = serial.lower() if serial else None
    last = None
    for i in range(attempts):
        try:
            dev = odrive.find_any(timeout=timeout)
            got = format(dev.serial_number, "x").lower()
            if want and got != want:
                raise RuntimeError(f"found {got}, wanted {want}")
            return dev
        except Exception as ex:  # noqa: BLE001
            last = ex
            print(f"  connect attempt {i+1} failed ({type(ex).__name__}); retrying...", flush=True)
            time.sleep(1.0)
    raise last


def g(obj, name, default="<none>"):
    try:
        return getattr(obj, name)
    except Exception as ex:  # noqa: BLE001
        return f"<err {type(ex).__name__}>"


def probe_spi_encoder(enc, label):
    print(f"\n=== {label} encoder (axis SPI) ===")
    cfg = enc.config
    print(f"  mode={g(cfg,'mode')} cpr={g(cfg,'cpr')} cs_pin={g(cfg,'abs_spi_cs_gpio_pin')} "
          f"pre_calibrated={g(cfg,'pre_calibrated')}")
    print(f"  error=0x{g(enc,'error'):x}  is_ready={g(enc,'is_ready')} index_found={g(enc,'index_found')}")
    print(f"  pos_estimate={g(enc,'pos_estimate'):.4f}  pos_abs={g(enc,'pos_abs')}  "
          f"count_in_cpr={g(enc,'count_in_cpr')}")
    print(f"  spi_error_rate={g(enc,'spi_error_rate')}")


def probe_mt6701(enc, label):
    print(f"\n=== {label} MT6701 debug (SPI/CRC, magnet-independent) ===")
    # Force a fresh sample if the helper exists.
    try:
        enc.mt6701_debug_sample()
        time.sleep(0.05)
    except Exception as ex:  # noqa: BLE001
        print(f"  (mt6701_debug_sample unavailable: {type(ex).__name__})")
    for _ in range(5):
        try:
            enc.mt6701_debug_sample()
        except Exception:  # noqa: BLE001
            break
        time.sleep(0.02)
    fields = [
        "mt6701_debug_mode",
        "mt6701_debug_word0", "mt6701_debug_word1", "mt6701_debug_raw24",
        "mt6701_debug_pos",
        "mt6701_debug_crc_recv", "mt6701_debug_crc_calc", "mt6701_debug_crc_ok",
        "mt6701_debug_sample_count", "mt6701_debug_bad_crc_count",
        "mt6701_debug_request_count",
        "mt6701_debug_start_ok_count", "mt6701_debug_start_fail_count",
    ]
    for f in fields:
        v = g(enc, f)
        if isinstance(v, int) and "word" in f or f == "mt6701_debug_raw24":
            print(f"  {f} = {v} (0x{v:x})")
        else:
            print(f"  {f} = {v}")


def probe_thermistor(odrv, axis, label):
    print(f"\n=== {label} motor thermistor ===")
    for path in ("motor_thermistor", "fet_thermistor"):
        obj = getattr(axis, path, None)
        if obj is None:
            print(f"  {path}: <absent>")
            continue
        cfg = getattr(obj, "config", None)
        gpio = g(cfg, "gpio_pin")
        raw = "?"
        try:
            if isinstance(gpio, int):
                raw = f"{odrv.get_adc_voltage(gpio):.4f}V"
        except Exception:  # noqa: BLE001
            pass
        print(f"  {path}: temp={g(obj,'temperature')}  raw={raw}  enabled={g(cfg,'enabled')} "
              f"gpio={gpio} lo={g(cfg,'temp_limit_lower')} hi={g(cfg,'temp_limit_upper')}")


def probe_can(odrv):
    print("\n=== CAN ===")
    can = getattr(odrv, "can", None)
    if can is None:
        print("  <no can object>")
        return
    cfg = getattr(can, "config", None)
    print(f"  baud={g(cfg,'baud_rate')}  protocol={g(cfg,'protocol')}  error=0x{g(can,'error'):x}")
    for ax_name in ("axis0", "axis1"):
        ax = getattr(odrv, ax_name, None)
        if ax is None:
            continue
        ccfg = ax.config  # can fields live flat on axis.config in 0.5.1
        print(f"  {ax_name}: node_id={g(ccfg,'can_node_id')} "
              f"node_id_extended={g(ccfg,'can_node_id_extended')} "
              f"heartbeat_rate_ms={g(ccfg,'can_heartbeat_rate_ms')}")


def main():
    print("connecting...", flush=True)
    odrv = connect()
    sn = format(odrv.serial_number, "x")
    print(f"serial: {sn}  vbus: {round(odrv.vbus_voltage,2)} V  "
          f"fw: {g(odrv,'fw_version_major')}.{g(odrv,'fw_version_minor')}.{g(odrv,'fw_version_revision')}  "
          f"hw: {g(odrv,'hw_version_major')}.{g(odrv,'hw_version_minor')}")
    print(f"user_config_loaded: {g(odrv,'user_config_loaded')}")
    print(f"axis errors: a0={g(odrv.axis0,'error')} a1={g(odrv.axis1,'error')}")

    # axis0 = onboard AS5047P (motor shaft). axis1 = external MT6701 (joint).
    probe_spi_encoder(odrv.axis0.encoder, "axis0 / onboard AS5047P")
    probe_spi_encoder(odrv.axis1.encoder, "axis1 / external MT6701")
    probe_mt6701(odrv.axis1.encoder, "axis1")
    # also dump mt6701 debug on axis0 in case wiring differs on this box
    probe_thermistor(odrv, odrv.axis0, "axis0")
    probe_can(odrv)
    print("\ndone.", flush=True)


if __name__ == "__main__":
    main()
