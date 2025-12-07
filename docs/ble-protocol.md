# BLE Protocol Guide

Developer-facing notes for the single custom BLE service exposed by the ring firmware.

## Advertising
- Connectable advertising with device name (`CONFIG_BT_DEVICE_NAME`).
- Includes custom service UUID `d8b8a466-9f41-4a4e-9740-20a1f2b6c8f5` in AD data.

## Service / Characteristics
- **Service UUID:** `d8b8a466-9f41-4a4e-9740-20a1f2b6c8f5`
- **Characteristics (read + notify, little-endian payloads)**
  1) **Measurements** — UUID `b8fd8a95-f86f-4b26-9a9d-7e9b9f3d3dd6`
     - `int16 hr_bpm`          : latest HR, -1 if unknown
     - `int16 spo2_pct`        : latest SpO2, -1 if unknown
     - `int16 temp_c_x100`     : skin temp * 100, INT16_MIN if unknown
     - `uint16 battery_mv`     : VDD in millivolts, 0 if unknown
     - `uint8 ring_mode`       : 0=ACTIVE, 1=IDLE, 2=DEEP_SLEEP
     - `uint8 contact`         : 1 if on-body contact, 0 otherwise
  2) **Motion** — UUID `6e2c8b9c-e0e1-49c9-9c2a-4b0e8a0f3c77`
     - `uint32 step_count`     : step counter with firmware offset applied
     - `uint8 step_state`      : BMA400 activity state (0=still,1=walk,2=run)
     - `uint8[3] reserved`
     - **Write (opcode):** send `0x01` (write without response) to reset the reported step count. This latches the current raw counter as an offset; the sensor is not cleared.

## Notify behavior
- Notifications fire only when the cached payload changes and CCC is enabled for that characteristic.
- Reads always return the latest cached payload.

## Sampling cadence / power interaction
- Measurements follow the power/state machine:
  - ACTIVE: temp ~2s, accel NORMAL, PPG on.
  - IDLE: temp ~10s, accel LP, PPG on.
  - DEEP_SLEEP: temp off, PPG off, accel LP (polled), battery not sampled.
- Battery sampled ~60s when not in DEEP_SLEEP; value is `battery_mv`.

## Endianness
- All multi-byte fields are little-endian (`sys_cpu_to_leXX` in firmware).*** End Patch

