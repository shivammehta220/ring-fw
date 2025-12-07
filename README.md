# Ring Firmware (Zephyr)

Application firmware for a wearable device built on Zephyr RTOS. It runs a single cooperative application thread that samples sensors, decides the power/state mode, and advertises data over BLE. With no busy-wait loops, Zephyr can drop into WFI between `k_sleep()` calls for maximum efficiency.

## Firmware Architecture
- **Main loop:** `src/main.c` runs a simple loop at `RING_MAIN_LOOP_PERIOD_MS` (200 ms). Each iteration:
  - Services PPG FIFO when not in deep sleep.
  - Samples the accelerometer on a cadence that depends on power mode.
  - Samples temperature and battery at mode-dependent cadences.
  - Updates the power/state machine (`ring_update_state`) based on motion + contact.
  - Publishes a consolidated snapshot to BLE via `ble_app_publish()`.
- **Single thread:** No additional worker threads; Zephyr handles idle/wake.
- **Power/state machine:** Uses `ring_mode_t` with three modes (Active, Idle, Deep Sleep).

## Power / State Model
- **ACTIVE**
  - Condition: On-body (`maxm86161_has_contact() == true`) and recent motion (`!bma400_app_get_sleep_flag()`).
  - Config: PPG ON, BMA400 in `NORMAL`, temp every ~2 s.
- **IDLE**
  - Condition: On-body but no motion for ~30 s (BMA400 sleep flag true).
  - Config: PPG still ON, BMA400 in `LP`, temp every ~10 s.
- **DEEP_SLEEP**
  - Condition: Contact false for >2 min.
  - Config: PPG OFF, temp OFF, BMA400 in `LP`, main loop keeps running to poll motion.
- **Transitions**
  - Active ↔ Idle: BMA400 no-motion flag (~30 s, set via `bma400_app_set_sleep_params()`).
  - Idle → Deep Sleep: contact false for >120 s.
  - Deep Sleep → Idle: motion detected again (no-motion flag clears) while in LP; PPG re-enabled on exit.

## Sensor Behavior
- **MAXM86161 (PPG/HR/SpO2/contact)**
  - Enabled in ACTIVE/IDLE, disabled in DEEP_SLEEP via `maxm86161_set_enabled(false)`.
  - Data pulled from getters: `maxm86161_get_hr_bpm()`, `maxm86161_get_spo2_pct()`, `maxm86161_has_contact()`.
- **BMA400 (accelerometer/steps/no-motion)**
  - Power modes: `NORMAL` in ACTIVE, `LP` in IDLE/DEEP_SLEEP.
  - No-motion flag (`bma400_app_get_sleep_flag()`) drives ACTIVE↔IDLE and wake from DEEP_SLEEP.
  - Step counter and activity state via `bma400_app_read_step_counter()`.
- **MAX30208 (skin temperature)**
  - Single-shot reads only when not in DEEP_SLEEP.
  - Cadence: ~2 s in ACTIVE, ~10 s in IDLE.
  - Read with `max30208_read_temperature_c()`.
- **Battery (VDD/4 SAADC)**
  - Sampled infrequently (`RING_BATT_PERIOD_MS` = 60 s) when not in DEEP_SLEEP via `ble_app_sample_battery_mv()`.

## BLE Service Model (single custom service)
- Located in `src/ble_app.c` / `src/ble_app.h`.
- **One custom service**, two characteristics (read + notify):
  - **Measurements** (`ring_data_uuid`): HR (BPM), SpO2 (%), skin temp (centi-deg C), battery mV, ring mode, contact flag.
  - **Motion** (`ring_motion_uuid`): step count (u32) and activity state; also accepts a **write-without-response opcode 0x01** to reset the reported step count (implemented as an offset, does not clear the sensor).
- **Data flow**
  - `main.c` builds `struct ring_ble_snapshot` and calls `ble_app_publish(&snap);`.
  - `ble_app_publish` caches, applies step offset, and notifies only when values change and CCC is enabled.
- **UUIDs**
  - Service: `d8b8a466-9f41-4a4e-9740-20a1f2b6c8f5`
  - Measurements: `b8fd8a95-f86f-4b26-9a9d-7e9b9f3d3dd6`
  - Motion: `6e2c8b9c-e0e1-49c9-9c2a-4b0e8a0f3c77`
- **Connection helpers**
  - `ble_app_is_connected()` indicates link status.
  - Advertising uses only the custom service UUID plus the device name.

## Developing / Extending BLE
1. **Add a new field to the snapshot:**
   - Extend `struct ring_ble_snapshot` in `src/ble_app.h`.
   - Update producers in `main.c` to fill the new field.
2. **Expose over BLE:**
   - Add packing to `ble_app_publish()` (`src/ble_app.c`) and extend the relevant payload struct.
   - If adding another characteristic, extend the GATT definition in `BT_GATT_SERVICE_DEFINE`.
3. **Endianness:** All values are little-endian; use `sys_cpu_to_leXX`.
4. **Notify behavior:** Notifications only fire when the payload changes and CCC is enabled; reads always return the latest cached payload.
5. **Testing:** Pair with a BLE client (nRF Connect, etc.), read characteristics, then enable notify to observe streamed updates as the ring state/sensors change.

## Building / Running
- Standard Zephyr flow; board definitions live under `boards/nordic/nrf52_dev/`.
- Configure via `prj.conf`; application sources under `src/`.
- Run west build/flash for your target board, e.g.:
  - `west build -b nrf52_dev_nrf52832`
  - `west flash`

## Quick Behavior Summary
- On finger, moving → ACTIVE: PPG ON, accel NORMAL, temp fast.
- On finger, resting → IDLE: PPG ON, accel LP, temp slower.
- Off finger for ~2 min → DEEP_SLEEP: PPG OFF, temp OFF, accel LP keeps motion wake.
- Motion after deep sleep → IDLE, re-enables PPG and resumes sampling.
