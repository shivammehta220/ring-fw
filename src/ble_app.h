/*
 * Simple BLE helper for the ring firmware.
 *
 * Provides a minimal custom service that exposes:
 *   - HR (BPM)
 *   - SpO2 (%)
 *   - Skin temperature (centi-deg C)
 *   - Battery voltage (mV)
 *   - Sleep/power state (ring_mode_t)
 *   - Contact state (on/off body)
 *
 * The service supports read + notify on a single characteristic to keep the
 * ATT footprint small. All values are little-endian in the packed payload.
 */

#ifndef BLE_APP_H_
#define BLE_APP_H_

#include <stdint.h>
#include <stdbool.h>

typedef enum {
    RING_MODE_ACTIVE = 0,
    RING_MODE_IDLE,
    RING_MODE_DEEP_SLEEP,
} ring_mode_t;

struct ring_ble_snapshot {
    int16_t     hr_bpm;        /* -1 if unknown */
    int16_t     spo2_pct;      /* -1 if unknown */
    int16_t     temp_c_x100;   /* INT16_MIN if unknown */
    uint16_t    battery_mv;    /* 0 if unknown */
    ring_mode_t ring_mode;     /* matches ring_mode_t values */
    bool        contact;       /* true if skin contact present */
    uint32_t    step_count;    /* total steps from BMA400 */
    uint8_t     step_state;    /* bma400_step_stat_t encoded */
};

/**
 * @brief Initialize BLE, start advertising, and prepare the custom service.
 *
 * @return 0 on success, negative errno on failure.
 */
int ble_app_init(void);

/**
 * @brief Sample battery voltage (mV) using SAADC VDD/4 channel.
 *
 * Caches the ADC configuration, performs a single conversion, and returns
 * the result in millivolts (actual VDD).
 *
 * @param[out] mv Pointer to store measured millivolts.
 * @return 0 on success, negative errno on failure.
 */
int ble_app_sample_battery_mv(uint16_t *mv);

/**
 * @brief Update the characteristic value and send a notify if subscribed.
 *
 * @param snapshot New sensor + state snapshot.
 */
void ble_app_publish(const struct ring_ble_snapshot *snapshot);

/**
 * @brief Returns true if a central is currently connected.
 */
bool ble_app_is_connected(void);

#endif /* BLE_APP_H_ */
