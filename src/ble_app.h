#ifndef BLE_APP_H_
#define BLE_APP_H_

#include <zephyr/kernel.h>
#include <stdint.h>

/**
 * Initialize Bluetooth LE stack, register GATT service and start advertising.
 *
 * Returns 0 on success, negative errno on failure.
 */
int ble_app_init(void);

/**
 * Update the latest sensor values and (if notifications are enabled
 * and there is an active connection) send them to the client.
 *
 * accel_*: signed milli-g (mg)
 * temp_c:  degrees Celsius
 * hr_bpm:  beats per minute, 0 = no valid HR
 * battery_percent: 0–100 %
 */

 void ble_app_update_sensors(int16_t accel_x_mg,
    int16_t accel_y_mg,
    int16_t accel_z_mg,
    float   temp_c,
    int     hr_bpm,
    uint8_t battery_percent);

#endif /* BLE_APP_H_ */
