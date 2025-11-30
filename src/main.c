#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>
#include <stdint.h>

#include "bma400.h"
#include "maxm86161.h"
#include "max30208.h"
#include "ble_app.h"

int main(void)
{
    int err;

    printk("Ring Firmware Application\n");
    printk("=========================\n\n");

    /* --- Sensor init --------------------------------------------------- */

    err = bma400_app_init();
    if (err) {
        printk("BMA400 app init failed: %d\n", err);
        return err;
    }

    err = maxm86161_app_init();
    if (err) {
        printk("MAXM86161 app init failed: %d\n", err);
        return err;
    }

    err = max30208_app_init();
    if (err) {
        printk("MAX30208 app init failed: %d\n", err);
        return err;
    }

    /* --- BLE init ------------------------------------------------------ */

    err = ble_app_init();
    if (err) {
        printk("BLE init failed: %d\n", err);
        return err;
    }

    printk("Setup complete. Running main loop...\n");

    /* Fake battery level for now (100%). You can later hook this
     * up to an ADC reading of VBAT on a real battery-powered board.
     */
    uint8_t battery_percent = 100;

    while (1) {
        float ax_mg_f = 0.0f;
        float ay_mg_f = 0.0f;
        float az_mg_f = 0.0f;
        float temp_c  = 0.0f;
        int   hr_bpm  = 0;

        /* --- Read accelerometer ---------------------------------------- */
        err = bma400_app_read_mg(&ax_mg_f, &ay_mg_f, &az_mg_f);
        if (err) {
            printk("BMA400 read error: %d\n", err);
        }

        int16_t ax_mg = (int16_t)ax_mg_f;
        int16_t ay_mg = (int16_t)ay_mg_f;
        int16_t az_mg = (int16_t)az_mg_f;

        /* --- Read temperature ------------------------------------------ */
        err = max30208_read_temperature_c(&temp_c);
        if (err) {
            printk("MAX30208 read error: %d\n", err);
            temp_c = 0.0f;
        }

        /* --- Service MAXM86161 FIFO & HR estimation -------------------- */
        maxm86161_app_print();  /* Still prints debug info + updates HR state */

        hr_bpm = maxm86161_get_hr_bpm();

        /* --- Push values into BLE service ------------------------------ */
        ble_app_update_sensors(ax_mg, ay_mg, az_mg,
                               temp_c,
                               hr_bpm,
                               battery_percent);

        /* ~5 Hz update rate (200 ms) */
        k_sleep(K_MSEC(200));
    }

    return 0;
}
