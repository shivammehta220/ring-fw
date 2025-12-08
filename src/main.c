#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>
#include <stdint.h>
#include <stdbool.h>
#include <limits.h>

#include "maxm86161.h"
#include "bma400.h"
#include "max30208.h"
#include "ble_app.h"

/* ------------------------------------------------------------------------- */
/* High-level ring state machine                                             */
/* ------------------------------------------------------------------------- */

static ring_mode_t ring_mode = RING_MODE_ACTIVE;

static const char *ring_mode_name(ring_mode_t m)
{
    switch (m) {
    case RING_MODE_ACTIVE:     return "ACTIVE";
    case RING_MODE_IDLE:       return "IDLE";
    case RING_MODE_DEEP_SLEEP: return "DEEP_SLEEP";
    default:                   return "?";
    }
}

/* Timing parameters (tweak as you like) */
#define RING_NO_CONTACT_TO_DEEP_SLEEP_MS   (120000) /* 2 minutes off-body */
#define RING_TEMP_PERIOD_ACTIVE_MS          (2000)  /* 2 s when active */
#define RING_TEMP_PERIOD_IDLE_MS           (10000)  /* 10 s when idle */
#define RING_BMA_PERIOD_MS                  (1000)  /* 1 s normally */
#define RING_DEEP_SLEEP_BMA_PERIOD_MS       (5000)  /* 5 s in deep sleep */
#define RING_MAIN_LOOP_PERIOD_MS             200    /* 5 Hz loop base */
#define RING_BATT_PERIOD_MS                (60000)  /* 60 s battery sample */

/* Timestamp bookkeeping */
static int64_t last_contact_ts_ms;
static int64_t last_temp_sample_ms;
static int64_t last_bma_sample_ms;
static int64_t last_batt_sample_ms;

static float    last_temp_c;
static bool     have_temp_sample;
static uint16_t last_batt_mv;
static bool     have_batt_sample;

/* Last known good step counter values (guarded against transient read failures) */
static uint32_t last_step_count = 0;
static bma400_step_stat_t last_step_stat = BMA400_STEP_STAT_STILL;

/* Apply mode → sensor power configuration */
static void ring_apply_mode(ring_mode_t new_mode)
{
    int ret;

    switch (new_mode) {
    case RING_MODE_ACTIVE:
        /* On-body + moving: full accel, PPG on */
        ret = maxm86161_set_enabled(true);
        if (ret) {
            printk("Ring: maxm86161_set_enabled(true) failed: %d\n", ret);
        }
        ret = bma400_app_set_power_mode(BMA400_POWER_MODE_NORMAL);
        if (ret) {
            printk("Ring: bma400_set_power_mode(NORMAL) failed: %d\n", ret);
        }
        break;

    case RING_MODE_IDLE:
        /* On-body but resting: keep PPG on, accel in LP */
        ret = maxm86161_set_enabled(true);
        if (ret) {
            printk("Ring: maxm86161_set_enabled(true) failed: %d\n", ret);
        }
        ret = bma400_app_set_power_mode(BMA400_POWER_MODE_LP);
        if (ret) {
            printk("Ring: bma400_set_power_mode(LP) failed: %d\n", ret);
        }
        break;

    case RING_MODE_DEEP_SLEEP:
        /* Off-body: shut down PPG, keep accel in LP for wake-on-motion */
        ret = maxm86161_set_enabled(false);
        if (ret) {
            printk("Ring: maxm86161_set_enabled(false) failed: %d\n", ret);
        }
        ret = bma400_app_set_power_mode(BMA400_POWER_MODE_LP);
        if (ret) {
            printk("Ring: bma400_set_power_mode(LP) failed: %d\n", ret);
        }
        break;
    }
}

/* Update state machine based on BMA400 no-motion + PPG contact */
static void ring_update_state(int64_t now_ms)
{
    /* BMA400 "sleep flag" means: no steps for configured timeout */
    bool no_motion_short = bma400_app_get_sleep_flag();
    bool on_body         = maxm86161_has_contact();

    if (on_body) {
        last_contact_ts_ms = now_ms;
    }

    ring_mode_t new_mode = ring_mode;

    if (ring_mode == RING_MODE_DEEP_SLEEP) {
        /* In deep sleep we can't rely on PPG contact (PPG is off).
         * Wake up when BMA400 sees motion again (no-motion flag clears).
         */
        if (!no_motion_short) {
            new_mode = RING_MODE_IDLE;
        }
    } else {
        if (!on_body) {
            /* PPG says no contact: after some time off-body -> deep sleep */
            if ((now_ms - last_contact_ts_ms) > RING_NO_CONTACT_TO_DEEP_SLEEP_MS) {
                new_mode = RING_MODE_DEEP_SLEEP;
            } else {
                /* Recently taken off or contact flaky: idle (keep PPG running) */
                new_mode = RING_MODE_IDLE;
            }
        } else {
            /* On-body: differentiate ACTIVE vs IDLE by motion */
            if (!no_motion_short) {
                new_mode = RING_MODE_ACTIVE;
            } else {
                new_mode = RING_MODE_IDLE;
            }
        }
    }

    if (new_mode != ring_mode) {
        printk("Ring: mode change %s -> %s (on_body=%d, no_motion=%d)\n",
               ring_mode_name(ring_mode),
               ring_mode_name(new_mode),
               on_body ? 1 : 0,
               no_motion_short ? 1 : 0);

        ring_mode = new_mode;
        ring_apply_mode(new_mode);
    }
}

/* ------------------------------------------------------------------------- */
/* main()                                                                    */
/* ------------------------------------------------------------------------- */

int main(void)
{
    int err;

    printk("Ring Firmware Application\n");
    printk("=========================\n\n");

    /* --- MAXM86161 init (PPG + HR/SpO2 + contact) ----------------------- */
    err = maxm86161_app_init();
    if (err) {
        printk("MAXM86161 app init failed: %d\n", err);
        return err;
    }

    /* --- BMA400 init (accel + step + no-motion) ------------------------- */
    err = bma400_app_init();
    if (err) {
        printk("BMA400 app init failed: %d\n", err);
        return err;
    }

    /* Treat BMA400 "sleep flag" as ~30s rest detector instead of 5 min. */
    struct bma400_sleep_params sleep_cfg = {
        .enabled             = true,
        .no_motion_timeout_s = 30U,  /* idle if no motion for 30 s */
    };
    bma400_app_set_sleep_params(&sleep_cfg);

    /* --- MAX30208 init (skin temperature) ------------------------------- */
    err = max30208_app_init();
    if (err) {
        printk("MAX30208 app init failed: %d\n", err);
        return err;
    }

    /* Initial state: assume ACTIVE until evidence otherwise */
    ring_mode            = RING_MODE_ACTIVE;
    last_contact_ts_ms   = k_uptime_get();
    last_temp_sample_ms  = 0;
    last_bma_sample_ms   = 0;
    last_batt_sample_ms  = -RING_BATT_PERIOD_MS;
    last_temp_c          = 0.0f;
    have_temp_sample     = false;
    last_batt_mv         = 0;
    have_batt_sample     = false;

    ring_apply_mode(ring_mode);

    err = ble_app_init();
    if (err) {
        printk("BLE init failed: %d\n", err);
        return err;
    }

    printk("Setup complete. Running main loop...\n");

    while (1) {
        int64_t now = k_uptime_get();

        /* Service PPG FIFO often when not in deep sleep to avoid overflow.
         * This also keeps HR/SpO2/contact updated.
         */
        if (ring_mode != RING_MODE_DEEP_SLEEP) {
            maxm86161_app_print();
        }

        /* Accelerometer sampling cadence depends on mode */
        int32_t bma_period_ms =
            (ring_mode == RING_MODE_DEEP_SLEEP) ?
            RING_DEEP_SLEEP_BMA_PERIOD_MS : RING_BMA_PERIOD_MS;

        if ((now - last_bma_sample_ms) >= bma_period_ms) {
            bma400_app_print();      /* updates no-motion flag + step log */
            last_bma_sample_ms = now;
        }

        /* Always try to read step counter (guarded against failures) */
        uint32_t step_count = 0;
        bma400_step_stat_t step_stat = BMA400_STEP_STAT_STILL;
        int ret = bma400_app_read_step_counter(&step_count, &step_stat);
        if (ret == 0) {
            /* Read succeeded: update last known good values */
            last_step_count = step_count;
            last_step_stat = step_stat;
            printk("BMA400: read success - step_count=%lu, step_stat=%d\n",
                   (unsigned long)step_count, (int)step_stat);
        } else {
            /* Read failed: keep last known values, log the error */
            printk("BMA400: read failed (ret=%d), keeping last known values: step_count=%lu, step_stat=%d\n",
                   ret, (unsigned long)last_step_count, (int)last_step_stat);
        }

        /* Temperature: slower when idle; off in deep sleep */
        if (ring_mode != RING_MODE_DEEP_SLEEP) {
            int32_t temp_period_ms =
                (ring_mode == RING_MODE_ACTIVE) ?
                RING_TEMP_PERIOD_ACTIVE_MS : RING_TEMP_PERIOD_IDLE_MS;

            if ((now - last_temp_sample_ms) >= temp_period_ms) {
                float temp_c = 0.0f;
                err = max30208_read_temperature_c(&temp_c);
                if (err == 0) {
                    last_temp_c = temp_c;
                    have_temp_sample = true;
                    printk("MAX30208: Temperature = %.2f C\n", temp_c);
                } else {
                    printk("MAX30208: measurement error: %d\n", err);
                }
                last_temp_sample_ms = now;
            }
        }

        /* Battery voltage: sample infrequently to save power */
        if ((now - last_batt_sample_ms) >= RING_BATT_PERIOD_MS &&
            ring_mode != RING_MODE_DEEP_SLEEP) {
            uint16_t mv = 0;
            err = ble_app_sample_battery_mv(&mv);
            if (err == 0) {
                last_batt_mv = mv;
                have_batt_sample = true;
            } else {
                printk("Battery sample error: %d\n", err);
            }
            last_batt_sample_ms = now;
        }

        /* Update power state based on latest contact + motion info */
        ring_update_state(now);

        /* Publish latest snapshot over BLE (read + notify if subscribed) */
        struct ring_ble_snapshot snap = {
            .hr_bpm      = maxm86161_get_hr_bpm(),
            .spo2_pct    = maxm86161_get_spo2_pct(),
            .temp_c_x100 = have_temp_sample ?
                           (int16_t)(last_temp_c * 100.0f) : INT16_MIN,
            .battery_mv  = have_batt_sample ? last_batt_mv : 0,
            .ring_mode   = ring_mode,
            .contact     = maxm86161_has_contact(),
            .step_count  = last_step_count,
            .step_state  = (uint8_t)last_step_stat,
        };
        ble_app_publish(&snap);

        /* Let Zephyr drop CPU into idle between samples */
        k_sleep(K_MSEC(RING_MAIN_LOOP_PERIOD_MS));
    }

    return 0;
}
