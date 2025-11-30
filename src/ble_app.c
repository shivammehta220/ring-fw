#include "ble_app.h"

#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/sys/util.h>

#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/bluetooth/uuid.h>

/* Standard services */
#include <zephyr/bluetooth/services/hrs.h>
#include <zephyr/bluetooth/services/bas.h>

/* ----------------------------------------------------------------
 * State
 * ---------------------------------------------------------------- */

struct ring_sensor_values {
    int16_t accel_x_mg;
    int16_t accel_y_mg;
    int16_t accel_z_mg;
    float   temp_c;
    uint8_t hr_bpm;
    uint8_t battery_percent;
};

static struct ring_sensor_values sensor_values;

static struct bt_conn *current_conn;

/* Notification/Indication flags for our custom/HTS services */
static bool notify_accel;
static bool hts_indicate_enabled;

/* ----------------------------------------------------------------
 * Custom Ring service for accelerometer (128-bit UUID)
 *
 * Base:  7d3b0001-427b-4c02-9a16-3b6f8e9f0001
 * Accel: 7d3b0002-427b-4c02-9a16-3b6f8e9f0001
 * ---------------------------------------------------------------- */

#define RING_SVC_UUID_VAL \
    BT_UUID_128_ENCODE(0x7d3b0001, 0x427b, 0x4c02, 0x9a16, 0x3b6f8e9f0001)

#define RING_ACCEL_UUID_VAL \
    BT_UUID_128_ENCODE(0x7d3b0002, 0x427b, 0x4c02, 0x9a16, 0x3b6f8e9f0001)

static struct bt_uuid_128 ring_svc_uuid   = BT_UUID_INIT_128(RING_SVC_UUID_VAL);
static struct bt_uuid_128 ring_accel_uuid = BT_UUID_INIT_128(RING_ACCEL_UUID_VAL);

/* ----------------------------------------------------------------
 * Health Thermometer Service (HTS) – standard UUIDs
 *
 * Service:          0x1809 (Health Thermometer)
 * Temp Measurement: 0x2A1C (Temperature Measurement)
 *
 * Temp value encoding:
 *   Flags (1 byte) = 0x00 (Celsius, no timestamp, no temp type)
 *   Temperature    = IEEE-11073 32-bit FLOAT:
 *                    8-bit exponent, 24-bit mantissa, base-10
 *                    value = mantissa * 10^exponent
 *   We use exponent = -2 (0xFE) so mantissa = temp_c * 100.
 * ---------------------------------------------------------------- */

#define HTS_SERVICE_UUID      0x1809
#define HTS_TEMP_MEAS_UUID    0x2A1C

/* Attribute index of the Temperature Measurement value inside hts_svc:
 * 0: Primary Service
 * 1: Characteristic declaration
 * 2: Characteristic value
 * 3: CCC
 */
#define HTS_TEMP_MEAS_ATTR_IDX 2

/* Forward declaration of the HTS GATT service instance */
BT_GATT_SERVICE_DEFINE(hts_svc,
    BT_GATT_PRIMARY_SERVICE(BT_UUID_DECLARE_16(HTS_SERVICE_UUID)),
    BT_GATT_CHARACTERISTIC(BT_UUID_DECLARE_16(HTS_TEMP_MEAS_UUID),
                           BT_GATT_CHRC_INDICATE,
                           0,               /* no direct read */
                           NULL, NULL, NULL),
    BT_GATT_CCC(NULL, BT_GATT_PERM_READ | BT_GATT_PERM_WRITE)
);

/* We’ll override the CCC callback after the define, via a wrapper */
static void hts_ccc_cfg_changed(const struct bt_gatt_attr *attr, uint16_t value);

/* Helper buffer and params for HTS indications */
static uint8_t hts_meas_buf[1 + 4]; /* 1 byte flags + 4 bytes FLOAT */
static struct bt_gatt_indicate_params hts_ind_params;

/* ----------------------------------------------------------------
 * Advertising data
 *
 * We advertise standard 16-bit services so Si Connect demos
 * recognize the device:
 *   0x180D – Heart Rate Service
 *   0x1809 – Health Thermometer
 *   0x180F – Battery Service
 *
 * (The custom Ring accel service is still present in the GATT
 *  database but not advertised to save space.)
 * ---------------------------------------------------------------- */

static const struct bt_data ad[] = {
    BT_DATA_BYTES(BT_DATA_FLAGS, (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR)),
    BT_DATA_BYTES(BT_DATA_UUID16_ALL,
                  BT_UUID_16_ENCODE(0x180D), /* Heart Rate */
                  BT_UUID_16_ENCODE(0x1809), /* Health Thermometer */
                  BT_UUID_16_ENCODE(0x180F)  /* Battery */
    ),
};

static const struct bt_data sd[] = {
    BT_DATA(BT_DATA_NAME_COMPLETE, CONFIG_BT_DEVICE_NAME,
            sizeof(CONFIG_BT_DEVICE_NAME) - 1),
};

/* ----------------------------------------------------------------
 * GATT read handlers for custom Ring service
 *
 * Encoding (little endian):
 *  - accel: int16 X, int16 Y, int16 Z in mg  (6 bytes)
 * ---------------------------------------------------------------- */

static ssize_t read_accel(struct bt_conn *conn,
                          const struct bt_gatt_attr *attr,
                          void *buf, uint16_t len, uint16_t offset)
{
    uint8_t out[6];

    sys_put_le16(sensor_values.accel_x_mg, &out[0]);
    sys_put_le16(sensor_values.accel_y_mg, &out[2]);
    sys_put_le16(sensor_values.accel_z_mg, &out[4]);

    return bt_gatt_attr_read(conn, attr, buf, len, offset, out, sizeof(out));
}

/* ----------------------------------------------------------------
 * CCCD callbacks
 * ---------------------------------------------------------------- */

static void accel_ccc_cfg_changed(const struct bt_gatt_attr *attr,
                                  uint16_t value)
{
    notify_accel = (value == BT_GATT_CCC_NOTIFY);
}

static void hts_ccc_cfg_changed(const struct bt_gatt_attr *attr,
                                uint16_t value)
{
    hts_indicate_enabled = (value == BT_GATT_CCC_INDICATE);
}

/* Re-declare HTS CCC with our callback (struct is already defined) */
BT_GATT_SERVICE_DEFINE(hts_svc_override,
    BT_GATT_PRIMARY_SERVICE(BT_UUID_DECLARE_16(HTS_SERVICE_UUID)),
    BT_GATT_CHARACTERISTIC(BT_UUID_DECLARE_16(HTS_TEMP_MEAS_UUID),
                           BT_GATT_CHRC_INDICATE,
                           0, NULL, NULL, NULL),
    BT_GATT_CCC(hts_ccc_cfg_changed,
                BT_GATT_PERM_READ | BT_GATT_PERM_WRITE)
);

/* ----------------------------------------------------------------
 * Custom Ring GATT service definition (Accel only)
 * ---------------------------------------------------------------- */

enum {
    RING_SVC_ACC_CHAR_ATTR_IDX = 1,
    RING_SVC_ACC_VAL_ATTR_IDX  = 2,
};

BT_GATT_SERVICE_DEFINE(ring_svc,
    BT_GATT_PRIMARY_SERVICE(&ring_svc_uuid),

    /* Accel characteristic: read + notify (6 bytes: X,Y,Z in mg) */
    BT_GATT_CHARACTERISTIC(&ring_accel_uuid.uuid,
                           BT_GATT_CHRC_READ | BT_GATT_CHRC_NOTIFY,
                           BT_GATT_PERM_READ,
                           read_accel, NULL, NULL),
    BT_GATT_CCC(accel_ccc_cfg_changed,
                BT_GATT_PERM_READ | BT_GATT_PERM_WRITE)
);

/* ----------------------------------------------------------------
 * Connection callbacks
 * ---------------------------------------------------------------- */

static void connected(struct bt_conn *conn, uint8_t err)
{
    if (err) {
        printk("BLE: Connection failed (err 0x%02X)\n", err);
        return;
    }

    current_conn = bt_conn_ref(conn);
    printk("BLE: Connected\n");
}

static void disconnected(struct bt_conn *conn, uint8_t reason)
{
    printk("BLE: Disconnected (reason 0x%02X)\n", reason);

    if (current_conn) {
        bt_conn_unref(current_conn);
        current_conn = NULL;
    }
}

BT_CONN_CB_DEFINE(conn_callbacks) = {
    .connected    = connected,
    .disconnected = disconnected,
};

/* ----------------------------------------------------------------
 * Helpers
 * ---------------------------------------------------------------- */

/**
 * Encode temperature in the Health Thermometer FLOAT format:
 *  Flags (byte 0)      : 0x00 (Celsius, no timestamp, no temp type)
 *  FLOAT (bytes 1..4)  : mantissa (24-bit) + exponent (8-bit, base 10)
 *                         value = mantissa * 10^exponent
 * We use exponent = -2 (0xFE) so mantissa = round(temp_c * 100).
 */
static void encode_hts_temp_measurement(float temp_c, uint8_t *buf)
{
    /* Flags */
    buf[0] = 0x00;

    /* Mantissa scaled by 100 */
    float scaled = temp_c * 100.0f;
    int32_t mantissa = (int32_t)((scaled >= 0.0f) ? (scaled + 0.5f) : (scaled - 0.5f));
    int8_t exponent = -2; /* 10^-2 => 0.01°C resolution */

    /* 24-bit signed mantissa, little-endian */
    buf[1] = (uint8_t)(mantissa & 0xFF);
    buf[2] = (uint8_t)((mantissa >> 8) & 0xFF);
    buf[3] = (uint8_t)((mantissa >> 16) & 0xFF);

    /* Exponent as signed 8-bit */
    buf[4] = (uint8_t)exponent;
}

/* Send a Health Thermometer indication if enabled */
static void hts_send_measurement(void)
{
    if (!current_conn || !hts_indicate_enabled) {
        return;
    }

    encode_hts_temp_measurement(sensor_values.temp_c, hts_meas_buf);

    hts_ind_params.attr = &hts_svc_override.attrs[HTS_TEMP_MEAS_ATTR_IDX];
    hts_ind_params.func = NULL;
    hts_ind_params.data = hts_meas_buf;
    hts_ind_params.len  = sizeof(hts_meas_buf);

    int err = bt_gatt_indicate(current_conn, &hts_ind_params);
    if (err) {
        printk("HTS: indicate failed (err %d)\n", err);
    }
}

/* ----------------------------------------------------------------
 * Public API
 * ---------------------------------------------------------------- */

int ble_app_init(void)
{
    int err;

    err = bt_enable(NULL);
    if (err) {
        printk("BLE: bt_enable failed (err %d)\n", err);
        return err;
    }

    printk("BLE: Stack initialized, starting advertising\n");

    /* Set an initial (fake) battery level so BAS has a value */
    bt_bas_set_battery_level(100U);

    /* Try with struct approach - initialize all fields */
    struct bt_le_adv_param adv_param;
    
    adv_param.id = BT_ID_DEFAULT;
    adv_param.sid = 0;
    adv_param.secondary_max_skip = 0;
    adv_param.options = BT_LE_ADV_OPT_CONNECTABLE;
    adv_param.interval_min = BT_GAP_ADV_FAST_INT_MIN_2;
    adv_param.interval_max = BT_GAP_ADV_FAST_INT_MAX_2;
    adv_param.peer = NULL;

    err = bt_le_adv_start(&adv_param, ad, ARRAY_SIZE(ad), sd, ARRAY_SIZE(sd));
    if (err) {
        printk("BLE: Advertising start failed (err %d)\n", err);
        return err;
    }

    printk("BLE: Advertising as \"%s\"\n", CONFIG_BT_DEVICE_NAME);

    return 0;
}

/**
 * Update the latest sensor values and push them out over BLE using:
 *   - Custom Ring accel service (128-bit UUID)     → notify
 *   - Heart Rate Service (0x180D)                  → bt_hrs_notify()
 *   - Health Thermometer Service (0x1809)          → indicate (0x2A1C)
 *   - Battery Service (0x180F)                     → bt_bas_set_battery_level()
 */
void ble_app_update_sensors(int16_t accel_x_mg,
                            int16_t accel_y_mg,
                            int16_t accel_z_mg,
                            float   temp_c,
                            int     hr_bpm,
                            uint8_t battery_percent)
{
    /* Clamp/sanitize inputs and store */
    sensor_values.accel_x_mg = accel_x_mg;
    sensor_values.accel_y_mg = accel_y_mg;
    sensor_values.accel_z_mg = accel_z_mg;
    sensor_values.temp_c     = temp_c;

    if (hr_bpm < 0) {
        hr_bpm = 0;
    } else if (hr_bpm > 255) {
        hr_bpm = 255;
    }
    sensor_values.hr_bpm = (uint8_t)hr_bpm;

    if (battery_percent > 100U) {
        battery_percent = 100U;
    }
    sensor_values.battery_percent = battery_percent;

    if (!current_conn) {
        return; /* No one connected, nothing to notify */
    }

    /* ------------------------------------------------------------
     * 1) Accel via custom Ring service (same as before)
     * ---------------------------------------------------------- */
    if (notify_accel) {
        uint8_t payload[6];

        sys_put_le16(sensor_values.accel_x_mg, &payload[0]);
        sys_put_le16(sensor_values.accel_y_mg, &payload[2]);
        sys_put_le16(sensor_values.accel_z_mg, &payload[4]);

        (void)bt_gatt_notify(current_conn,
                             &ring_svc.attrs[RING_SVC_ACC_VAL_ATTR_IDX],
                             payload,
                             sizeof(payload));
    }

    /* ------------------------------------------------------------
     * 2) Heart Rate Service – standard 0x180D / 0x2A37
     *
     * bt_hrs_notify() handles the Heart Rate Measurement format
     * and only sends if the client has enabled notifications.
     * ---------------------------------------------------------- */
    (void)bt_hrs_notify(sensor_values.hr_bpm);

    /* ------------------------------------------------------------
     * 3) Health Thermometer Service – standard 0x1809 / 0x2A1C
     *
     * We encode temp in IEEE-11073 FLOAT and send an indication.
     * ---------------------------------------------------------- */
    hts_send_measurement();

    /* ------------------------------------------------------------
     * 4) Battery Service – standard 0x180F / 0x2A19
     *
     * bt_bas_set_battery_level() updates the characteristic and
     * notifies subscribers if they've enabled notifications.
     * ---------------------------------------------------------- */
    (void)bt_bas_set_battery_level(sensor_values.battery_percent);
}
