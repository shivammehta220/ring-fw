/*
 * Minimal BLE layer for the ring firmware.
 *
 * - Single custom service with one characteristic carrying a packed snapshot.
 * - Supports read + notify to keep ATT footprint small.
 * - Also updates the standard Battery Service percentage for interoperability.
 */

#include <zephyr/kernel.h>
#include <zephyr/types.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/sys/printk.h>
#include <zephyr/sys/util.h>
#include <zephyr/logging/log.h>
#include <errno.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/bluetooth/hci.h>
#include <zephyr/drivers/adc.h>
#include <string.h>

#include "ble_app.h"

LOG_MODULE_REGISTER(ble_app, LOG_LEVEL_INF);

/* ------------------------------------------------------------------------- */
/* UUIDs                                                                     */
/* ------------------------------------------------------------------------- */

/* Random, locally generated 128-bit UUIDs */
#define BT_UUID_RING_SERVICE_VAL \
    BT_UUID_128_ENCODE(0xd8b8a466, 0x9f41, 0x4a4e, 0x9740, 0x20a1f2b6c8f5)
#define BT_UUID_RING_DATA_VAL \
    BT_UUID_128_ENCODE(0xb8fd8a95, 0xf86f, 0x4b26, 0x9a9d, 0x7e9b9f3d3dd6)
#define BT_UUID_RING_MOTION_VAL \
    BT_UUID_128_ENCODE(0x6e2c8b9c, 0xe0e1, 0x49c9, 0x9c2a, 0x4b0e8a0f3c77)

static struct bt_uuid_128 ring_service_uuid = BT_UUID_INIT_128(BT_UUID_RING_SERVICE_VAL);
static struct bt_uuid_128 ring_data_uuid    = BT_UUID_INIT_128(BT_UUID_RING_DATA_VAL);
static struct bt_uuid_128 ring_motion_uuid  = BT_UUID_INIT_128(BT_UUID_RING_MOTION_VAL);

/* ------------------------------------------------------------------------- */
/* Battery ADC                                                               */
/* ------------------------------------------------------------------------- */

#if DT_NODE_HAS_STATUS(DT_NODELABEL(vbatt), okay)
static const struct adc_dt_spec batt_adc = ADC_DT_SPEC_GET(DT_NODELABEL(vbatt));
#endif

/* ------------------------------------------------------------------------- */
/* GATT service + state                                                      */
/* ------------------------------------------------------------------------- */

struct __packed ring_payload_meas {
    int16_t hr_bpm;
    int16_t spo2_pct;
    int16_t temp_c_x100;
    uint16_t battery_mv;
    uint8_t ring_mode;
    uint8_t contact;
};

struct __packed ring_payload_motion {
    uint32_t step_count;
    uint8_t  step_state;
    uint8_t  reserved[3];
};

static struct ring_payload_meas   g_payload_meas;
static struct ring_payload_motion g_payload_motion;
static bool notify_enabled_meas;
static bool notify_enabled_motion;
static struct bt_conn *current_conn;

static ssize_t ring_data_read(struct bt_conn *conn,
                              const struct bt_gatt_attr *attr,
                              void *buf, uint16_t len, uint16_t offset)
{
    const void *value = attr->user_data;
    return bt_gatt_attr_read(conn, attr, buf, len, offset, value,
                             sizeof(struct ring_payload_meas));
}

static void ring_ccc_cfg_changed(const struct bt_gatt_attr *attr, uint16_t value)
{
    notify_enabled_meas = (value == BT_GATT_CCC_NOTIFY);
    LOG_INF("BLE meas notify %s", notify_enabled_meas ? "enabled" : "disabled");
}

static ssize_t ring_motion_read(struct bt_conn *conn,
                                const struct bt_gatt_attr *attr,
                                void *buf, uint16_t len, uint16_t offset)
{
    const void *value = attr->user_data;
    return bt_gatt_attr_read(conn, attr, buf, len, offset, value,
                             sizeof(struct ring_payload_motion));
}

static void ring_motion_ccc_cfg_changed(const struct bt_gatt_attr *attr, uint16_t value)
{
    notify_enabled_motion = (value == BT_GATT_CCC_NOTIFY);
    LOG_INF("BLE motion notify %s", notify_enabled_motion ? "enabled" : "disabled");
}

BT_GATT_SERVICE_DEFINE(ring_svc,
    BT_GATT_PRIMARY_SERVICE(&ring_service_uuid),
    BT_GATT_CHARACTERISTIC(&ring_data_uuid.uuid,
                           BT_GATT_CHRC_READ | BT_GATT_CHRC_NOTIFY,
                           BT_GATT_PERM_READ,
                           ring_data_read, NULL, &g_payload_meas),
    BT_GATT_CCC(ring_ccc_cfg_changed,
                BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),

    BT_GATT_CHARACTERISTIC(&ring_motion_uuid.uuid,
                           BT_GATT_CHRC_READ | BT_GATT_CHRC_NOTIFY,
                           BT_GATT_PERM_READ,
                           ring_motion_read, NULL, &g_payload_motion),
    BT_GATT_CCC(ring_motion_ccc_cfg_changed,
                BT_GATT_PERM_READ | BT_GATT_PERM_WRITE)
);

#define RING_DATA_ATTR   (&ring_svc.attrs[2])
#define RING_MOTION_ATTR (&ring_svc.attrs[5])

/* ------------------------------------------------------------------------- */
/* Connection callbacks                                                      */
/* ------------------------------------------------------------------------- */

static void connected(struct bt_conn *conn, uint8_t err)
{
    if (err) {
        LOG_ERR("BLE connect failed (err %u)", err);
        return;
    }

    current_conn = bt_conn_ref(conn);
    LOG_INF("BLE connected");
}

static void disconnected(struct bt_conn *conn, uint8_t reason)
{
    ARG_UNUSED(conn);
    LOG_INF("BLE disconnected (reason 0x%02X)", reason);

    if (current_conn) {
        bt_conn_unref(current_conn);
        current_conn = NULL;
    }
}

static struct bt_conn_cb conn_callbacks = {
    .connected = connected,
    .disconnected = disconnected,
};

/* ------------------------------------------------------------------------- */
/* Helpers                                                                   */
/* ------------------------------------------------------------------------- */

#define ADV_INT_MIN  BT_GAP_ADV_FAST_INT_MIN_2
#define ADV_INT_MAX  BT_GAP_ADV_FAST_INT_MAX_2

/* ------------------------------------------------------------------------- */
/* Public API                                                                */
/* ------------------------------------------------------------------------- */

bool ble_app_is_connected(void)
{
    return current_conn != NULL;
}

int ble_app_sample_battery_mv(uint16_t *mv)
{
#if !DT_NODE_HAS_STATUS(DT_NODELABEL(vbatt), okay)
    ARG_UNUSED(mv);
    return -ENODEV;
#else
    if (mv == NULL) {
        return -EINVAL;
    }
    if (!device_is_ready(batt_adc.dev)) {
        LOG_ERR("Battery ADC device not ready");
        return -ENODEV;
    }

    int16_t raw_sample = 0;
    struct adc_sequence sequence = {
        .buffer = &raw_sample,
        .buffer_size = sizeof(raw_sample),
    };

    int err = adc_sequence_init_dt(&batt_adc, &sequence);
    if (err) {
        LOG_ERR("adc_sequence_init_dt failed: %d", err);
        return err;
    }

    err = adc_read(batt_adc.dev, &sequence);
    if (err) {
        LOG_ERR("adc_read failed: %d", err);
        return err;
    }

    int32_t batt_mv = raw_sample;
    err = adc_raw_to_millivolts_dt(&batt_adc, &batt_mv);
    if (err) {
        LOG_ERR("adc_raw_to_millivolts_dt failed: %d", err);
        return err;
    }

    /* Input is VDD/4, so scale back up to full VDD. */
    batt_mv *= 4;

    if (batt_mv < 0) {
        batt_mv = 0;
    }
    *mv = (uint16_t)MIN((int32_t)UINT16_MAX, batt_mv);
    return 0;
#endif
}

int ble_app_init(void)
{
    int err = bt_enable(NULL);
    if (err) {
        LOG_ERR("bt_enable failed: %d", err);
        return err;
    }

    bt_conn_cb_register(&conn_callbacks);

    const struct bt_data ad[] = {
        BT_DATA_BYTES(BT_DATA_FLAGS, (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR)),
        BT_DATA_BYTES(BT_DATA_UUID128_ALL, BT_UUID_RING_SERVICE_VAL),
    };

    const struct bt_le_adv_param adv_param = {
        .id = BT_ID_DEFAULT,
        .sid = 0,
        .interval_min = ADV_INT_MIN,
        .interval_max = ADV_INT_MAX,
        .options = BT_LE_ADV_OPT_CONNECTABLE | BT_LE_ADV_OPT_USE_NAME,
    };

    err = bt_le_adv_start(&adv_param, ad, ARRAY_SIZE(ad), NULL, 0);
    if (err) {
        LOG_ERR("Advertising start failed: %d", err);
        return err;
    }

    LOG_INF("BLE initialized, advertising as \"%s\"",
            CONFIG_BT_DEVICE_NAME);

    return 0;
}

void ble_app_publish(const struct ring_ble_snapshot *snapshot)
{
    if (snapshot == NULL) {
        return;
    }

    struct ring_payload_meas new_payload = {
        .hr_bpm      = sys_cpu_to_le16(snapshot->hr_bpm),
        .spo2_pct    = sys_cpu_to_le16(snapshot->spo2_pct),
        .temp_c_x100 = sys_cpu_to_le16(snapshot->temp_c_x100),
        .battery_mv  = sys_cpu_to_le16(snapshot->battery_mv),
        .ring_mode   = (uint8_t)snapshot->ring_mode,
        .contact     = snapshot->contact ? 1U : 0U,
    };

    struct ring_payload_motion new_motion = {
        .step_count = sys_cpu_to_le32(snapshot->step_count),
        .step_state = snapshot->step_state,
        .reserved   = {0},
    };

    bool meas_changed   = memcmp(&g_payload_meas, &new_payload, sizeof(g_payload_meas)) != 0;
    bool motion_changed = memcmp(&g_payload_motion, &new_motion, sizeof(g_payload_motion)) != 0;
    g_payload_meas      = new_payload;
    g_payload_motion    = new_motion;

    if (current_conn == NULL) {
        return;
    }

    if (meas_changed && notify_enabled_meas) {
        int err = bt_gatt_notify(current_conn,
                                 RING_DATA_ATTR,
                                 &g_payload_meas,
                                 sizeof(g_payload_meas));
        if (err) {
            LOG_WRN("bt_gatt_notify meas failed: %d", err);
        }
    }

    if (motion_changed && notify_enabled_motion) {
        int err = bt_gatt_notify(current_conn,
                                 RING_MOTION_ATTR,
                                 &g_payload_motion,
                                 sizeof(g_payload_motion));
        if (err) {
            LOG_WRN("bt_gatt_notify motion failed: %d", err);
        }
    }
}
