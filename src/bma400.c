#include "bma400.h"

#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>
#include <zephyr/devicetree.h>
#include <errno.h>

/* Find any node in the DT with compatible "bosch,bma400" */
#define BMA400_NODE DT_COMPAT_GET_ANY_STATUS_OKAY(bosch_bma400)

#if !DT_NODE_HAS_STATUS(BMA400_NODE, okay)
#error "No BMA400 devicetree node found. Add one with compatible = \"bosch,bma400\"."
#endif

static struct bma400_dev bma400_global;
static bool bma400_initialized;

/* Host-side sleep / motion state */
static struct bma400_sleep_params bma400_sleep_cfg = {
    .enabled = true,
    .no_motion_timeout_s = 300, /* default: 5 minutes of no motion */
};

static bool     bma400_no_motion_flag;
static uint32_t bma400_last_step_count;
static int64_t  bma400_last_motion_ts_ms;

/* ------------------------------------------------------------------------- */
/* Low-level helpers                                                        */
/* ------------------------------------------------------------------------- */

int bma400_read_reg(const struct bma400_dev *dev,
                    uint8_t reg, uint8_t *val)
{
    if (!dev || !dev->i2c || !val) {
        return -EINVAL;
    }

    int ret = i2c_reg_read_byte(dev->i2c, dev->i2c_addr, reg, val);
    if (ret < 0) {
        return ret;
    }

    return 0;
}

int bma400_write_reg(const struct bma400_dev *dev,
                     uint8_t reg, uint8_t val)
{
    if (!dev || !dev->i2c) {
        return -EINVAL;
    }

    int ret = i2c_reg_write_byte(dev->i2c, dev->i2c_addr, reg, val);
    if (ret < 0) {
        return ret;
    }

    return 0;
}

/* ------------------------------------------------------------------------- */
/* Core configuration                                                       */
/* ------------------------------------------------------------------------- */

int bma400_app_read_step_counter(uint32_t *step_count,
                                 bma400_step_stat_t *step_stat)
{
    if (!bma400_initialized) {
        return -ENODEV;
    }

    return bma400_read_step_counter(&bma400_global, step_count, step_stat);
}

int bma400_app_set_power_mode(bma400_power_mode_t mode)
{
    if (!bma400_initialized) {
        return -ENODEV;
    }

    uint8_t acc_conf0 = 0;
    int ret = bma400_read_reg(&bma400_global, BMA400_REG_ACC_CONFIG0, &acc_conf0);
    if (ret) {
        return ret;
    }

    acc_conf0 &= ~BMA400_ACC_CONFIG0_POWER_MODE_MASK;
    acc_conf0 |= (uint8_t)mode;

    ret = bma400_write_reg(&bma400_global, BMA400_REG_ACC_CONFIG0, acc_conf0);
    if (ret) {
        return ret;
    }

    printk("BMA400: power mode changed to %s\n",
           (mode == BMA400_POWER_MODE_NORMAL) ? "NORMAL" :
           (mode == BMA400_POWER_MODE_LP)     ? "LP" : "SLEEP");

    return 0;
}


static float bma400_lsb_per_g(bma400_range_t range)
{
    /* From datasheet / product tables: 1024 LSB/g @ 2g, then halves per step. :contentReference[oaicite:5]{index=5} */
    switch (range) {
    case BMA400_RANGE_2G:  return 1024.0f;
    case BMA400_RANGE_4G:  return 512.0f;
    case BMA400_RANGE_8G:  return 256.0f;
    case BMA400_RANGE_16G: return 128.0f;
    default:               return 1024.0f;
    }
}

int bma400_init(struct bma400_dev *dev,
                const struct device *i2c_dev,
                uint16_t i2c_addr,
                bma400_range_t range)
{
    if (!dev || !i2c_dev) {
        return -EINVAL;
    }

    if (!device_is_ready(i2c_dev)) {
        printk("BMA400: I2C device not ready\n");
        return -ENODEV;
    }

    dev->i2c      = i2c_dev;
    dev->i2c_addr = i2c_addr;
    dev->range    = range;

    uint8_t chip_id = 0;
    int ret = bma400_read_reg(dev, BMA400_REG_CHIP_ID, &chip_id);
    if (ret) {
        printk("BMA400: Failed to read CHIP_ID (%d)\n", ret);
        return ret;
    }

    if (chip_id != BMA400_CHIP_ID) {
        printk("BMA400: Unexpected CHIP_ID=0x%02x (expected 0x%02x)\n",
               chip_id, BMA400_CHIP_ID);
        return -EIO;
    }

    /* Put accelerometer into normal mode */
    uint8_t acc_conf0 = 0;
    ret = bma400_read_reg(dev, BMA400_REG_ACC_CONFIG0, &acc_conf0);
    if (ret) {
        return ret;
    }

    acc_conf0 &= ~BMA400_ACC_CONFIG0_POWER_MODE_MASK;
    acc_conf0 |= BMA400_ACC_CONFIG0_POWER_MODE_NORMAL;

    ret = bma400_write_reg(dev, BMA400_REG_ACC_CONFIG0, acc_conf0);
    if (ret) {
        return ret;
    }

    /* ACC_CONFIG1:
     *   - ODR = 100 Hz (acc_odr = 0x8)
     *   - OSR = 3 (osr = 0x3) for best noise
     *   - range as requested
     */
    uint8_t acc_conf1 =
        ((uint8_t)0x8 << 4) |  /* acc_odr */
        ((uint8_t)0x3 << 2) |  /* osr */
        ((uint8_t)range & 0x03);

    ret = bma400_write_reg(dev, BMA400_REG_ACC_CONFIG1, acc_conf1);
    if (ret) {
        return ret;
    }

    /* Leave ACC_CONFIG2 at its reset value (0x00) for now. */

    return 0;
}

/* ------------------------------------------------------------------------- */
/* Raw and scaled data                                                      */
/* ------------------------------------------------------------------------- */

int bma400_read_raw(const struct bma400_dev *dev,
                    int16_t *x, int16_t *y, int16_t *z)
{
    if (!dev || !dev->i2c || !x || !y || !z) {
        return -EINVAL;
    }

    uint8_t buf[6];
    int ret = i2c_burst_read(dev->i2c, dev->i2c_addr,
                             BMA400_REG_ACC_X_LSB, buf, sizeof(buf));
    if (ret < 0) {
        return ret;
    }

    /* 12-bit values, MSB[7:4] + LSB[7:0], 2's complement. */
    int16_t raw_x = (int16_t)((((int16_t)buf[1] << 8) | buf[0]) >> 4);
    int16_t raw_y = (int16_t)((((int16_t)buf[3] << 8) | buf[2]) >> 4);
    int16_t raw_z = (int16_t)((((int16_t)buf[5] << 8) | buf[4]) >> 4);

    /* Sign-extend from 12-bit to 16-bit */
    if (raw_x & 0x0800) raw_x |= 0xF000;
    if (raw_y & 0x0800) raw_y |= 0xF000;
    if (raw_z & 0x0800) raw_z |= 0xF000;

    *x = raw_x;
    *y = raw_y;
    *z = raw_z;

    return 0;
}

int bma400_read_mg(const struct bma400_dev *dev,
                   float *x_mg, float *y_mg, float *z_mg)
{
    if (!dev || !x_mg || !y_mg || !z_mg) {
        return -EINVAL;
    }

    int16_t x_raw, y_raw, z_raw;
    int ret = bma400_read_raw(dev, &x_raw, &y_raw, &z_raw);
    if (ret) {
        return ret;
    }

    float lsb_g = bma400_lsb_per_g(dev->range);
    float scale = 1000.0f / lsb_g; /* mg per LSB */

    *x_mg = (float)x_raw * scale;
    *y_mg = (float)y_raw * scale;
    *z_mg = (float)z_raw * scale;

    return 0;
}

int bma400_read_temperature_c(const struct bma400_dev *dev, float *temp_c)
{
    if (!dev || !temp_c) {
        return -EINVAL;
    }

    uint8_t raw = 0;
    int ret = bma400_read_reg(dev, BMA400_REG_TEMP_DATA, &raw);
    if (ret) {
        return ret;
    }

    /* Rough conversion; your real body temperature comes from MAX30208 anyway. */
    *temp_c = 23.0f + ((int8_t)raw) / 2.0f;

    return 0;
}

/* ------------------------------------------------------------------------- */
/* Step counter / activity                                                  */
/* ------------------------------------------------------------------------- */

int bma400_read_step_counter(const struct bma400_dev *dev,
                             uint32_t *step_count,
                             bma400_step_stat_t *step_stat)
{
    if (!dev || !dev->i2c) {
        return -EINVAL;
    }

    uint8_t buf[4];

    /* STEP_CNT0..2 + STEP_STAT in a single burst to avoid race conditions. :contentReference[oaicite:6]{index=6} */
    int ret = i2c_burst_read(dev->i2c, dev->i2c_addr,
                             BMA400_REG_STEP_CNT0, buf, sizeof(buf));
    if (ret < 0) {
        return ret;
    }

    uint32_t cnt = ((uint32_t)buf[0]) |
                   ((uint32_t)buf[1] << 8) |
                   ((uint32_t)buf[2] << 16);

    if (step_count) {
        *step_count = cnt;
    }

    if (step_stat) {
        uint8_t field = buf[3] & 0x03; /* step_stat_field<1:0> */
        *step_stat = (bma400_step_stat_t)field;
    }

    return 0;
}

/* ------------------------------------------------------------------------- */
/* Application-level helpers                                                */
/* ------------------------------------------------------------------------- */

int bma400_app_read_mg(float *x_mg, float *y_mg, float *z_mg)
{
    if (!bma400_initialized) {
        return -ENODEV;
    }

    return bma400_read_mg(&bma400_global, x_mg, y_mg, z_mg);
}

void bma400_app_set_sleep_params(const struct bma400_sleep_params *params)
{
    if (!params) {
        return;
    }

    bma400_sleep_cfg = *params;
}

bool bma400_app_get_sleep_flag(void)
{
    return bma400_no_motion_flag;
}

int bma400_app_init(void)
{
    const struct device *i2c_dev = DEVICE_DT_GET(DT_BUS(BMA400_NODE));
    uint16_t addr = DT_REG_ADDR(BMA400_NODE);

    int ret = bma400_init(&bma400_global, i2c_dev, addr, BMA400_RANGE_2G);
    if (ret) {
        printk("BMA400: init failed: %d\n", ret);
        return ret;
    }

    /* Enable step counter computation and step interrupt.
     *
     * Datasheet: "The step counter computation is enabled if
     * INT_CONFIG1.step_int = ‘1’." :contentReference[oaicite:7]{index=7}
     */
    uint8_t int_cfg1 = 0;
    ret = bma400_read_reg(&bma400_global, BMA400_REG_INT_CONFIG1, &int_cfg1);
    if (ret == 0) {
        int_cfg1 |= 0x01; /* step_int = 1 */
        ret = bma400_write_reg(&bma400_global, BMA400_REG_INT_CONFIG1, int_cfg1);
    }

    if (ret) {
        printk("BMA400: Failed to enable step counter (%d)\n", ret);
        return ret;
    }

    /* Map the step interrupt to INT1.
     *
     * Register 0x23 INT12_MAP:
     *   bit0 = step_int1, bit1 = tap_int1, bit2 = actch_int1,
     *   bit3 = step_int2, bit4 = tap_int2, bit5 = actch_int2 :contentReference[oaicite:8]{index=8}
     */
    uint8_t int12_map = 0;
    ret = bma400_read_reg(&bma400_global, BMA400_REG_INT12_MAP, &int12_map);
    if (ret == 0) {
        int12_map |= 0x01; /* step_int1 -> INT1 */
        ret = bma400_write_reg(&bma400_global, BMA400_REG_INT12_MAP, int12_map);
    }

    if (ret) {
        printk("BMA400: Failed to map step interrupt to INT1 (%d)\n", ret);
        return ret;
    }

    /* Configure INT1 electrical characteristics:
     *   - active high
     *   - push-pull
     *
     * INT12_IO_CTRL bits:
     *   bit0 = int1_lvl (0: active low, 1: active high)
     *   bit1 = int1_od  (0: push-pull, 1: open-drain)
     *   bit2 = int2_lvl
     *   bit3 = int2_od :contentReference[oaicite:9]{index=9}
     */
    uint8_t io_ctrl = 0;
    ret = bma400_read_reg(&bma400_global, BMA400_REG_INT12_IO_CTRL, &io_ctrl);
    if (ret == 0) {
        io_ctrl &= ~0x03; /* clear int1 bits */
        io_ctrl |= 0x01;  /* active high, push-pull */
        ret = bma400_write_reg(&bma400_global, BMA400_REG_INT12_IO_CTRL, io_ctrl);
    }

    if (ret) {
        printk("BMA400: Failed to configure INT1 IO (%d)\n", ret);
        return ret;
    }

    /* Initialize step/sleep tracking state */
    uint32_t step_count = 0;
    (void)bma400_read_step_counter(&bma400_global, &step_count, NULL);

    bma400_last_step_count   = step_count;
    bma400_last_motion_ts_ms = k_uptime_get();
    bma400_no_motion_flag    = false;

    bma400_initialized = true;

    printk("BMA400: Initialization complete (step + motion support enabled)\n");
    return 0;
}

int bma400_app_print(void)
{
    if (!bma400_initialized) {
        return -ENODEV;
    }

    uint32_t          step_count = 0;
    bma400_step_stat_t step_stat = BMA400_STEP_STAT_STILL;

    int ret = bma400_read_step_counter(&bma400_global,
                                       &step_count,
                                       &step_stat);
    if (ret) {
        printk("BMA400: Failed to read step counter (%d)\n", ret);
        return ret;
    }

    int64_t now_ms = k_uptime_get();

    if (step_count != bma400_last_step_count) {
        /* We saw new steps -> definitely motion */
        bma400_last_step_count   = step_count;
        bma400_last_motion_ts_ms = now_ms;
        bma400_no_motion_flag    = false;
    } else if (bma400_sleep_cfg.enabled &&
               bma400_sleep_cfg.no_motion_timeout_s > 0U) {

        int64_t dt_ms = now_ms - bma400_last_motion_ts_ms;
        if (dt_ms >= (int64_t)bma400_sleep_cfg.no_motion_timeout_s * 1000) {
            bma400_no_motion_flag = true;
        }
    }

    const char *activity_str = "still";
    switch (step_stat) {
    case BMA400_STEP_STAT_WALK: activity_str = "walk"; break;
    case BMA400_STEP_STAT_RUN:  activity_str = "run";  break;
    case BMA400_STEP_STAT_STILL:
    default:
        activity_str = "still";
        break;
    }

    printk("BMA400: steps=%lu, activity=%s, sleep_candidate=%s\n",
           (unsigned long)step_count,
           activity_str,
           bma400_no_motion_flag ? "yes" : "no");

    return 0;
}
