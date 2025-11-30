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

/* ---------- low-level helpers (unchanged) ---------- */

static int16_t bma400_convert_12bit(uint8_t lsb, uint8_t msb)
{
    /* Combine 8 LSB and 4 MSB into a signed 12-bit value. */
    int16_t v = (int16_t)(((uint16_t)msb << 8) | lsb);

    if (v > 2047) {
        v -= 4096;
    }

    return v;
}

static float bma400_lsb_per_g(bma400_range_t range)
{
    switch (range) {
    case BMA400_RANGE_2G:
        return 1024.0f;
    case BMA400_RANGE_4G:
        return 512.0f;
    case BMA400_RANGE_8G:
        return 256.0f;
    case BMA400_RANGE_16G:
        return 128.0f;
    default:
        return 1024.0f;
    }
}

int bma400_read_reg(const struct bma400_dev *dev,
                    uint8_t reg, uint8_t *val)
{
    if (!dev || !dev->i2c || !val) {
        return -EINVAL;
    }

    return i2c_write_read(dev->i2c, dev->i2c_addr,
                          &reg, 1, val, 1);
}

int bma400_write_reg(const struct bma400_dev *dev,
                     uint8_t reg, uint8_t val)
{
    if (!dev || !dev->i2c) {
        return -EINVAL;
    }

    uint8_t buf[2] = { reg, val };

    return i2c_write(dev->i2c, buf, sizeof(buf), dev->i2c_addr);
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
        return -ENODEV;
    }

    dev->i2c      = i2c_dev;
    dev->i2c_addr = i2c_addr;
    dev->range    = range;

    int ret;
    uint8_t chip_id;

    /* Check chip ID (datasheet: 0x90) */
    ret = bma400_read_reg(dev, BMA400_REG_CHIP_ID, &chip_id);
    if (ret) {
        return ret;
    }

    if (chip_id != BMA400_CHIP_ID) {
        printk("BMA400: unexpected chip ID 0x%02X (expected 0x%02X)\n",
               chip_id, BMA400_CHIP_ID);
        return -ENODEV;
    }

    /* Switch from sleep (power-up default) to normal mode. */
    ret = bma400_write_reg(dev, BMA400_REG_ACC_CONFIG0,
                           BMA400_ACC_CONFIG0_POWER_MODE_NORMAL);
    if (ret) {
        return ret;
    }

    /* tstartup_normal ~ 1.5 ms */
    k_sleep(K_MSEC(2));

    /* ACC_CONFIG1:
     *   [7:6] acc_range  (0=+-2g, 1=+-4g, 2=+-8g, 3=+-16g)
     *   [5:4] osr        (3 = high performance)
     *   [3:0] acc_odr    (0x8 = 100 Hz)
     */
    uint8_t range_bits = ((uint8_t)range) & 0x03;
    uint8_t osr_bits   = 0x3;   /* high performance */
    uint8_t odr_bits   = 0x8;   /* 100 Hz */

    uint8_t cfg1 = (uint8_t)((range_bits << 6) |
                             (osr_bits   << 4) |
                             (odr_bits & 0x0F));

    ret = bma400_write_reg(dev, BMA400_REG_ACC_CONFIG1, cfg1);
    if (ret) {
        return ret;
    }

    /* ACC_CONFIG2: data_src_reg
     * 0x04 => use acc_filt2 (100 Hz) as data source.
     */
    ret = bma400_write_reg(dev, BMA400_REG_ACC_CONFIG2, 0x04);
    if (ret) {
        return ret;
    }

    return 0;
}

int bma400_read_raw(const struct bma400_dev *dev,
                    int16_t *x, int16_t *y, int16_t *z)
{
    if (!dev || !dev->i2c || !x || !y || !z) {
        return -EINVAL;
    }

    uint8_t buf[6];
    int ret = i2c_burst_read(dev->i2c, dev->i2c_addr,
                             BMA400_REG_ACC_X_LSB,
                             buf, sizeof(buf));
    if (ret) {
        return ret;
    }

    *x = bma400_convert_12bit(buf[0], buf[1]);
    *y = bma400_convert_12bit(buf[2], buf[3]);
    *z = bma400_convert_12bit(buf[4], buf[5]);

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

    uint8_t raw;
    int ret = bma400_read_reg(dev, BMA400_REG_TEMP_DATA, &raw);
    if (ret) {
        return ret;
    }

    /* temp = ((int8_t)temp_data) * 0.5 + 23.0 */
    int8_t sraw = (int8_t)raw;
    *temp_c = (float)sraw * 0.5f + 23.0f;

    return 0;
}

/* ---------- app-level helpers ---------- */

int bma400_app_init(void)
{
    const struct device *i2c_dev = DEVICE_DT_GET(DT_BUS(BMA400_NODE));

    if (!device_is_ready(i2c_dev)) {
        printk("BMA400: I2C bus %s not ready\n", i2c_dev->name);
        return -ENODEV;
    }

    uint16_t addr = DT_REG_ADDR(BMA400_NODE);

    int ret = bma400_init(&bma400_global, i2c_dev, addr, BMA400_RANGE_2G);
    if (ret) {
        printk("BMA400: init failed (addr 0x%02X): %d\n", addr, ret);
        return ret;
    }

    printk("BMA400: init OK on %s @ 0x%02X\n", i2c_dev->name, addr);
    return 0;
}

int bma400_app_print(void)
{
    float x_mg, y_mg, z_mg;
    int ret = bma400_read_mg(&bma400_global, &x_mg, &y_mg, &z_mg);
    if (ret) {
        printk("BMA400 accel read error: %d\n", ret);
        return ret;
    }

    printk("BMA400 accel: X=%.1f mg  Y=%.1f mg  Z=%.1f mg\n",
           x_mg, y_mg, z_mg);

    return 0;
}

int bma400_app_read_mg(float *x_mg, float *y_mg, float *z_mg)
{
    return bma400_read_mg(&bma400_global, x_mg, y_mg, z_mg);
}
