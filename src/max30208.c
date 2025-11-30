#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/sys/printk.h>
#include <errno.h>
#include <stdbool.h>

#include "max30208.h"

/* 7-bit I2C address – from your devicetree: reg = <0x50>; */
#define MAX30208_I2C_ADDR              0x50

/* Register map (from datasheet) */
#define MAX30208_REG_STATUS            0x00
#define MAX30208_REG_INT_ENABLE        0x01

#define MAX30208_REG_FIFO_WRITE_PTR    0x04
#define MAX30208_REG_FIFO_READ_PTR     0x05
#define MAX30208_REG_FIFO_OVERFLOW     0x06
#define MAX30208_REG_FIFO_DATA_COUNT   0x07
#define MAX30208_REG_FIFO_DATA         0x08
#define MAX30208_REG_FIFO_CONFIG1      0x09
#define MAX30208_REG_FIFO_CONFIG2      0x0A

#define MAX30208_REG_SYSTEM_CONTROL    0x0C

#define MAX30208_REG_ALARM_HI_MSB      0x10
#define MAX30208_REG_ALARM_HI_LSB      0x11
#define MAX30208_REG_ALARM_LO_MSB      0x12
#define MAX30208_REG_ALARM_LO_LSB      0x13
#define MAX30208_REG_TEMP_SETUP        0x14

#define MAX30208_REG_GPIO_SETUP        0x20
#define MAX30208_REG_GPIO_CONTROL      0x21

#define MAX30208_REG_PART_ID1          0x31
#define MAX30208_REG_PART_ID2          0x32
#define MAX30208_REG_PART_ID3          0x33
#define MAX30208_REG_PART_ID4          0x34
#define MAX30208_REG_PART_ID5          0x35
#define MAX30208_REG_PART_ID6          0x36
#define MAX30208_REG_PART_IDENTIFIER   0xFF

/* STATUS bits */
#define MAX30208_STATUS_TEMP_RDY       BIT(0)

/* TEMP_SENSOR_SETUP bits */
#define MAX30208_TEMP_SETUP_CONVERT_T  BIT(0)
#define MAX30208_TEMP_SETUP_RFU_MASK   (BIT(7) | BIT(6))  /* must be 1s */

/* FIFO_CONFIG2 bits */
#define MAX30208_FIFO_CFG2_FLUSH_FIFO  BIT(4)
#define MAX30208_FIFO_CFG2_STAT_CLR    BIT(3)

/* SYSTEM_CONTROL bits */
#define MAX30208_SYSCTL_RESET          BIT(0)

/* Internal state */
static const struct device *max30208_i2c_dev;
static bool max30208_initialized;

/* Simple retry settings */
#define MAX30208_I2C_RETRIES   3
#define MAX30208_I2C_DELAY_MS  1

/* Small helpers with retry logic */
static int max30208_i2c_write_byte(uint8_t reg, uint8_t value)
{
    int err;

    if (!max30208_i2c_dev) {
        return -ENODEV;
    }

    for (int i = 0; i < MAX30208_I2C_RETRIES; i++) {
        err = i2c_reg_write_byte(max30208_i2c_dev,
                                 MAX30208_I2C_ADDR,
                                 reg,
                                 value);
        if (!err) {
            return 0;
        }
        k_msleep(MAX30208_I2C_DELAY_MS);
    }

    return err;
}

static int max30208_i2c_read_byte(uint8_t reg, uint8_t *value)
{
    int err;

    if (!max30208_i2c_dev) {
        return -ENODEV;
    }

    for (int i = 0; i < MAX30208_I2C_RETRIES; i++) {
        err = i2c_reg_read_byte(max30208_i2c_dev,
                                MAX30208_I2C_ADDR,
                                reg,
                                value);
        if (!err) {
            return 0;
        }
        k_msleep(MAX30208_I2C_DELAY_MS);
    }

    return err;
}

static int max30208_i2c_burst_read(uint8_t reg, uint8_t *buf, size_t len)
{
    int err;

    if (!max30208_i2c_dev) {
        return -ENODEV;
    }

    for (int i = 0; i < MAX30208_I2C_RETRIES; i++) {
        err = i2c_burst_read(max30208_i2c_dev,
                             MAX30208_I2C_ADDR,
                             reg,
                             buf,
                             len);
        if (!err) {
            return 0;
        }
        k_msleep(MAX30208_I2C_DELAY_MS);
    }

    return err;
}

int max30208_app_init(void)
{
    /* Try a few common I2C bus names; adjust if needed for your board */
    static const char * const bus_candidates[] = {
        "I2C_0",
        "i2c0",
        "I2C0",
        "i2c@40003000",
        NULL
    };

    for (int i = 0; bus_candidates[i] != NULL; ++i) {
        max30208_i2c_dev = device_get_binding(bus_candidates[i]);
        if (max30208_i2c_dev != NULL) {
            break;
        }
    }

    if (max30208_i2c_dev == NULL) {
        printk("MAX30208: failed to find I2C bus\n");
        return -ENODEV;
    }

    /* Optional soft reset */
    int err = max30208_i2c_write_byte(MAX30208_REG_SYSTEM_CONTROL,
                                      MAX30208_SYSCTL_RESET);
    if (err) {
        printk("MAX30208: SYSTEM_CONTROL reset write failed: %d\n", err);
        return err;
    }

    /* Short delay for reset to complete */
    k_msleep(2);

    /* Flush FIFO and configure TEMP_RDY clearing on data read */
    uint8_t fifo_cfg2 = MAX30208_FIFO_CFG2_FLUSH_FIFO |
                        MAX30208_FIFO_CFG2_STAT_CLR;
    err = max30208_i2c_write_byte(MAX30208_REG_FIFO_CONFIG2, fifo_cfg2);
    if (err) {
        printk("MAX30208: FIFO_CONFIG2 write failed: %d\n", err);
        return err;
    }

    /* Read PART_ID (0xFF should be 0x30 for MAX30208) */
    uint8_t part_id = 0;
    err = max30208_i2c_read_byte(MAX30208_REG_PART_IDENTIFIER, &part_id);
    if (err) {
        printk("MAX30208: PART_ID read failed: %d\n", err);
        return err;
    }

    if (part_id != 0x30) {
        printk("MAX30208: unexpected PART_ID=0x%02X (expected 0x30)\n", part_id);
        return -EINVAL;
    }

    printk("MAX30208: init OK on %s @ 0x%02X (PART_ID=0x%02X)\n",
           max30208_i2c_dev->name, MAX30208_I2C_ADDR, part_id);

    max30208_initialized = true;
    return 0;
}

int max30208_read_temperature_c(float *temp_c)
{
    if (!max30208_initialized) {
        return -EIO;
    }
    if (temp_c == NULL) {
        return -EINVAL;
    }

    int err;

    /* Start a single-shot temperature conversion
     * TEMP_SENSOR_SETUP:
     *   bits 7:6 = 1 (RFU, must be 1)
     *   bit 0   = 1 (CONVERT_T)
     */
    uint8_t temp_setup_val = MAX30208_TEMP_SETUP_RFU_MASK |
                             MAX30208_TEMP_SETUP_CONVERT_T;
    err = max30208_i2c_write_byte(MAX30208_REG_TEMP_SETUP, temp_setup_val);
    if (err) {
        printk("MAX30208: failed to start conversion: %d\n", err);
        return err;
    }

    /* Wait for STATUS.TEMP_RDY = 1 (bit 0) */
    const int timeout_ms = 200;   /* Slightly longer timeout */
    uint8_t status = 0;
    int waited_ms = 0;

    while (waited_ms < timeout_ms) {
        err = max30208_i2c_read_byte(MAX30208_REG_STATUS, &status);
        if (err) {
            /* Treat occasional read failures as transient; keep trying */
            printk("MAX30208: STATUS read transient error: %d\n", err);
            k_msleep(1);
            waited_ms++;
            continue;
        }

        if (status & MAX30208_STATUS_TEMP_RDY) {
            break;
        }

        k_msleep(1);
        waited_ms++;
    }

    if (!(status & MAX30208_STATUS_TEMP_RDY)) {
        printk("MAX30208: TEMP_RDY timeout (status=0x%02X)\n", status);
        return -ETIMEDOUT;
    }

    /* Read one 16-bit sample from FIFO_DATA (0x08): MSB then LSB */
    uint8_t buf[2] = {0};
    err = max30208_i2c_burst_read(MAX30208_REG_FIFO_DATA, buf, sizeof(buf));
    if (err) {
        printk("MAX30208: FIFO_DATA read failed: %d\n", err);
        return err;
    }

    int16_t raw = (int16_t)((buf[0] << 8) | buf[1]);

    /* In your setup, 0 is almost certainly invalid; treat as error */
    if (raw == 0) {
        printk("MAX30208: got raw=0 sample, treating as invalid\n");
        return -EIO;
    }

    /* Data is two's complement, 16-bit, scale 0.005 °C/LSB */
    *temp_c = (float)raw * 0.005f;

    return 0;
}

void max30208_app_print(void)
{
    if (!max30208_initialized) {
        printk("MAX30208: not initialized\n");
        return;
    }

    float temp_c;
    int err = max30208_read_temperature_c(&temp_c);
    if (err) {
        printk("MAX30208: measurement error: %d\n", err);
        return;
    }

    printk("MAX30208: Temperature = %.2f C\n", temp_c);
}
