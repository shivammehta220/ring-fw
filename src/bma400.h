#ifndef BMA400_H_
#define BMA400_H_

#include <zephyr/device.h>
#include <zephyr/drivers/i2c.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Default I2C address for Accel 5 Click (ADDR jumper = 1) */
#define BMA400_I2C_ADDR_DEFAULT 0x15

/* Register addresses */
#define BMA400_REG_CHIP_ID      0x00
#define BMA400_CHIP_ID          0x90

#define BMA400_REG_STATUS       0x03

#define BMA400_REG_ACC_X_LSB    0x04
#define BMA400_REG_ACC_X_MSB    0x05
#define BMA400_REG_ACC_Y_LSB    0x06
#define BMA400_REG_ACC_Y_MSB    0x07
#define BMA400_REG_ACC_Z_LSB    0x08
#define BMA400_REG_ACC_Z_MSB    0x09

#define BMA400_REG_TEMP_DATA    0x11

#define BMA400_REG_ACC_CONFIG0  0x19
#define BMA400_REG_ACC_CONFIG1  0x1A
#define BMA400_REG_ACC_CONFIG2  0x1B

#define BMA400_REG_INT_CONFIG0  0x1F
#define BMA400_REG_INT_CONFIG1  0x20
#define BMA400_REG_INT1_MAP     0x21

/* ACC_CONFIG0: power_mode<1:0> */
#define BMA400_ACC_CONFIG0_POWER_MODE_MASK    0x03
#define BMA400_ACC_CONFIG0_POWER_MODE_SLEEP   0x00
#define BMA400_ACC_CONFIG0_POWER_MODE_LP      0x01
#define BMA400_ACC_CONFIG0_POWER_MODE_NORMAL  0x02

typedef enum {
    BMA400_RANGE_2G  = 0,
    BMA400_RANGE_4G  = 1,
    BMA400_RANGE_8G  = 2,
    BMA400_RANGE_16G = 3,
} bma400_range_t;

struct bma400_dev {
    const struct device *i2c;
    uint16_t i2c_addr;
    bma400_range_t range;
};

/**
 * Low-level driver API
 */
int bma400_init(struct bma400_dev *dev,
                const struct device *i2c_dev,
                uint16_t i2c_addr,
                bma400_range_t range);

int bma400_read_raw(const struct bma400_dev *dev,
                    int16_t *x, int16_t *y, int16_t *z);

int bma400_read_mg(const struct bma400_dev *dev,
                   float *x_mg, float *y_mg, float *z_mg);

int bma400_read_temperature_c(const struct bma400_dev *dev, float *temp_c);

int bma400_app_read_mg(float *x_mg, float *y_mg, float *z_mg);

int bma400_read_reg(const struct bma400_dev *dev,
                    uint8_t reg, uint8_t *val);

int bma400_write_reg(const struct bma400_dev *dev,
                     uint8_t reg, uint8_t val);



/**
 * App-level helpers so main.c stays tiny.
 *
 * bma400_app_init():
 *   - finds the devicetree node "bosch,bma400"
 *   - gets its I2C bus + reg address
 *   - calls bma400_init() internally
 *
 * bma400_app_print():
 *   - reads accel in mg
 *   - prints to console
 */
int bma400_app_init(void);
int bma400_app_print(void);

#ifdef __cplusplus
}
#endif

#endif /* BMA400_H_ */
