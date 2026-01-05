#ifndef BMA400_H_
#define BMA400_H_

#include <zephyr/device.h>
#include <zephyr/drivers/i2c.h>
#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Default I2C address for Accel 5 Click (ADDR jumper = 1) */
#define BMA400_I2C_ADDR_DEFAULT 0x15

/* Register addresses */
#define BMA400_REG_CHIP_ID          0x00
#define BMA400_CHIP_ID              0x90

#define BMA400_REG_STATUS           0x03

#define BMA400_REG_ACC_X_LSB        0x04
#define BMA400_REG_ACC_X_MSB        0x05
#define BMA400_REG_ACC_Y_LSB        0x06
#define BMA400_REG_ACC_Y_MSB        0x07
#define BMA400_REG_ACC_Z_LSB        0x08
#define BMA400_REG_ACC_Z_MSB        0x09

#define BMA400_REG_TEMP_DATA        0x11

/* Step counter / activity status */
#define BMA400_REG_STEP_CNT0        0x15
#define BMA400_REG_STEP_CNT1        0x16
#define BMA400_REG_STEP_CNT2        0x17
#define BMA400_REG_STEP_STAT        0x18

/* Accelerometer configuration */
#define BMA400_REG_ACC_CONFIG0      0x19
#define BMA400_REG_ACC_CONFIG1      0x1A
#define BMA400_REG_ACC_CONFIG2      0x1B

/* Interrupt status and configuration */
#define BMA400_REG_INT_STAT0        0x0E
#define BMA400_REG_INT_STAT1        0x0F
#define BMA400_REG_INT_STAT2        0x10

#define BMA400_REG_INT_CONFIG0      0x1F
#define BMA400_REG_INT_CONFIG1      0x20
#define BMA400_REG_INT1_MAP         0x21
#define BMA400_REG_INT2_MAP         0x22
#define BMA400_REG_INT12_MAP        0x23
#define BMA400_REG_INT12_IO_CTRL    0x24

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

/* Step status encoding from STEP_STAT.step_stat_field<1:0> */
typedef enum {
    BMA400_STEP_STAT_STILL = 0,
    BMA400_STEP_STAT_WALK  = 1,
    BMA400_STEP_STAT_RUN   = 2,
} bma400_step_stat_t;

/* Simple host-side sleep configuration.
 *
 * We are NOT yet changing the BMA400 power mode automatically.
 * The application will use the "no_motion" flag exposed via
 * bma400_app_get_sleep_flag() to decide when to actually sleep.
 */
struct bma400_sleep_params {
    bool     enabled;              /* If false, sleep flag is never asserted */
    uint32_t no_motion_timeout_s;  /* Seconds without motion/steps before flag */
};

/* Minimal public device struct. All extra state is kept private in bma400.c. */
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

/* Burst-read the hardware step counter and activity status. */
int bma400_read_step_counter(const struct bma400_dev *dev,
                             uint32_t *step_count,
                             bma400_step_stat_t *step_stat);

/* Convenience helpers that operate on the single global instance
 * selected from devicetree (bosch,bma400).
 */
int bma400_app_read_mg(float *x_mg, float *y_mg, float *z_mg);

int bma400_read_reg(const struct bma400_dev *dev,
                    uint8_t reg, uint8_t *val);

int bma400_write_reg(const struct bma400_dev *dev,
                     uint8_t reg, uint8_t val);

/* Application-level helpers
 *
 * bma400_app_init():
 *   - finds the devicetree node "bosch,bma400"
 *   - gets its I2C bus + reg address
 *   - calls bma400_init() internally
 *   - enables step counting and maps the step interrupt to INT1
 *
 * bma400_app_print():
 *   - reads step counter & activity status
 *   - updates internal "no motion" (sleep candidate) flag
 *   - prints a one-line debug summary
 */
int bma400_app_init(void);
int bma400_app_print(void);

/* Sleep / no-motion helper accessors.  These only manage host-side flags;
 * they do NOT put the BMA400 into low-power or sleep modes yet.
 */

/* Override default sleep parameters (enabled=true, 300 s). */
void bma400_app_set_sleep_params(const struct bma400_sleep_params *params);

/* Returns true if we've seen no new motion/steps for the configured timeout. */
bool bma400_app_get_sleep_flag(void);

typedef enum {
    BMA400_POWER_MODE_SLEEP  = BMA400_ACC_CONFIG0_POWER_MODE_SLEEP,
    BMA400_POWER_MODE_LP     = BMA400_ACC_CONFIG0_POWER_MODE_LP,
    BMA400_POWER_MODE_NORMAL = BMA400_ACC_CONFIG0_POWER_MODE_NORMAL,
} bma400_power_mode_t;

/* Change power mode of the global BMA400 instance selected from devicetree. */
int bma400_app_set_power_mode(bma400_power_mode_t mode);

/* Force wake-up from SLEEP mode by writing directly to ACC_CONFIG0.
 * Use this if the device is stuck in SLEEP and normal register reads fail.
 * Returns 0 on success, negative error code on failure.
 */
int bma400_app_force_wake(void);

/* Convenience: read step count + activity for the global instance. */
int bma400_app_read_step_counter(uint32_t *step_count,
                                 bma400_step_stat_t *step_stat);


#ifdef __cplusplus
}
#endif

#endif /* BMA400_H_ */
