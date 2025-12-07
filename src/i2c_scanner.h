/*
 * I2C Address Scanner Header
 */

#ifndef I2C_SCANNER_H
#define I2C_SCANNER_H

#include <zephyr/device.h>

/**
 * @brief Initialize I2C scanner
 * 
 * Gets the I2C device from device tree and verifies it's ready.
 * 
 * @return Pointer to I2C device on success, NULL on failure
 */
const struct device *i2c_scanner_init(void);

/**
 * @brief Scan I2C bus for devices
 * 
 * @param i2c_dev I2C device pointer
 * @return 0 on success, negative error code on failure
 */
int i2c_scan(const struct device *i2c_dev);

#endif /* I2C_SCANNER_H */

