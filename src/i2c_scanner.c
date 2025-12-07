/*
 * I2C Address Scanner
 * 
 * Scans the I2C bus and reports which addresses respond.
 * This is useful for debugging I2C device detection issues.
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/sys/printk.h>
#include <zephyr/logging/log.h>
#include <zephyr/devicetree.h>

LOG_MODULE_REGISTER(i2c_scanner, LOG_LEVEL_DBG);

/* Use the devicetree node label i2c0 directly */
#define I2C_NODE DT_NODELABEL(i2c0)

/**
 * @brief Scan I2C bus for devices
 * 
 * @param i2c_dev I2C device pointer
 * @return 0 on success, negative error code on failure
 */
int i2c_scan(const struct device *i2c_dev)
{
	uint8_t address;
	int ret;
	int found_count = 0;

	if (i2c_dev == NULL) {
		LOG_ERR("I2C device pointer is NULL");
		return -EINVAL;
	}

	if (!device_is_ready(i2c_dev)) {
		LOG_ERR("I2C device is not ready");
		return -ENODEV;
	}

	printk("\n");
	printk("========================================\n");
	printk("I2C Bus Scanner\n");
	printk("========================================\n");
	printk("Scanning I2C bus for devices...\n");
	printk("\n");

	/* I2C addresses are 7-bit, so range is 0x08 to 0x77 */
	/* Addresses 0x00-0x07 and 0x78-0x7F are reserved */
	printk("     0  1  2  3  4  5  6  7  8  9  A  B  C  D  E  F\n");

	for (uint8_t row = 0; row < 8; row++) {
		printk("%02X: ", row * 16);
		
		for (uint8_t col = 0; col < 16; col++) {
			address = (row * 16) + col;
			
			/* Skip reserved addresses */
			if (address < 0x08 || address > 0x77) {
				printk("   ");
				continue;
			}

			/* Try to write to the address (probe) */
			/* Use i2c_write with zero-length buffer to probe */
			ret = i2c_write(i2c_dev, NULL, 0, address);
			
			if (ret == 0) {
				/* Device found! */
				printk("%02X ", address);
				found_count++;
			} else {
				printk("-- ");
			}
		}
		printk("\n");
	}

	printk("\n");
	printk("========================================\n");
	printk("Scan complete. Found %d device(s).\n", found_count);
	printk("========================================\n");
	printk("\n");

	/* Print known device addresses for reference */
	printk("Known device addresses:\n");
	printk("  BMA400:     0x15\n");
	printk("  MAX30208:   0x18\n");
	printk("  MAXM86161:  0x62\n");
	printk("\n");

	return 0;
}

/**
 * @brief Initialize I2C scanner
 */
const struct device *i2c_scanner_init(void)
{
	const struct device *i2c_dev;

	/* Get the I2C device from device tree using devicetree API */
	i2c_dev = DEVICE_DT_GET(I2C_NODE);

	if (!device_is_ready(i2c_dev)) {
		LOG_ERR("I2C device not ready: %s", i2c_dev->name);
		return NULL;
	}

	LOG_INF("I2C device found: %s", i2c_dev->name);
	return i2c_dev;
}

