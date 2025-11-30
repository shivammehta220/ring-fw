#ifndef MAX30208_H_
#define MAX30208_H_

#include <zephyr/kernel.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Initialize the MAX30208 helper.
 *
 * - Finds an I2C bus
 * - Resets the MAX30208
 * - Flushes FIFO
 * - Verifies PART_ID
 *
 * @return 0 on success, negative errno on failure.
 */
int max30208_app_init(void);

/**
 * @brief Take a single temperature measurement.
 *
 * @param[out] temp_c  Temperature in degrees Celsius.
 *
 * @return 0 on success, negative errno on failure.
 */
int max30208_read_temperature_c(float *temp_c);

/**
 * @brief Convenience function that measures and prints temperature.
 *
 * Safe to call periodically from the main thread loop.
 */
void max30208_app_print(void);

#ifdef __cplusplus
}
#endif

#endif /* MAX30208_H_ */
