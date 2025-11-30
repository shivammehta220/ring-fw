#ifndef MAXM86161_H_
#define MAXM86161_H_

#include <zephyr/device.h>
#include <stdbool.h>
#include <stdint.h>

/* Application-level helpers */
int maxm86161_app_init(void);
void maxm86161_app_print(void);

/* Simple accessors for HR state */
int  maxm86161_get_hr_bpm(void);   /* Rounded BPM, 0 = no valid HR */
bool maxm86161_has_contact(void);  /* true when a finger is on LED1 */

#endif /* MAXM86161_H_ */
