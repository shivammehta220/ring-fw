#ifndef MAXM86161_H_
#define MAXM86161_H_

#include <stdint.h>
#include <stdbool.h>

/*
 * Public API used by main.c
 *
 * maxm86161_app_init()  - initialize sensor (I2C + EN pin + PPG/LED/FIFO)
 * maxm86161_app_print() - read FIFO, update HR/SpO2, print debug line
 * maxm86161_get_hr_bpm()      - latest heart-rate estimate (BPM)
 * maxm86161_get_spo2_pct()    - latest SpO2 estimate (%)
 * maxm86161_has_contact()     - true if finger/skin is detected
 */

int  maxm86161_app_init(void);
void maxm86161_app_print(void);

int  maxm86161_get_hr_bpm(void);
int  maxm86161_get_spo2_pct(void);
bool maxm86161_has_contact(void);

int maxm86161_set_enabled(bool enabled);


#endif /* MAXM86161_H_ */
