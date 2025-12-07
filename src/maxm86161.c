#include "maxm86161.h"

#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>

#include <math.h>
#include <limits.h>

/* ----------------------------------------------------------------
 * Devicetree / I2C binding
 * ---------------------------------------------------------------- */

#define MAXM86161_NODE DT_NODELABEL(maxm86161)

#if !DT_NODE_HAS_STATUS(MAXM86161_NODE, okay)
#error "maxm86161 devicetree node not found or disabled"
#endif

static const struct i2c_dt_spec maxm86161_i2c = I2C_DT_SPEC_GET(MAXM86161_NODE);

#if DT_NODE_HAS_PROP(MAXM86161_NODE, enable_gpios)
/* Optional EN pin: enable-gpios = <&gpioX Y GPIO_ACTIVE_HIGH>; in DTS */
static const struct gpio_dt_spec maxm86161_en_gpio =
    GPIO_DT_SPEC_GET(MAXM86161_NODE, enable_gpios);
#define MAXM86161_HAS_EN_GPIO 1
#else
#define MAXM86161_HAS_EN_GPIO 0
#endif

/* ----------------------------------------------------------------
 * Register addresses (from datasheet)
 * ---------------------------------------------------------------- */

#define MAXM86161_REG_INT_STATUS1      0x00
#define MAXM86161_REG_INT_STATUS2      0x01
#define MAXM86161_REG_INT_ENABLE1      0x02
#define MAXM86161_REG_INT_ENABLE2      0x03
#define MAXM86161_REG_FIFO_WR_PTR      0x04
#define MAXM86161_REG_FIFO_RD_PTR      0x05
#define MAXM86161_REG_OVF_COUNTER      0x06
#define MAXM86161_REG_FIFO_DATA_COUNT  0x07
#define MAXM86161_REG_FIFO_DATA        0x08
#define MAXM86161_REG_FIFO_CFG1        0x09
#define MAXM86161_REG_FIFO_CFG2        0x0A
#define MAXM86161_REG_SYSTEM_CONTROL   0x0D

#define MAXM86161_REG_PPG_CFG1         0x11
#define MAXM86161_REG_PPG_CFG2         0x12
#define MAXM86161_REG_PPG_CFG3         0x13
#define MAXM86161_REG_PD_BIAS          0x15

#define MAXM86161_REG_LED_SEQ1         0x20
#define MAXM86161_REG_LED_SEQ2         0x21
#define MAXM86161_REG_LED_SEQ3         0x22
#define MAXM86161_REG_LED1_PA          0x23
#define MAXM86161_REG_LED2_PA          0x24
#define MAXM86161_REG_LED3_PA          0x25
#define MAXM86161_REG_LED_RANGE1       0x2A

#define MAXM86161_REG_TEMP_CFG         0x40
#define MAXM86161_REG_TEMP_INT         0x41
#define MAXM86161_REG_TEMP_FRAC        0x42

#define MAXM86161_REG_PART_ID          0xFF

/* Expected PART_ID from datasheet */
#define MAXM86161_PART_ID_EXPECTED     0x36

/* ----------------------------------------------------------------
 * Bit fields
 * ---------------------------------------------------------------- */

/* SYSTEM_CONTROL (0x0D) bits */
#define MAXM86161_SYSCTRL_RESET_BIT    (1U << 0)
#define MAXM86161_SYSCTRL_SHDN_BIT     (1U << 1)
#define MAXM86161_SYSCTRL_LP_MODE_BIT  (1U << 2)
#define MAXM86161_SYSCTRL_SINGLE_PPG   (1U << 3)

/* FIFO_CFG2 (0x0A) bits */
#define MAXM86161_FIFO_FLUSH_BIT       (1U << 4)
#define MAXM86161_FIFO_STAT_CLR_BIT    (1U << 3)
#define MAXM86161_FIFO_A_FULL_TYPE_BIT (1U << 2)
#define MAXM86161_FIFO_RO_BIT          (1U << 1)

/* ----------------------------------------------------------------
 * HR / SpO2 estimation parameters
 * ---------------------------------------------------------------- */

/* Sample rate we configure via PPG_CFG2 (~25 sps) */
#define MAXM86161_SAMPLE_RATE_HZ           25.0f

/* FIFO read chunk size per call */
#define MAXM86161_MAX_SAMPLES_PER_READ     32U

/* HR buffer: circular buffer of LED1 samples */
// #define MAXM86161_HR_BUF_LEN               256U

/* Use last ~8 seconds worth of samples => 8 * 25 = 200 */
// #define MAXM86161_HR_WINDOW_SAMPLES        200U

/* Minimum peak-to-peak amplitude (raw counts) to consider valid PPG */
#define MAXM86161_HR_MIN_AMPLITUDE         300.0f

/* Peak detection parameters */
// #define MAXM86161_HR_MIN_PEAK_DISTANCE     5U    /* samples (~300 bpm max) */
// #define MAXM86161_HR_MIN_BPM              30.0f
// #define MAXM86161_HR_MAX_BPM             220.0f
// #define MAXM86161_HR_SMOOTH_ALPHA          0.2f

/* Contact detection (from your earlier logs) */
#define MAXM86161_CONTACT_RAW_ON        5000U
#define MAXM86161_CONTACT_RAW_OFF       4000U
#define MAXM86161_CONTACT_ON_SAMPLES      10U
#define MAXM86161_CONTACT_OFF_SAMPLES     25U
#define MAXM86161_CONTACT_MIN_P2P       300U

// #define MAXM86161_HR_MAX_PEAKS            16U

/* Very simple SpO2 estimator tuning */
#define MAXM86161_SPO2_DC_MIN           5000.0f
#define MAXM86161_SPO2_AC_MIN              10.0f

/* ----------------------------------------------------------------
 * HR / SpO2 helper configuration
 * ---------------------------------------------------------------- */

 #define MAXM86161_HR_BUF_LEN              256
 #define MAXM86161_HR_WINDOW_SEC           8.0f
 #define MAXM86161_HR_WINDOW_SAMPLES       ((uint16_t)(MAXM86161_SAMPLE_RATE_HZ * MAXM86161_HR_WINDOW_SEC))
 
 #define MAXM86161_HR_MIN_BPM              40.0f
 #define MAXM86161_HR_MAX_BPM              200.0f
 #define MAXM86161_HR_MIN_PEAK_DISTANCE    5u
 #define MAXM86161_HR_SMOOTH_ALPHA         0.30f
 #define MAXM86161_HR_MAX_PEAKS            16u
 
 /* NEW: HR band-pass/peak-detector parameters -------------------------------- */
 
/* LP cutoff after DC removal; with sample_rate ≈ 25 Hz this gives a ~0–3 Hz band */
#define MAXM86161_HR_LP_CUTOFF_HZ             4.0f

/* Reject windows where the detrended PPG has almost no motion (very poor signal) */
#define MAXM86161_HR_MIN_P2P                  150.0f   /* raw counts after DC removal */

/* Dynamic peak threshold: require peaks to be a fraction of the window P2P */
#define MAXM86161_HR_MIN_PEAK_HEIGHT_FRACTION 0.15f
 
 /* And also enforce an absolute floor so tiny ripples don't count as peaks */
 #define MAXM86161_HR_MIN_PEAK_HEIGHT_ABS      200.0f
 

/* ----------------------------------------------------------------
 * Internal state
 * ---------------------------------------------------------------- */

static bool maxm86161_initialized = false;

/* Circular buffer for LED1 samples (HR) */
typedef struct {
    uint32_t samples[MAXM86161_HR_BUF_LEN];
    uint16_t head;
    uint16_t count;

    bool     contact;
    uint16_t on_count;
    uint16_t off_count;

    float    hr_bpm;         /* latest filtered BPM */
    float    hr_bpm_smooth;  /* EMA state */
} maxm86161_hr_state_t;

static maxm86161_hr_state_t hr_state;

/* Simple SpO2 state derived from LED2 (RED) + LED3 (IR) */
static float spo2_dc_red   = 0.0f;
static float spo2_ac_red   = 0.0f;
static float spo2_dc_ir    = 0.0f;
static float spo2_ac_ir    = 0.0f;
static int   spo2_pct      = 0;

/* ----------------------------------------------------------------
 * I2C helpers
 * ---------------------------------------------------------------- */

static int maxm86161_write_reg(uint8_t reg, uint8_t val)
{
    uint8_t buf[2] = { reg, val };
    return i2c_write_dt(&maxm86161_i2c, buf, sizeof(buf));
}

static int maxm86161_read_reg(uint8_t reg, uint8_t *val)
{
    return i2c_write_read_dt(&maxm86161_i2c, &reg, 1, val, 1);
}

static int maxm86161_read_multi(uint8_t reg, uint8_t *buf, size_t len)
{
    return i2c_write_read_dt(&maxm86161_i2c, &reg, 1, buf, len);
}

/* ----------------------------------------------------------------
 * HR / SpO2 helper functions
 * ---------------------------------------------------------------- */

static void maxm86161_hr_reset_state(void)
{
    for (uint16_t i = 0; i < MAXM86161_HR_BUF_LEN; ++i) {
        hr_state.samples[i] = 0U;
    }

    hr_state.head          = 0U;
    hr_state.count         = 0U;
    hr_state.contact       = false;
    hr_state.on_count      = 0U;
    hr_state.off_count     = 0U;
    hr_state.hr_bpm        = 0.0f;
    hr_state.hr_bpm_smooth = 0.0f;

    spo2_dc_red = spo2_ac_red = 0.0f;
    spo2_dc_ir  = spo2_ac_ir  = 0.0f;
    spo2_pct    = 0;
}

/* Update simple contact detection based on LED1 raw DC */
static void maxm86161_update_contact(uint32_t raw)
{
    if (!hr_state.contact) {
        if (raw > MAXM86161_CONTACT_RAW_ON) {
            hr_state.on_count++;
            if (hr_state.on_count >= MAXM86161_CONTACT_ON_SAMPLES) {
                hr_state.contact  = true;
                hr_state.off_count = 0;
            }
        } else {
            hr_state.on_count = 0;
        }
    } else {
        if (raw < MAXM86161_CONTACT_RAW_OFF) {
            hr_state.off_count++;
            if (hr_state.off_count >= MAXM86161_CONTACT_OFF_SAMPLES) {
                hr_state.contact   = false;
                hr_state.on_count  = 0;
                hr_state.hr_bpm    = 0.0f;
                hr_state.hr_bpm_smooth = 0.0f;
                spo2_pct           = 0;
            }
        } else {
            hr_state.off_count = 0;
        }
    }
}

/* Very simple peak-based HR estimator over the last N samples */
static void maxm86161_hr_push_sample(uint32_t sample)
{
    /* 1) Push raw sample into circular buffer -------------------------------- */
    hr_state.samples[hr_state.head] = sample;

    hr_state.head = (hr_state.head + 1U) % MAXM86161_HR_BUF_LEN;
    if (hr_state.count < MAXM86161_HR_BUF_LEN) {
        hr_state.count++;
    }

    /* Need at least one full analysis window */
    if (hr_state.count < MAXM86161_HR_WINDOW_SAMPLES) {
        return;
    }

    /* If we don't currently have contact, don't try to estimate HR */
    if (!hr_state.contact) {
        hr_state.hr_bpm        = 0.0f;
        hr_state.hr_bpm_smooth = 0.0f;
        return;
    }

    /* 2) Build window + DC removal ------------------------------------------ */
    const uint16_t N     = MAXM86161_HR_WINDOW_SAMPLES;
    const uint16_t start = (hr_state.head + MAXM86161_HR_BUF_LEN - N) % MAXM86161_HR_BUF_LEN;

    float y[MAXM86161_HR_WINDOW_SAMPLES];  /* detrended + filtered PPG */

    float sum   = 0.0f;
    float min_v = 1.0e30f;
    float max_v = -1.0e30f;

    for (uint16_t i = 0; i < N; i++) {
        uint16_t idx = (uint16_t)((start + i) % MAXM86161_HR_BUF_LEN);
        float    v   = (float)hr_state.samples[idx];

        sum   += v;
        if (v < min_v) min_v = v;
        if (v > max_v) max_v = v;
    }

    const float dc  = sum / (float)N;
    const float p2p = max_v - min_v;

    /* If the signal has almost no swing, don't trust any HR */
    if (p2p < MAXM86161_HR_MIN_P2P) {
        hr_state.hr_bpm        = 0.0f;
        hr_state.hr_bpm_smooth = 0.0f;
        return;
    }

    /* Build detrended window: x[n] - DC */
    for (uint16_t i = 0; i < N; i++) {
        uint16_t idx = (uint16_t)((start + i) % MAXM86161_HR_BUF_LEN);
        float    v   = (float)hr_state.samples[idx];
        y[i] = v - dc;
    }

    /* 3) Simple IIR low-pass → overall band-pass (DC removal + LP) ---------- */
    {
        /* 1st order LP at MAXM86161_HR_LP_CUTOFF_HZ, sampled at MAXM86161_SAMPLE_RATE_HZ */
        const float dt     = 1.0f / MAXM86161_SAMPLE_RATE_HZ;
        const float rc_lp  = 1.0f / (2.0f * 3.14159265f * MAXM86161_HR_LP_CUTOFF_HZ);
        const float alpha  = dt / (rc_lp + dt);      /* standard RC LP discretisation */

        float lp = 0.0f;
        for (uint16_t i = 0; i < N; i++) {
            lp   += alpha * (y[i] - lp);
            y[i]  = lp;
        }
    }

    /* 4) More robust peak detection ----------------------------------------- */

    /* Dynamic height threshold based on window P2P, with an absolute floor */
    float min_peak_height = MAXM86161_HR_MIN_PEAK_HEIGHT_FRACTION * p2p;
    if (min_peak_height < MAXM86161_HR_MIN_PEAK_HEIGHT_ABS) {
        min_peak_height = MAXM86161_HR_MIN_PEAK_HEIGHT_ABS;
    }

    uint16_t peak_indices[MAXM86161_HR_MAX_PEAKS];
    uint16_t peak_count = 0;

    int16_t last_peak_i = -(int16_t)MAXM86161_HR_MIN_PEAK_DISTANCE;

    /* Look for local maxima that:
     *   - are above the dynamic threshold
     *   - are separated by MIN_PEAK_DISTANCE samples
     */
    for (uint16_t i = 1; i + 1 < N; i++) {
        float y_prev = y[i - 1];
        float y_curr = y[i];
        float y_next = y[i + 1];

        /* Local maximum */
        if (!(y_curr > y_prev && y_curr >= y_next)) {
            continue;
        }

        /* Big enough to matter */
        if (y_curr < min_peak_height) {
            continue;
        }

        /* Not too close to previous detected peak */
        if ((int16_t)i - last_peak_i < (int16_t)MAXM86161_HR_MIN_PEAK_DISTANCE) {
            continue;
        }

        peak_indices[peak_count++] = i;
        last_peak_i = (int16_t)i;

        if (peak_count >= MAXM86161_HR_MAX_PEAKS) {
            break;
        }
    }

    /* Need at least two peaks to estimate a period */
    if (peak_count < 2U) {
        hr_state.hr_bpm        = 0.0f;
        hr_state.hr_bpm_smooth = 0.0f;
        return;
    }

    /* 5) Convert inter-peak intervals → BPM, reject outliers ----------------- */
    float total_interval_samples = 0.0f;
    uint16_t valid_intervals     = 0;

    for (uint16_t k = 1; k < peak_count; k++) {
        uint16_t d = (uint16_t)(peak_indices[k] - peak_indices[k - 1]);
        if (d == 0U) {
            continue;
        }

        float bpm = 60.0f * MAXM86161_SAMPLE_RATE_HZ / (float)d;
        if (bpm < MAXM86161_HR_MIN_BPM || bpm > MAXM86161_HR_MAX_BPM) {
            continue;  /* discard clearly bogus intervals */
        }

        total_interval_samples += (float)d;
        valid_intervals++;
    }

    if (valid_intervals == 0U) {
        hr_state.hr_bpm        = 0.0f;
        hr_state.hr_bpm_smooth = 0.0f;
        return;
    }

    const float avg_interval_samples = total_interval_samples / (float)valid_intervals;
    const float hr_bpm_new           = 60.0f * MAXM86161_SAMPLE_RATE_HZ / avg_interval_samples;

    /* 6) Exponential smoothing for a stable display value ------------------- */
    if (hr_state.hr_bpm_smooth <= 0.0f) {
        hr_state.hr_bpm_smooth = hr_bpm_new;
    } else {
        hr_state.hr_bpm_smooth =
            (1.0f - MAXM86161_HR_SMOOTH_ALPHA) * hr_state.hr_bpm_smooth +
            MAXM86161_HR_SMOOTH_ALPHA * hr_bpm_new;
    }

    hr_state.hr_bpm = hr_state.hr_bpm_smooth;
}

/* Simple IIR update for SpO2 DC/AC tracking */
static void maxm86161_spo2_update_red(uint32_t raw)
{
    const float alpha = 0.01f;
    const float beta  = 0.01f;

    if (spo2_dc_red <= 1.0f) {
        spo2_dc_red = (float)raw;
        spo2_ac_red = 0.0f;
    } else {
        spo2_dc_red += alpha * ((float)raw - spo2_dc_red);
        float abs_ac = fabsf((float)raw - spo2_dc_red);
        if (spo2_ac_red <= 1.0f) {
            spo2_ac_red = abs_ac;
        } else {
            spo2_ac_red += beta * (abs_ac - spo2_ac_red);
        }
    }
}

static void maxm86161_spo2_update_ir(uint32_t raw)
{
    const float alpha = 0.01f;
    const float beta  = 0.01f;

    if (spo2_dc_ir <= 1.0f) {
        spo2_dc_ir = (float)raw;
        spo2_ac_ir = 0.0f;
    } else {
        spo2_dc_ir += alpha * ((float)raw - spo2_dc_ir);
        float abs_ac = fabsf((float)raw - spo2_dc_ir);
        if (spo2_ac_ir <= 1.0f) {
            spo2_ac_ir = abs_ac;
        } else {
            spo2_ac_ir += beta * (abs_ac - spo2_ac_ir);
        }
    }
}

/* Very rough ratio-of-ratios SpO2 approximation */
static void maxm86161_spo2_compute(void)
{
    if (!hr_state.contact) {
        spo2_pct = 0;
        return;
    }

    if (spo2_dc_red < MAXM86161_SPO2_DC_MIN ||
        spo2_dc_ir  < MAXM86161_SPO2_DC_MIN) {
        /* Not enough signal yet */
        spo2_pct = 0;
        return;
    }

    if (spo2_ac_red < MAXM86161_SPO2_AC_MIN ||
        spo2_ac_ir  < MAXM86161_SPO2_AC_MIN) {
        spo2_pct = 0;
        return;
    }

    float ratio_num = (spo2_ac_red / spo2_dc_red);
    float ratio_den = (spo2_ac_ir  / spo2_dc_ir);

    if (ratio_den <= 0.0f) {
        return;
    }

    float R = ratio_num / ratio_den;

    /* Generic linear mapping used in simple oximeter demos:
     * SpO2 ≈ 110 - 25 * R  (for transmissive; reflective needs tuning)
     */
    float spo2 = 110.0f - 25.0f * R;

    if (spo2 > 100.0f) spo2 = 100.0f;
    if (spo2 < 0.0f)   spo2 = 0.0f;

    spo2_pct = (int)(spo2 + 0.5f);
}

/* ----------------------------------------------------------------
 * Device configuration helpers
 * ---------------------------------------------------------------- */

static int maxm86161_hw_reset(void)
{
    int ret;

    /* Software reset */
    ret = maxm86161_write_reg(MAXM86161_REG_SYSTEM_CONTROL,
                              MAXM86161_SYSCTRL_RESET_BIT);
    if (ret) {
        printk("MAXM86161: failed to write RESET (%d)\n", ret);
        return ret;
    }

    k_msleep(2);

    /* Shutdown while configuring; SINGLE_PPG set */
    ret = maxm86161_write_reg(MAXM86161_REG_SYSTEM_CONTROL,
                              MAXM86161_SYSCTRL_SHDN_BIT |
                              MAXM86161_SYSCTRL_SINGLE_PPG);
    if (ret) {
        printk("MAXM86161: failed to enter shutdown (%d)\n", ret);
        return ret;
    }

    return 0;
}

static int maxm86161_config_ppg_led_fifo(void)
{
    int ret;
    uint8_t fifo_cfg2;
    uint8_t val;

    /* Clear any pending interrupts */
    (void)maxm86161_read_reg(MAXM86161_REG_INT_STATUS1, &val);
    (void)maxm86161_read_reg(MAXM86161_REG_INT_STATUS2, &val);

    /* PPG_CFG1 (0x11):
     *  ALC_EN = 1 (ambient light cancellation)
     *  OFFSET_NO = 0
     *  ADC_RANGE = 2 (16 µA)
     *  TINT = 2 (~58.7 µs)
     */
    ret = maxm86161_write_reg(MAXM86161_REG_PPG_CFG1, 0x8A);
    if (ret) {
        printk("MAXM86161: failed to write PPG_CFG1 (%d)\n", ret);
        return ret;
    }

    /* PPG_CFG2 (0x12):
     *  PPG_SR[7:3]  = 0x00 -> ~25 sps
     *  SMP_AVE[2:0] = 0x02 -> 4 sample average
     *  => 0x02
     */
    ret = maxm86161_write_reg(MAXM86161_REG_PPG_CFG2, 0x00);
    if (ret) {
        printk("MAXM86161: failed to write PPG_CFG2 (%d)\n", ret);
        return ret;
    }

    /* PPG_CFG3 (0x13):
     *  LED_SETLNG[7:6] = 0x3 => 12 µs settling
     *  rest 0
     *  => 0xC0
     */
    ret = maxm86161_write_reg(MAXM86161_REG_PPG_CFG3, 0xC0);
    if (ret) {
        printk("MAXM86161: failed to write PPG_CFG3 (%d)\n", ret);
        return ret;
    }

    /* PD_BIAS (0x15):
     *  PDBIAS1[2:0] = 0x1 (small PD cap)
     */
    ret = maxm86161_write_reg(MAXM86161_REG_PD_BIAS, 0x01);
    if (ret) {
        printk("MAXM86161: failed to write PD_BIAS (%d)\n", ret);
        return ret;
    }

    /* LED ranges (0x2A):
     * For simplicity, use default range (0x00).
     * You can increase range if you need more LED current headroom.
     */
    ret = maxm86161_write_reg(MAXM86161_REG_LED_RANGE1, 0x00);
    if (ret) {
        printk("MAXM86161: failed to write LED_RANGE1 (%d)\n", ret);
        return ret;
    }

    /* LED currents:
     *
     * NOTE: On the MAXM86161 module used on Heart Rate 2 Click,
     * typical mapping is:
     *   LED1: Green
     *   LED2: Red
     *   LED3: IR
     *
     * Currents here are moderate starting values; tune as needed.
     */
    ret = maxm86161_write_reg(MAXM86161_REG_LED1_PA, 0x40); /* green */
    if (ret) {
        printk("MAXM86161: failed to write LED1_PA (%d)\n", ret);
        return ret;
    }

    ret = maxm86161_write_reg(MAXM86161_REG_LED2_PA, 0x30); /* red */
    if (ret) {
        printk("MAXM86161: failed to write LED2_PA (%d)\n", ret);
        return ret;
    }

    ret = maxm86161_write_reg(MAXM86161_REG_LED3_PA, 0x30); /* IR */
    if (ret) {
        printk("MAXM86161: failed to write LED3_PA (%d)\n", ret);
        return ret;
    }

    /* LED sequence:
     * Single PPG mode; we schedule three exposures:
     *   Sequence 1: LED1
     *   Sequence 2: LED2
     *   Sequence 3: LED3
     * Tags in FIFO:
     *   LED1 -> tag 0x01
     *   LED2 -> tag 0x02
     *   LED3 -> tag 0x03
     */
    ret = maxm86161_write_reg(MAXM86161_REG_LED_SEQ1, 0x21); /* C2=2,C1=1 */
    if (ret) {
        printk("MAXM86161: failed to write LED_SEQ1 (%d)\n", ret);
        return ret;
    }

    ret = maxm86161_write_reg(MAXM86161_REG_LED_SEQ2, 0x03); /* C3=3 */
    if (ret) {
        printk("MAXM86161: failed to write LED_SEQ2 (%d)\n", ret);
        return ret;
    }

    ret = maxm86161_write_reg(MAXM86161_REG_LED_SEQ3, 0x00);
    if (ret) {
        printk("MAXM86161: failed to write LED_SEQ3 (%d)\n", ret);
        return ret;
    }

    /* FIFO configuration:
     * FIFO_CFG1 (0x09):
     *   SMP_AVE_FIFO = 0 (we already averaged in PPG_CFG2)
     *   FIFO_ROLLOVER_EN = 0 (stop when full)
     * Here keep defaults (0x00) for simplicity.
     */
    ret = maxm86161_write_reg(MAXM86161_REG_FIFO_CFG1, 0x00);
    if (ret) {
        printk("MAXM86161: failed to write FIFO_CFG1 (%d)\n", ret);
        return ret;
    }

    /* FIFO_CFG2 (0x0A):
     *   FIFO_A_FULL[7:5] = threshold (we don't use interrupt)
     *   FIFO_FLUSH = 1 to clear
     *   FIFO_STAT_CLR = 1 to clear pointers/overflow
     */
    fifo_cfg2 = 0;
    fifo_cfg2 |= MAXM86161_FIFO_FLUSH_BIT;
    fifo_cfg2 |= MAXM86161_FIFO_STAT_CLR_BIT;

    ret = maxm86161_write_reg(MAXM86161_REG_FIFO_CFG2, fifo_cfg2);
    if (ret) {
        printk("MAXM86161: failed to write FIFO_CFG2 (%d)\n", ret);
        return ret;
    }

    /* We use polling, so keep all interrupts disabled */
    ret = maxm86161_write_reg(MAXM86161_REG_INT_ENABLE1, 0x00);
    if (ret) {
        printk("MAXM86161: failed to write INT_ENABLE1 (%d)\n", ret);
        return ret;
    }
    ret = maxm86161_write_reg(MAXM86161_REG_INT_ENABLE2, 0x00);
    if (ret) {
        printk("MAXM86161: failed to write INT_ENABLE2 (%d)\n", ret);
        return ret;
    }

    /* Start sampling:
     *   SINGLE_PPG = 1
     *   LP_MODE    = 1 (low power at low SR)
     *   SHDN       = 0
     */
    uint8_t sys_ctrl = MAXM86161_SYSCTRL_SINGLE_PPG |
                       MAXM86161_SYSCTRL_LP_MODE_BIT;

    ret = maxm86161_write_reg(MAXM86161_REG_SYSTEM_CONTROL, sys_ctrl);
    if (ret) {
        printk("MAXM86161: failed to start sampling (%d)\n", ret);
        return ret;
    }

    k_msleep(10);

    return 0;
}

/* ----------------------------------------------------------------
 * Public API
 * ---------------------------------------------------------------- */

int maxm86161_app_init(void)
{
    int ret;
    uint8_t part_id;

    /* EN pin like Passoll driver: drive high, wait 5 ms */
#if MAXM86161_HAS_EN_GPIO
    if (!device_is_ready(maxm86161_en_gpio.port)) {
        printk("MAXM86161: EN GPIO port not ready\n");
        return -ENODEV;
    }

    ret = gpio_pin_configure_dt(&maxm86161_en_gpio, GPIO_OUTPUT_ACTIVE);
    if (ret) {
        printk("MAXM86161: failed to configure EN GPIO (%d)\n", ret);
        return ret;
    }

    k_msleep(5);
#endif

    if (!device_is_ready(maxm86161_i2c.bus)) {
        printk("MAXM86161: I2C bus not ready\n");
        return -ENODEV;
    }

    maxm86161_hr_reset_state();

    ret = maxm86161_hw_reset();
    if (ret) {
        return ret;
    }

    /* Verify PART_ID */
    ret = maxm86161_read_reg(MAXM86161_REG_PART_ID, &part_id);
    if (ret) {
        printk("MAXM86161: failed to read PART_ID (%d)\n", ret);
        return ret;
    }

    if (part_id != MAXM86161_PART_ID_EXPECTED) {
        printk("MAXM86161: unexpected PART_ID=0x%02X (expected 0x%02X)\n",
               part_id, MAXM86161_PART_ID_EXPECTED);
        return -EINVAL;
    }

    printk("MAXM86161: PART_ID = 0x%02X OK\n", part_id);

    ret = maxm86161_config_ppg_led_fifo();
    if (ret) {
        return ret;
    }

    maxm86161_initialized = true;
    printk("MAXM86161: Initialization complete.\n");

    return 0;
}

/* Read FIFO, push samples into HR/SpO2 state, and print debug info */
void maxm86161_app_print(void)
{
    if (!maxm86161_initialized) {
        return;
    }

    int ret;
    uint8_t fifo_count = 0;

    ret = maxm86161_read_reg(MAXM86161_REG_FIFO_DATA_COUNT, &fifo_count);
    if (ret) {
        printk("MAXM86161: failed to read FIFO_DATA_COUNT (%d)\n", ret);
        return;
    }

    if (fifo_count == 0U) {
        /* Nothing to read, still update debug once in a while */
        maxm86161_spo2_compute();
        printk("MAXM86161: fifo_cnt=0, HR=%d BPM, SpO2=%d%%%s\n",
               maxm86161_get_hr_bpm(),
               maxm86161_get_spo2_pct(),
               maxm86161_has_contact() ? "" : " (no contact)");
        return;
    }

    if (fifo_count > MAXM86161_MAX_SAMPLES_PER_READ) {
        fifo_count = MAXM86161_MAX_SAMPLES_PER_READ;
    }

    uint8_t buf[3U * MAXM86161_MAX_SAMPLES_PER_READ];

    ret = maxm86161_read_multi(MAXM86161_REG_FIFO_DATA,
                               buf, 3U * fifo_count);
    if (ret < 0) {
        printk("MAXM86161: failed to read FIFO data (%d)\n", ret);
        return;
    }

    /* Per-LED counters + last values for logging */
    uint8_t  led_count[3] = { 0U, 0U, 0U };
    uint32_t led_last[3]  = { 0U, 0U, 0U };

    for (uint8_t i = 0U; i < fifo_count; ++i) {
        const uint8_t *p = &buf[3U * i];

        uint32_t sample24 = ((uint32_t)p[0] << 16) |
                            ((uint32_t)p[1] << 8)  |
                            ((uint32_t)p[2]);

        uint8_t  tag   = (sample24 >> 19) & 0x1FU;     /* upper 5 bits */
        uint32_t value =  sample24        & 0x7FFFFU;  /* 19-bit value */

        uint8_t led_index;

        switch (tag) {
        case 0x01:  /* LED1 (green) */
            led_index = 0;
            /* NEW: update contact from raw LED1 DC level */
            maxm86161_update_contact(value);
            maxm86161_hr_push_sample(value);
            break;
        
        case 0x02:  /* LED2 (red) */
            led_index = 1;
            maxm86161_spo2_update_red(value);
            break;
        
        case 0x03:  /* LED3 (IR) */
            led_index = 2;
            maxm86161_spo2_update_ir(value);
            break;
        
        default:
            /* Unknown tag or ambient sample; ignore for now */
            continue;
        }

        if (led_index < 3U) {
            led_count[led_index]++;
            led_last[led_index] = value;
        }
    }

    maxm86161_spo2_compute();

    int hr_bpm  = maxm86161_get_hr_bpm();
    int spo2    = maxm86161_get_spo2_pct();
    bool contact_on = maxm86161_has_contact();

    printk("MAXM86161: fifo_cnt=%u, "
           "LED1=%u(last=%lu) LED2=%u(last=%lu) LED3=%u(last=%lu), "
           "HR=%d BPM, SpO2=%d%%%s\n",
           (unsigned int)fifo_count,
           (unsigned int)led_count[0], (unsigned long)led_last[0],
           (unsigned int)led_count[1], (unsigned long)led_last[1],
           (unsigned int)led_count[2], (unsigned long)led_last[2],
           hr_bpm,
           spo2,
           contact_on ? "" : " (no contact)");
}

/* ----------------------------------------------------------------
 * Public accessors
 * ---------------------------------------------------------------- */

int maxm86161_get_hr_bpm(void)
{
    if (!maxm86161_initialized) {
        return 0;
    }

    if (!hr_state.contact) {
        return 0;
    }

    if (hr_state.hr_bpm < 1.0f) {
        return 0;
    }

    return (int)(hr_state.hr_bpm + 0.5f);
}

int maxm86161_get_spo2_pct(void)
{
    if (!maxm86161_initialized) {
        return 0;
    }

    if (!hr_state.contact) {
        return 0;
    }

    return spo2_pct;
}

bool maxm86161_has_contact(void)
{
    if (!maxm86161_initialized) {
        return false;
    }

    return hr_state.contact;
}

int maxm86161_set_enabled(bool enabled)
{
    if (!maxm86161_initialized) {
        return -EIO;
    }

    uint8_t sys_ctrl = 0;
    int ret = maxm86161_read_reg(MAXM86161_REG_SYSTEM_CONTROL, &sys_ctrl);
    if (ret) {
        printk("MAXM86161: read SYSTEM_CONTROL failed: %d\n", ret);
        return ret;
    }

    if (enabled) {
        /* Clear SHDN, keep SINGLE_PPG + LP_MODE bits as configured in init */
        sys_ctrl &= ~MAXM86161_SYSCTRL_SHDN_BIT;
    } else {
        sys_ctrl |= MAXM86161_SYSCTRL_SHDN_BIT;
    }

    ret = maxm86161_write_reg(MAXM86161_REG_SYSTEM_CONTROL, sys_ctrl);
    if (ret) {
        printk("MAXM86161: write SYSTEM_CONTROL failed: %d\n", ret);
        return ret;
    }

    if (!enabled) {
        /* Clear contact + HR/SpO2 state so we don't report stale values. */
        maxm86161_hr_reset_state();
    }

    printk("MAXM86161: %s\n", enabled ? "enabled" : "disabled");
    return 0;
}
