#include "maxm86161.h"

#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/sys/printk.h>
#include <limits.h>


/* ----------------------------------------------------------------
 * Devicetree / I2C binding
 * ---------------------------------------------------------------- */

#define MAXM86161_NODE DT_NODELABEL(maxm86161)

#if !DT_NODE_HAS_STATUS(MAXM86161_NODE, okay)
#error "No enabled DT node labeled 'maxm86161'. Please add one with reg = <0x62> on your I2C bus."
#endif

static const struct i2c_dt_spec maxm86161_i2c = I2C_DT_SPEC_GET(MAXM86161_NODE);

/* ----------------------------------------------------------------
 * Register map (from datasheet)
 * ---------------------------------------------------------------- */

#define MAXM86161_REG_INT_STATUS1      0x00
#define MAXM86161_REG_INT_STATUS2      0x01
#define MAXM86161_REG_INT_ENABLE1      0x02
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

/* System Control (0x0D) bits */
#define MAXM86161_SYSCTRL_RESET_BIT    (1U << 0)
#define MAXM86161_SYSCTRL_SHDN_BIT     (1U << 1)
#define MAXM86161_SYSCTRL_LP_MODE_BIT  (1U << 2)
#define MAXM86161_SYSCTRL_SINGLE_PPG   (1U << 3)

/* FIFO Configuration 2 (0x0A) bits */
#define MAXM86161_FIFO_FLUSH_BIT       (1U << 4)
#define MAXM86161_FIFO_STAT_CLR_BIT    (1U << 3)
#define MAXM86161_FIFO_A_FULL_TYPE_BIT (1U << 2)
#define MAXM86161_FIFO_RO_BIT          (1U << 1)

/* ----------------------------------------------------------------
 * HR estimation parameters
 * ---------------------------------------------------------------- */

/* Sample rate we configured via PPG_CFG2 (0x12) ~25 sps */
#define MAXM86161_SAMPLE_RATE_HZ           25.0f

/* FIFO read chunk size per call */
#define MAXM86161_MAX_SAMPLES_PER_READ     32U

/* HR buffer: circular buffer of LED1 samples */
#define MAXM86161_HR_BUF_LEN               256U

/* Use last ~8 seconds worth of samples => 8 * 25 = 200 */
#define MAXM86161_HR_WINDOW_SAMPLES        200U

/* Minimum peak-to-peak amplitude (in raw counts) to consider valid PPG */
#define MAXM86161_HR_MIN_AMPLITUDE         20.0f

/* Minimum distance between peaks in samples to avoid double-counting
 * (25 Hz -> 5 samples ≈ 0.2 s -> 300 bpm max)
 */
#define MAXM86161_HR_MIN_PEAK_DISTANCE     5U

/* Recompute HR at most once per this interval (ms) */
#define MAXM86161_HR_RECALC_INTERVAL_MS    1000U

/* Simple flag to gate printing until init succeeds */
static bool maxm86161_initialized = false;

/* HR estimator state */
static uint32_t hr_buf[MAXM86161_HR_BUF_LEN];
static uint16_t hr_buf_head = 0;
static uint32_t hr_total_samples = 0;
static float    hr_last_bpm = 0.0f;
static uint32_t hr_last_compute_ms = 0;

/* Static buffers for HR estimation (avoid stack overflow) */
static float    hr_y_buf[MAXM86161_HR_WINDOW_SAMPLES];
static uint16_t hr_peak_indices_buf[MAXM86161_HR_WINDOW_SAMPLES];

/* ----------------------------------------------------------------
 * Simple heart-rate estimation over LED1
 *
 * Derived from your log:
 * - No finger  : LED1 ≈ 300–400 counts
 * - With finger: LED1 ≈ 70k–90k counts
 * so contact gating uses a raw threshold around 5 kcounts.
 * --------------------------------------------------------------*/

 #define MAXM86161_SAMPLE_RATE_HZ        25.0f   /* matches PPG_CFG2 = 0x00 */
 #define MAXM86161_HR_BUF_LEN            256
 #define MAXM86161_HR_WINDOW_SAMPLES     200     /* ~8 s at 25 Hz */
 
 #define MAXM86161_HR_MIN_AMPLITUDE      300U    /* p2p counts */
 #define MAXM86161_HR_MIN_PEAK_DISTANCE  5       /* samples */
 #define MAXM86161_HR_MIN_BPM            30.0f
 #define MAXM86161_HR_MAX_BPM            220.0f
 #define MAXM86161_HR_SMOOTH_ALPHA       0.2f    /* EMA smoothing */
 
 #define MAXM86161_CONTACT_RAW_ON        5000U   /* DC level with finger on */
 #define MAXM86161_CONTACT_RAW_OFF       4000U   /* hysteresis off level */
 #define MAXM86161_CONTACT_ON_SAMPLES    10      /* ~0.4 s at 25 Hz */
 #define MAXM86161_CONTACT_OFF_SAMPLES   25      /* ~1 s at 25 Hz */
 #define MAXM86161_CONTACT_MIN_P2P       300U
 
 #define MAXM86161_HR_MAX_PEAKS          16
 
 typedef struct {
     uint32_t samples[MAXM86161_HR_BUF_LEN];
     uint16_t head;
     uint16_t count;
 
     bool     contact;
     uint16_t on_count;
     uint16_t off_count;
 
     float    hr_bpm;         /* latest filtered BPM */
     float    hr_bpm_smooth;  /* internal EMA state */
 } maxm86161_hr_state_t;
 
 static maxm86161_hr_state_t hr_state;
 
 static void maxm86161_hr_reset(void)
 {
     /* Clear circular buffer */
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
 }
 
 /* Push one LED1 sample, update contact + HR estimate if possible */
 static void maxm86161_hr_push_sample(uint32_t raw)
 {
     /* --- Contact detection based on raw DC level --- */
     if (raw > MAXM86161_CONTACT_RAW_ON) {
         hr_state.on_count++;
         hr_state.off_count = 0U;
     } else if (raw < MAXM86161_CONTACT_RAW_OFF) {
         hr_state.off_count++;
         hr_state.on_count = 0U;
     }
 
     if (!hr_state.contact && hr_state.on_count >= MAXM86161_CONTACT_ON_SAMPLES) {
         /* Finger just appeared */
         hr_state.contact       = true;
         hr_state.head          = 0U;
         hr_state.count         = 0U;
         hr_state.hr_bpm        = 0.0f;
         hr_state.hr_bpm_smooth = 0.0f;
     }
 
     if (hr_state.contact && hr_state.off_count >= MAXM86161_CONTACT_OFF_SAMPLES) {
         /* Finger just removed */
         hr_state.contact       = false;
         hr_state.hr_bpm        = 0.0f;
         hr_state.hr_bpm_smooth = 0.0f;
     }
 
     /* Always push into the circular buffer */
     hr_state.samples[hr_state.head] = raw;
     hr_state.head = (hr_state.head + 1U) % MAXM86161_HR_BUF_LEN;
     if (hr_state.count < MAXM86161_HR_BUF_LEN) {
         hr_state.count++;
     }
 
     if (!hr_state.contact) {
         /* Do NOT try to compute HR when there is no finger */
         return;
     }
 
     if (hr_state.count < MAXM86161_HR_WINDOW_SAMPLES) {
         /* Not enough history yet */
         return;
     }
 
     const uint16_t N = MAXM86161_HR_WINDOW_SAMPLES;
     uint32_t window[N];
     uint16_t start = (hr_state.head + MAXM86161_HR_BUF_LEN - N) %
                      MAXM86161_HR_BUF_LEN;
 
     uint32_t max_v = 0U;
     uint32_t min_v = UINT32_MAX;
     uint64_t sum   = 0U;
 
     for (uint16_t i = 0U; i < N; ++i) {
         uint32_t v = hr_state.samples[(start + i) % MAXM86161_HR_BUF_LEN];
         window[i]  = v;
         sum       += v;
 
         if (v > max_v) {
             max_v = v;
         }
         if (v < min_v) {
             min_v = v;
         }
     }
 
     uint32_t p2p = max_v - min_v;
     uint32_t min_p2p = (MAXM86161_HR_MIN_AMPLITUDE > MAXM86161_CONTACT_MIN_P2P)
                        ? MAXM86161_HR_MIN_AMPLITUDE
                        : MAXM86161_CONTACT_MIN_P2P;
 
     /* If the modulation is too small, treat as "no usable HR" */
     if (p2p < min_p2p) {
         hr_state.hr_bpm        = 0.0f;
         hr_state.hr_bpm_smooth = 0.0f;
         return;
     }
 
     float mean   = (float)sum / (float)N;
     float thresh = mean + 0.5f * ((float)max_v - mean);  /* ~50% above DC */
 
     uint16_t peaks[MAXM86161_HR_MAX_PEAKS];
     uint8_t  peak_count = 0U;
 
     for (uint16_t i = 1U; i < N - 1U; ++i) {
         float v0 = (float)window[i - 1U];
         float v1 = (float)window[i];
         float v2 = (float)window[i + 1U];
 
         if ((v1 > v0) && (v1 >= v2) && (v1 > thresh)) {
             if ((peak_count == 0U) ||
                 (i - peaks[peak_count - 1U]) >= MAXM86161_HR_MIN_PEAK_DISTANCE) {
 
                 if (peak_count < MAXM86161_HR_MAX_PEAKS) {
                     peaks[peak_count++] = i;
                 }
             }
         }
     }
 
     if (peak_count < 2U) {
         hr_state.hr_bpm        = 0.0f;
         hr_state.hr_bpm_smooth = 0.0f;
         return;
     }
 
     uint32_t sum_interval = 0U;
     for (uint8_t i = 1U; i < peak_count; ++i) {
         sum_interval += (uint32_t)(peaks[i] - peaks[i - 1U]);
     }
 
     float avg_interval_samples =
         (float)sum_interval / (float)(peak_count - 1U);
 
     if (avg_interval_samples <= 0.0f) {
         return;
     }
 
     float bpm = 60.0f * MAXM86161_SAMPLE_RATE_HZ / avg_interval_samples;
 
     if ((bpm < MAXM86161_HR_MIN_BPM) || (bpm > MAXM86161_HR_MAX_BPM)) {
         hr_state.hr_bpm        = 0.0f;
         hr_state.hr_bpm_smooth = 0.0f;
         return;
     }
 
     /* Simple EMA smoothing to avoid big jumps */
     if (hr_state.hr_bpm_smooth <= 0.1f) {
         hr_state.hr_bpm_smooth = bpm;
     } else {
         hr_state.hr_bpm_smooth +=
             MAXM86161_HR_SMOOTH_ALPHA * (bpm - hr_state.hr_bpm_smooth);
     }
 
     hr_state.hr_bpm = hr_state.hr_bpm_smooth;
 }
 
 /* Public accessors (header declares these) */
 int maxm86161_get_hr_bpm(void)
 {
     if (hr_state.hr_bpm <= 0.0f) {
         return 0;
     }
 
     return (int)(hr_state.hr_bpm + 0.5f);
 }
 
 bool maxm86161_has_contact(void)
 {
     return hr_state.contact;
 }
 

/* ----------------------------------------------------------------
 * I2C helpers
 * ---------------------------------------------------------------- */

static int maxm86161_write_reg(uint8_t reg, uint8_t value)
{
    return i2c_reg_write_byte_dt(&maxm86161_i2c, reg, value);
}

static int maxm86161_read_reg(uint8_t reg, uint8_t *value)
{
    return i2c_reg_read_byte_dt(&maxm86161_i2c, reg, value);
}

static int maxm86161_read_multi(uint8_t reg, uint8_t *buf, size_t len)
{
    return i2c_burst_read_dt(&maxm86161_i2c, reg, buf, len);
}

/* ----------------------------------------------------------------
 * HR estimator helpers (LED1 channel)
 * ---------------------------------------------------------------- */

static void hr_push_sample(uint32_t value)
{
    hr_buf[hr_buf_head] = value;
    hr_buf_head = (hr_buf_head + 1U) % MAXM86161_HR_BUF_LEN;
    hr_total_samples++;
}

static void hr_update_estimate(void)
{
    uint32_t now_ms = k_uptime_get_32();

    if ((now_ms - hr_last_compute_ms) < MAXM86161_HR_RECALC_INTERVAL_MS) {
        return; /* too soon to recompute */
    }

    hr_last_compute_ms = now_ms;

    if (hr_total_samples < MAXM86161_HR_WINDOW_SAMPLES) {
        hr_last_bpm = 0.0f;
        return;
    }

    /* Extract last WINDOW_SAMPLES from circular buffer */
    const uint16_t N = MAXM86161_HR_WINDOW_SAMPLES;
    float *y = hr_y_buf;  /* Use static buffer to avoid stack overflow */
    double sum = 0.0;

    /* Compute mean (DC) and detrend */
    uint16_t start = (hr_buf_head + MAXM86161_HR_BUF_LEN - N) % MAXM86161_HR_BUF_LEN;

    for (uint16_t i = 0; i < N; i++) {
        uint16_t idx = (start + i) % MAXM86161_HR_BUF_LEN;
        sum += (double)hr_buf[idx];
    }

    float mean = (float)(sum / (double)N);

    float max_y = -1e9f;
    float min_y =  1e9f;

    for (uint16_t i = 0; i < N; i++) {
        uint16_t idx = (start + i) % MAXM86161_HR_BUF_LEN;
        float v = (float)hr_buf[idx] - mean;
        y[i] = v;

        if (v > max_y) max_y = v;
        if (v < min_y) min_y = v;
    }

    float p2p = max_y - min_y;
    if (p2p < MAXM86161_HR_MIN_AMPLITUDE) {
        /* Not enough pulsatile signal (probably no finger or bad contact) */
        hr_last_bpm = 0.0f;
        return;
    }

    /* Simple peak detection on detrended signal */
    float threshold = max_y * 0.5f; /* 50% of max positive lobe */
    uint16_t *peak_indices = hr_peak_indices_buf;  /* Use static buffer to avoid stack overflow */
    uint16_t peak_count = 0;

    for (uint16_t i = 1; i < (N - 1U); i++) {
        float v = y[i];

        if ((v > y[i - 1U]) &&
            (v >= y[i + 1U]) &&
            (v > threshold)) {

            if ((peak_count == 0U) ||
                ((i - peak_indices[peak_count - 1U]) >= MAXM86161_HR_MIN_PEAK_DISTANCE)) {
                peak_indices[peak_count++] = i;
            }
        }
    }

    if (peak_count < 2U) {
        /* Not enough beats in window */
        hr_last_bpm = 0.0f;
        return;
    }

    /* Average peak-to-peak distance (in samples) */
    float sum_intervals = 0.0f;
    for (uint16_t k = 1; k < peak_count; k++) {
        sum_intervals += (float)(peak_indices[k] - peak_indices[k - 1U]);
    }
    float avg_interval_samples = sum_intervals / (float)(peak_count - 1U);

    if (avg_interval_samples <= 0.0f) {
        hr_last_bpm = 0.0f;
        return;
    }

    float bpm = 60.0f * MAXM86161_SAMPLE_RATE_HZ / avg_interval_samples;

    /* Reject obviously bogus BPM values */
    if ((bpm < 30.0f) || (bpm > 220.0f)) {
        hr_last_bpm = 0.0f;
    } else {
        hr_last_bpm = bpm;
    }
}

/* Public accessor */
float maxm86161_get_last_hr_bpm(void)
{
    return hr_last_bpm;
}

/* ----------------------------------------------------------------
 * Device configuration (following datasheet pseudo-code)
 * ---------------------------------------------------------------- */

int maxm86161_app_init(void)
{
    int ret;
    uint8_t val;

    if (!device_is_ready(maxm86161_i2c.bus)) {
        printk("MAXM86161: I2C bus not ready\n");
        return -ENODEV;
    }

    maxm86161_hr_reset();
    
    /* Verify PART_ID */
    ret = maxm86161_read_reg(MAXM86161_REG_PART_ID, &val);
    if (ret) {
        printk("MAXM86161: Failed to read PART_ID (%d)\n", ret);
        return ret;
    }

    if (val != MAXM86161_PART_ID_EXPECTED) {
        printk("MAXM86161: Unexpected PART_ID 0x%02X (expected 0x%02X)\n",
               val, MAXM86161_PART_ID_EXPECTED);
        /* Not fatal, but warn. You can choose to fail instead. */
    } else {
        printk("MAXM86161: PART_ID = 0x%02X OK\n", val);
    }

    /* Soft reset: write RESET bit */
    ret = maxm86161_write_reg(MAXM86161_REG_SYSTEM_CONTROL,
                              MAXM86161_SYSCTRL_RESET_BIT);
    if (ret) {
        printk("MAXM86161: Failed to write RESET (%d)\n", ret);
        return ret;
    }

    k_msleep(2);

    /* Put device into shutdown while configuring; enable SINGLE_PPG */
    ret = maxm86161_write_reg(MAXM86161_REG_SYSTEM_CONTROL,
                              MAXM86161_SYSCTRL_SINGLE_PPG |
                              MAXM86161_SYSCTRL_SHDN_BIT);
    if (ret) {
        printk("MAXM86161: Failed to enter shutdown (%d)\n", ret);
        return ret;
    }

    /* Clear any pending interrupts */
    (void)maxm86161_read_reg(MAXM86161_REG_INT_STATUS1, &val);
    (void)maxm86161_read_reg(MAXM86161_REG_INT_STATUS2, &val);

    /* ---- PPG Configuration (single green LED1 for HR) ----
     *
     *  PPG_CFG1 (0x11):
     *    PPG_TINT[1:0]    = 0x3  => 117.3 us
     *    PPG1_ADC_RGE[1:0]= 0x2  => 16 uA range
     *    ALC enabled, no ADD_OFFSET
     *    => 0b0000_1011 = 0x0B
     */
    ret = maxm86161_write_reg(MAXM86161_REG_PPG_CFG1, 0x0B);
    if (ret) {
        printk("MAXM86161: Failed to write PPG_CFG1 (%d)\n", ret);
        return ret;
    }

    /* PPG_CFG2 (0x12):
     *  PPG_SR[4:0] = 0x00 => ~25 sps
     *  SMP_AVE[2:0] = 0   => 1 sample average
     */
    ret = maxm86161_write_reg(MAXM86161_REG_PPG_CFG2, 0x00);
    if (ret) {
        printk("MAXM86161: Failed to write PPG_CFG2 (%d)\n", ret);
        return ret;
    }

    /* PPG_CFG3 (0x13):
     *  LED_SETLNG[1:0] = 0x3 => 12 us settling
     *  others 0
     *  => 0b1100_0000 = 0xC0
     */
    ret = maxm86161_write_reg(MAXM86161_REG_PPG_CFG3, 0xC0);
    if (ret) {
        printk("MAXM86161: Failed to write PPG_CFG3 (%d)\n", ret);
        return ret;
    }

    /* PD_BIAS (0x15):
     *  PDBIAS1[2:0] = 0x1 (Cpd = 0~65pF)
     */
    ret = maxm86161_write_reg(MAXM86161_REG_PD_BIAS, 0x01);
    if (ret) {
        printk("MAXM86161: Failed to write PD_BIAS (%d)\n", ret);
        return ret;
    }

    /* LED Range 1 (0x2A):
     *  LED1_RGE = 0x3 (124mA max range)
     *  LED2_RGE = 0x3
     *  LED3_RGE = 0x3
     *  => 0b0011_1111 = 0x3F
     */
    ret = maxm86161_write_reg(MAXM86161_REG_LED_RANGE1, 0x3F);
    if (ret) {
        printk("MAXM86161: Failed to write LED_RANGE1 (%d)\n", ret);
        return ret;
    }

    /* LED drive currents:
     *  LED1_DRV = 0x20 => ~15.36mA (per datasheet example)
     *  For now enable only LED1; keep others off.
     *  (If you later enable LED2/LED3 and sequences, the demux logic
     *   in maxm86161_app_print() already understands tags 0x02/0x03.)
     */
    ret = maxm86161_write_reg(MAXM86161_REG_LED1_PA, 0x20);
    if (ret) {
        printk("MAXM86161: Failed to write LED1_PA (%d)\n", ret);
        return ret;
    }

    ret = maxm86161_write_reg(MAXM86161_REG_LED2_PA, 0x00);
    if (ret) {
        printk("MAXM86161: Failed to write LED2_PA (%d)\n", ret);
        return ret;
    }

    ret = maxm86161_write_reg(MAXM86161_REG_LED3_PA, 0x00);
    if (ret) {
        printk("MAXM86161: Failed to write LED3_PA (%d)\n", ret);
        return ret;
    }

    /* LED Sequence:
     * Use only LED1 (green) in sequence 1.
     *
     *  LED_SEQ1 (0x20): LEDC1[3:0] = 0x1 => LED1, LEDC2 = 0
     *  LED_SEQ2 (0x21): 0x00
     *  LED_SEQ3 (0x22): 0x00
     */
    ret = maxm86161_write_reg(MAXM86161_REG_LED_SEQ1, 0x01);
    if (ret) {
        printk("MAXM86161: Failed to write LED_SEQ1 (%d)\n", ret);
        return ret;
    }

    ret = maxm86161_write_reg(MAXM86161_REG_LED_SEQ2, 0x00);
    if (ret) {
        printk("MAXM86161: Failed to write LED_SEQ2 (%d)\n", ret);
        return ret;
    }

    ret = maxm86161_write_reg(MAXM86161_REG_LED_SEQ3, 0x00);
    if (ret) {
        printk("MAXM86161: Failed to write LED_SEQ3 (%d)\n", ret);
        return ret;
    }

    /* FIFO configuration:
     *  FIFO_CFG1 (0x09): FIFO_A_FULL[6:0] = 0x0F => interrupt threshold;
     *  (we'll mostly poll FIFO_DATA_COUNT, but this is still a sane value)
     */
    ret = maxm86161_write_reg(MAXM86161_REG_FIFO_CFG1, 0x0F);
    if (ret) {
        printk("MAXM86161: Failed to write FIFO_CFG1 (%d)\n", ret);
        return ret;
    }

    /* FIFO_CFG2 (0x0A):
     *  First, flush FIFO.
     */
    ret = maxm86161_write_reg(MAXM86161_REG_FIFO_CFG2, MAXM86161_FIFO_FLUSH_BIT);
    if (ret) {
        printk("MAXM86161: Failed to flush FIFO (%d)\n", ret);
        return ret;
    }

    k_msleep(1);

    /* Then set RD_DATA_CLR + FIFO_RO, A_FULL_TYPE=0 */
    uint8_t fifo_cfg2 =
        MAXM86161_FIFO_STAT_CLR_BIT |   /* clear status on FIFO read */
        MAXM86161_FIFO_RO_BIT;          /* roll-over when full */

    ret = maxm86161_write_reg(MAXM86161_REG_FIFO_CFG2, fifo_cfg2);
    if (ret) {
        printk("MAXM86161: Failed to write FIFO_CFG2 (%d)\n", ret);
        return ret;
    }

    /* (Optional) Interrupt enables: we'll use polling, so keep them disabled.
     * If you later want DATA_RDY interrupt:
     *   write MAXM86161_REG_INT_ENABLE1 with DATA_RDY_EN bit set.
     */

    /* Finally, start sampling:
     *  SYSTEM_CONTROL:
     *    SINGLE_PPG = 1
     *    LP_MODE    = 1 (optional; OK at low sample rate)
     *    SHDN       = 0
     */
    uint8_t sys_ctrl = MAXM86161_SYSCTRL_SINGLE_PPG |
                       MAXM86161_SYSCTRL_LP_MODE_BIT;

    ret = maxm86161_write_reg(MAXM86161_REG_SYSTEM_CONTROL, sys_ctrl);
    if (ret) {
        printk("MAXM86161: Failed to start sampling (%d)\n", ret);
        return ret;
    }

    k_msleep(10);

    maxm86161_initialized = true;
    printk("MAXM86161: Initialization complete.\n");

    return 0;
}

/* ----------------------------------------------------------------
 * Polling + LED1/2/3 demux + HR estimate
 * ---------------------------------------------------------------- */

 #define MAXM86161_MAX_FIFO_SAMPLES 32U  /* chip FIFO depth for PPG samples */

 void maxm86161_app_print(void)
 {
     int ret;
     uint8_t fifo_cnt = 0U;
 
     if (!maxm86161_initialized) {
         return;
     }
 
     ret = maxm86161_read_reg(MAXM86161_REG_FIFO_DATA_COUNT, &fifo_cnt);
     if (ret < 0) {
         printk("MAXM86161: failed to read FIFO_DATA_COUNT (%d)\n", ret);
         return;
     }
 
     uint8_t num_samples = fifo_cnt & 0x7FU;  /* lower 7 bits is sample count */
 
     if (num_samples == 0U) {
         printk("MAXM86161: fifo_cnt=0\n");
         return;
     }
 
     if (num_samples > MAXM86161_MAX_FIFO_SAMPLES) {
         num_samples = MAXM86161_MAX_FIFO_SAMPLES;
     }
 
     uint8_t buf[3U * MAXM86161_MAX_FIFO_SAMPLES];
 
     ret = maxm86161_read_multi(MAXM86161_REG_FIFO_DATA,
                                buf,
                                3U * num_samples);
     if (ret < 0) {
         printk("MAXM86161: failed to read FIFO data (%d)\n", ret);
         return;
     }
 
     /* Per-LED counters and last value */
     uint8_t  led_count[3] = { 0U, 0U, 0U };
     uint32_t led_last[3]  = { 0U, 0U, 0U };
     uint16_t used_samples = 0U;
 
     for (uint8_t i = 0U; i < num_samples; ++i) {
         const uint8_t *p = &buf[3U * i];
 
         uint32_t sample24 = ((uint32_t)p[0] << 16) |
                             ((uint32_t)p[1] << 8)  |
                              (uint32_t)p[2];
 
         uint8_t  tag   = (sample24 >> 19) & 0x1FU; /* upper 5 bits */
         uint32_t value =  sample24        & 0x7FFFFU; /* 19-bit value */
 
         uint8_t led_index;
 
         switch (tag) {
         case 0x01:  /* LED1 */
             led_index = 0;
             /* Only LED1 is used for HR for now */
             maxm86161_hr_push_sample(value);
             break;
         case 0x02:  /* LED2 */
             led_index = 1;
             break;
         case 0x03:  /* LED3 */
             led_index = 2;
             break;
         default:
             /* ambient / unused tag, ignore in this simple app */
             continue;
         }
 
         if (led_count[led_index] < UINT8_MAX) {
             led_count[led_index]++;
         }
         led_last[led_index] = value;
         used_samples++;
     }
 
     int  hr_bpm     = maxm86161_get_hr_bpm();
     bool contact_on = maxm86161_has_contact();
 
     printk("MAXM86161: fifo_cnt=%u, read=%u, "
            "LED1=%u(last=%lu) LED2=%u(last=%lu) LED3=%u(last=%lu), "
            "HR=%d BPM%s\n",
            (unsigned int)num_samples,
            (unsigned int)used_samples,
            (unsigned int)led_count[0], (unsigned long)led_last[0],
            (unsigned int)led_count[1], (unsigned long)led_last[1],
            (unsigned int)led_count[2], (unsigned long)led_last[2],
            hr_bpm,
            contact_on ? "" : " (no contact)");
 }
 
