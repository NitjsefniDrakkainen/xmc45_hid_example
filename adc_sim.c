
#define _GNU_SOURCE
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <stdbool.h>
#include <time.h>
#include <pthread.h>
#include <math.h>    // (optional, not required here)


#define ADC_BITS            12
#define ADC_MAX             ((1u << ADC_BITS) - 1u)
#define VREF                3.3f


#define ADC_CHANNELS_NUM    10      // 2 joysticks + 4 pots + 2x battery = 10 channels
#define ADC_SAMPLES_NUM     4       // 4 samples per channel -> oversampling window
#define SAMPLE_PERIOD_US    250     // per full scan of all channels (i.e., 1 sample per channel)
#define FRAMES_TO_RUN       1000    // number of complete-IRQs (1 kHz) to run before exit


// IIR configuration: shift-only alpha = 1/2^N
#define IIR_SHIFT_N         3       // N=3 -> alpha = 0.125 ~ 20 Hz cutoff @ 1 kHz update

#define DEADBAND 2  // in oversampled-count units (~13-bit domain)
#define AVG_CENTER_POS 1024

typedef struct {
    uint8_t use_q15;   // 0: shift-only, 1: Q15
    uint8_t N;         // used if use_q15 == 0
    int32_t k_q15;     // used if use_q15 == 1
    uint32_t filtered;
} iir_cfg_t;


// ---- DMA buffer ----
// Layout: adc_dma_buffer[channel][sample_index 0..3]
static volatile uint16_t adc_dma_buffer[ADC_CHANNELS_NUM][ADC_SAMPLES_NUM];

// ---- Thread sync ----
static pthread_mutex_t mtx = PTHREAD_MUTEX_INITIALIZER;
static pthread_cond_t  cv  = PTHREAD_COND_INITIALIZER;
static bool half_complete_flag = false;
static bool complete_flag      = false;
static bool stop_requested     = false;

// ---- State for processing ----
static uint32_t partial_sum[ADC_CHANNELS_NUM]; // sum of first half (2 samples)
static int32_t  filtered[ADC_CHANNELS_NUM];    // IIR state (kept wider for headroom)
static bool     filter_initialized = false;
static iir_cfg_t cfg;
//


// static inline void iir_update(iir_cfg_t *cfg, uint32_t sum)
// {
//     if (!cfg->use_q15) {
//         // Shift-only mode
//         int32_t err = (int32_t)sum - (int32_t)cfg->filtered;
//         int32_t delta = (err + (1 << (cfg->N - 1))) >> cfg->N;  // rounded
//         cfg->filtered += delta;
//     } else {
//         // Q15 mode
//         int32_t err = (int32_t)sum - (int32_t)cfg->filtered;
//         int32_t delta = (int32_t)(( (int64_t)err * cfg->k_q15 + (1 << 14) ) >> 15);
//         cfg->filtered += delta;
//     }
// }

static inline void iir_update(iir_cfg_t *cfg, int32_t *state, uint32_t sum)
{
    int32_t err = (int32_t)sum - *state;

    if (!cfg->use_q15) {
        int32_t delta = (err + (1 << (cfg->N - 1))) >> cfg->N;
        *state += delta;
    } else {
        int32_t delta = (int32_t)(((int64_t)err * cfg->k_q15 + (1 << 14)) >> 15);
        *state += delta;
    }
}


static inline int32_t alpha_to_q15(float fc, float fs_hz)
{
    float alpha = (2.0f * 3.14159265359f * fc) / fs_hz; // α ≈ 2π fc / fs
    if (alpha < 0.0f) alpha = 0.0f;
    if (alpha > 1.0f) alpha = 1.0f;
    return (int32_t)lroundf(alpha * 32768.0f);
}


static void adc_process_block(uint16_t * buffer, bool flag);
// Utility: sleep for microseconds (best-effort on Linux)
static void sleep_us(long us)
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    long nsec = ts.tv_nsec + us * 1000L;
    ts.tv_sec += nsec / 1000000000L;
    ts.tv_nsec = nsec % 1000000000L;
    clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &ts, NULL);
}

static void prvCenterPosCompensate(uint16_t *usIn, uint16_t usCenterPos, uint16_t usThreshold)
{
    int32_t lDelta = (int32_t)(*usIn) - (int32_t)usCenterPos;
    int16_t sCompensation = 2048 - (int16_t)usCenterPos;

    if (lDelta > (int32_t)usThreshold)
    {
        lDelta -= usThreshold;
    }
    else if (lDelta < -(int32_t)usThreshold)
    {
        lDelta += usThreshold;
    }
    else
    {
        lDelta = 0;
    }

    *usIn = (uint16_t)((lDelta + (int32_t)usCenterPos + sCompensation) - usCenterPos);
}


static inline uint32_t median4_u16(uint16_t a, uint16_t b, uint16_t c, uint16_t d) {
    // Sort network for 4 elements (small & branchless-ish)
    uint16_t t;
    if (a > b) { t=a; a=b; b=t; }
    if (c > d) { t=c; c=d; d=t; }
    if (a > c) { t=a; a=c; c=t; }
    if (b > d) { t=b; b=d; d=t; }
    // Now a<=c and b<=d, median is max(a,c) with min(b,d), approximate with (b+c)/2
    return ((uint32_t)b + (uint32_t)c) >> 1;
}

// Callback prototypes
static void halfComplete_clbk(void);
static void transferComplete_clbk(void);


// Simulate one ADC sample for a given channel: 1.65 V ± noise
// 12-bit ADC, Vref = 3.3 -> midscale ~ 2048.
// Noise: uniform ±8 LSB (tweak as desired).
static inline uint16_t simulate_adc_reading(int ch, unsigned *seed)
{
    (void)ch; // Currently same for all channels; could vary by channel if desired
    int mid = (ADC_MAX + 1) / 2; // 2048 for 12-bit
    int noise = (int)(rand_r(seed) % 17) - 8; // [-8..+8]
    int val = mid + noise;

    if (val < 0) val = 0;
    if (val > (int)ADC_MAX) val = ADC_MAX;
    return (uint16_t)val;
}


// Producer thread: simulates DMA filling the buffer.
// Every 250 us, it writes 1 sample per channel (i.e., a "sequence").
// After 2 sequences -> halfComplete; after 4 sequences -> transferComplete.
// Repeats for FRAMES_TO_RUN frames.
static void* producer_thread(void *arg)
{
    (void)arg;

    unsigned seed = (unsigned)time(NULL) ^ 0xC0FFEEu;

    for (int frame = 0; frame < FRAMES_TO_RUN && !stop_requested; ++frame) {
        // Fill the 4 sample slots for each channel.
        for (int s = 0; s < ADC_SAMPLES_NUM; ++s) {
            // Simulate one sequence (1 sample per channel)
            for (int ch = 0; ch < ADC_CHANNELS_NUM; ++ch) {
                uint16_t sample = simulate_adc_reading(ch, &seed);
                adc_dma_buffer[ch][s] = sample;
            }

            // Trigger half/complete callbacks at the right times
            if (s == 1) {
                halfComplete_clbk();
            } else if (s == 3) {
                transferComplete_clbk();
            }

            // Wait 250 us until next sequence
            sleep_us(SAMPLE_PERIOD_US);
        }
        // After s=3 complete, one "frame" is done (1 ms total)
    }

    // Ask consumer to stop
    pthread_mutex_lock(&mtx);
    stop_requested = true;
    pthread_cond_signal(&cv);
    pthread_mutex_unlock(&mtx);

    return NULL;
}


// Consumer thread: responds to half and complete events.
// - On half: accumulates partial sums for each channel (first 2 samples).
// - On complete: adds second half, performs oversampling + IIR, prints summary.
static void* consumer_thread(void *arg)
{
    (void)arg;

    int frame_counter = 0;

    while (1) {
        pthread_mutex_lock(&mtx);
        while (!half_complete_flag && !complete_flag && !stop_requested) {
            pthread_cond_wait(&cv, &mtx);
        }

        bool do_half = half_complete_flag;
        bool do_complete = complete_flag;
        half_complete_flag = false;
        complete_flag = false;
        bool should_stop = stop_requested;
        pthread_mutex_unlock(&mtx);

        if (should_stop) break;

        if (do_half) {
            // Process first half: sample indices 0 and 1
            for (int ch = 0; ch < ADC_CHANNELS_NUM; ++ch) {
                uint32_t s0 = adc_dma_buffer[ch][0];
                uint32_t s1 = adc_dma_buffer[ch][1];
                partial_sum[ch] = s0 + s1;
            }
        }

        if (do_complete) {
            // Finish oversampling: add sample indices 2 and 3
            // Then compute IIR with alpha = 1 / 2^N (shift-only)
            for (int ch = 0; ch < ADC_CHANNELS_NUM; ++ch) {
                uint32_t s2 = adc_dma_buffer[ch][2];
                uint32_t s3 = adc_dma_buffer[ch][3];
                uint32_t sum4 = partial_sum[ch] + s2 + s3;

                // uint16_t s0 = adc_dma_buffer[ch][0];
                // uint16_t s1 = adc_dma_buffer[ch][1];
                // uint16_t s2 = adc_dma_buffer[ch][2];
                // uint16_t s3 = adc_dma_buffer[ch][3];
                // uint32_t m = median4_u16(s0, s1, s2, s3);

                // "True" 4x oversampling for +1 bit: sum of 4 then >> 1
                // Use rounding to reduce bias
                uint32_t oversampled = (sum4 + 1u) >> 1; // average * 2 (i.e., adds 1 bit of resolution)

                // Scale to match previous dynamic range (×2)
                // uint32_t oversampled = m << 1;


                if (!filter_initialized) {
                    filtered[ch] = (int32_t)oversampled;
                } else {
                    int32_t err = (int32_t)oversampled - filtered[ch];
                    if (err >= -DEADBAND && err <= DEADBAND) {
                        // hold
                    } else {
                        // int32_t delta = (err + (1 << (IIR_SHIFT_N - 1))) >> IIR_SHIFT_N;
                        // filtered[ch] += delta;
                        iir_update(&cfg, &filtered[ch], (int32_t)oversampled);
                    }
                }
            }
            filter_initialized = true;
            frame_counter++;

            // Print a brief summary (limit verbosity)
            if ((frame_counter % 50) == 0) {
                printf("Frame %d:\n", frame_counter);
                for (int ch = 0; ch < ADC_CHANNELS_NUM; ++ch) {
                    // For human readability, show oversampled and filtered in raw counts
                    // and also voltage estimate from filtered (back to 12-bit-ish scale)
                    // Note: 'oversampled' isn't kept here per channel; we can guess from filtered,
                    // or just print filtered. To show something meaningful, compute a quick
                    // "voltage" from filtered/2 (since oversampled ~ avg*2).
                    float filtered_to_12bit = (float)filtered[ch] / 2.0f; // approx revert oversampling scale
                    float volts = (filtered_to_12bit / (float)ADC_MAX) * VREF;
                    uint16_t val = filtered[ch] >> 1;
                    prvCenterPosCompensate(&val, AVG_CENTER_POS, 10);
                    printf("  ch%02d: raw: %6d, filtered=%6d (~%.3f V), 12 bits: %6d, 11 bits: %6d, 10 bits: %6d\n", ch, adc_dma_buffer[ch][3], filtered[ch], volts, val, filtered[ch]>>2, filtered[ch]>>3);
                }
                puts("");
            }
        }
    }

    return NULL;
}

int main()
{

    printf("ADC DMA simulation starting...\n");
    printf("Channels=%d, Samples/Ch=%d, ScanPeriod=%dus, IIR alpha=1/2^%d\n",
           ADC_CHANNELS_NUM, ADC_SAMPLES_NUM, SAMPLE_PERIOD_US, IIR_SHIFT_N);
    printf("Expect 1 kHz IIR update rate (4x 250us sequences per frame).\n\n");
cfg.k_q15 = alpha_to_q15(20.0f, 1000.0f);
cfg.use_q15 = 1;
cfg.N = 3;
    // Seed global RNG too (not essential)
    srand((unsigned)time(NULL) ^ 0xB16B00B5u);

    pthread_t prod, cons;
    pthread_create(&prod, NULL, producer_thread, NULL);
    pthread_create(&cons, NULL, consumer_thread, NULL);

    pthread_join(prod, NULL);
    pthread_join(cons, NULL);

    printf("Simulation complete.\n");
    return 0;

}


// IRQ callbacks (simulated): set flags and wake consumer
static void halfComplete_clbk(void)
{
    pthread_mutex_lock(&mtx);
    half_complete_flag = true;
    pthread_cond_signal(&cv);
    pthread_mutex_unlock(&mtx);
}

static void transferComplete_clbk(void)
{
    pthread_mutex_lock(&mtx);
    complete_flag = true;
    pthread_cond_signal(&cv);
    pthread_mutex_unlock(&mtx);
}

static void adc_process_block(uint16_t * buffer, bool flag)
{

}
