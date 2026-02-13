#include <stdio.h>
#include <stdint.h>
#include <math.h>
#include <limits.h>
//Assumptions: 12-bit adc is read every 250us (4kHz), every 4 samples oversampling and iir is performed (1kHz sample frequency)
//Use proper rounding in oversampling
//sum = (sum + 1) >> 1;

//iir with configurable alpha:
//version 1:
// Shift‑only alphas: α = 1 / 2ᴺ
// Keep N as a runtime parameter. This gives you alphas in the discrete set:

// N=1 → α=0.5
// N=2 → α=0.25
// N=3 → α=0.125
// N=4 → α=0.0625
// sum: your oversampled input
// filtered: your IIR state
// N: 1..15 typically (runtime-configurable)
static inline void iir_shift_only(uint32_t sum, uint32_t *filtered, uint8_t N)
{
    // Compute signed error to allow movement in both directions
    int32_t err = (int32_t)sum - (int32_t)(*filtered);

    // Rounding to reduce bias: add 0.5 LSB of the shifted domain
    int32_t delta = (err + (1 << (N - 1))) >> N;

    *filtered += (int32_t)delta;
}

//version 2:
// k is Q15 gain: k = (int32_t)lroundf(alpha * 32768.0f);   // 0..32768
static inline void iir_q15(uint32_t sum, uint32_t *filtered, int32_t k_q15)
{
    int32_t err = (int32_t)sum - (int32_t)(*filtered);
    // Multiply in 32-bit and round before shifting down
    int32_t delta = (int32_t)(( (int64_t)err * (int64_t)k_q15 + (1 << 14) ) >> 15);
    *filtered += delta;
}

//best of both:

typedef struct {
    uint8_t use_q15;   // 0: shift-only, 1: Q15
    uint8_t N;         // used if use_q15 == 0
    int32_t k_q15;     // used if use_q15 == 1
    uint32_t filtered;
} iir_cfg_t;

static inline void iir_update(iir_cfg_t *cfg, uint32_t sum)
{
    if (!cfg->use_q15) {
        // Shift-only mode
        int32_t err = (int32_t)sum - (int32_t)cfg->filtered;
        int32_t delta = (err + (1 << (cfg->N - 1))) >> cfg->N;  // rounded
        cfg->filtered += delta;
    } else {
        // Q15 mode
        int32_t err = (int32_t)sum - (int32_t)cfg->filtered;
        int32_t delta = (int32_t)(( (int64_t)err * cfg->k_q15 + (1 << 14) ) >> 15);
        cfg->filtered += delta;
    }
}


static inline int32_t alpha_to_q15(float fc, float fs_hz)
{
    float alpha = (2.0f * 3.14159265359f * fc) / fs_hz; // α ≈ 2π fc / fs
    if (alpha < 0.0f) alpha = 0.0f;
    if (alpha > 1.0f) alpha = 1.0f;
    return (int32_t)lroundf(alpha * 32768.0f);
}


