#include "microduck_infer.h"
#include <math.h>

static inline float elu1(float x) { return x > 0.0f ? x : (expf(x) - 1.0f); }

int microduck_forward(const microduck_policy_t *p, const float *obs, float *act)
{
    static float bufA[MICRODUCK_MAX_WIDTH];
    static float bufB[MICRODUCK_MAX_WIDTH];
    if (p->obs_dim > MICRODUCK_MAX_WIDTH || p->dims[0] != p->obs_dim || p->dims[p->n_layers] != p->act_dim) {
        return -1;
    }
    for (uint8_t i = 1; i < p->n_layers; i++) {
        if (p->dims[i] > MICRODUCK_MAX_WIDTH) {
            return -1;
        }
    }
    // normalizer: (obs - mean) / std
    float *x = bufA;
    float *y = bufB;
    for (uint16_t i = 0; i < p->obs_dim; i++) {
        x[i] = (obs[i] - p->obs_mean[i]) / p->obs_std[i];
    }
    for (uint8_t l = 0; l < p->n_layers; l++) {
        const uint16_t n_in = p->dims[l];
        const uint16_t n_out = p->dims[l + 1];
        const float *W = p->W[l];
        const float *b = p->b[l];
        float *dst = (l == p->n_layers - 1) ? act : y;
        for (uint16_t o = 0; o < n_out; o++) {
            const float *w = W + (uint32_t)o * n_in;
            float acc = b[o];
            for (uint16_t i = 0; i < n_in; i++) {
                acc += w[i] * x[i];
            }
            dst[o] = p->act[l] ? elu1(acc) : acc;
        }
        if (dst != act) {
            float *t = x; x = y; y = t;
        }
    }
    return 0;
}
