// Author: Roberto Navoni, member of the ArduPilot Dev Team
// Contact: r.navoni74@gmail.com
// Developed by Roberto Navoni — DelphyAI LAB
// For information: r.navoni74@gmail.com
#include "nnmixer_infer.h"
#include <math.h>
#include <stddef.h>

#define CARTAN_EPS 1e-8f

static inline float elu1(float x) { return x > 0.0f ? x : (expf(x) - 1.0f); }

// y[n_out] = W[n_out x n_in] x[n_in] + b
static void gemv(const float *W, const float *b, const float *x, float *y, uint16_t n_out, uint16_t n_in)
{
    for (uint16_t o = 0; o < n_out; o++) {
        const float *w = W + (uint32_t)o * n_in;
        float acc = b[o];
        for (uint16_t i = 0; i < n_in; i++) {
            acc += w[i] * x[i];
        }
        y[o] = acc;
    }
}

int nnmixer_cartan_forward(const nnmixer_cartan_t *p, const float *obs, float *act)
{
    static float xn[NNMIXER_MAX_WIDTH];
    static float f[NNMIXER_MAX_WIDTH];      // fiber (paint)
    static float g[NNMIXER_MAX_WIDTH];      // scratch fiber
    const uint16_t q = p->paint;
    if (p->obs_dim > NNMIXER_MAX_WIDTH || q > NNMIXER_MAX_WIDTH) {
        return -1;
    }
    for (uint16_t i = 0; i < p->obs_dim; i++) {
        xn[i] = (obs[i] - p->obs_mean[i]) / p->obs_std[i];
    }
    // embed: h = [0 ; W_in xn + b_in]
    gemv(p->in_W, p->in_b, xn, f, q, p->obs_dim);
    float c = 0.0f;
    for (uint8_t l = 0; l < p->n_layers; l++) {
        // paint GEMM on the fiber
        gemv(p->W[l], p->b[l], f, g, q, q);
        // left translation by beta: solvable_mul(beta, [c ; g])
        //   c' = c + beta0 ; f' = g + exp(-c) * beta_rest
        const float *beta = p->beta[l];
        const float e_mc = expf(-c);
        for (uint16_t i = 0; i < q; i++) {
            f[i] = g[i] + e_mc * beta[1 + i];
        }
        c = c + beta[0];
        // fiber rotation by unit theta = [phi0 ; phi_rest]
        const float *th = p->theta[l];
        const float phi0 = th[0];
        float mod = 0.0f, dot = 0.0f;
        for (uint16_t i = 0; i < q; i++) {
            mod += f[i] * f[i];
            dot += f[i] * th[1 + i];
        }
        const float c_exp = expf(c);
        const float a = (mod + 1.0f) * c_exp;
        const float bb = 1.0f / (c_exp > CARTAN_EPS ? c_exp : CARTAN_EPS);
        float arg = 0.5f * (-phi0 * (a - bb) + (bb + a)) - dot;
        if (arg < CARTAN_EPS) {
            arg = CARTAN_EPS;
        }
        const float yc = -logf(arg);
        if (fabsf(phi0 + 1.0f) < CARTAN_EPS) {
            for (uint16_t i = 0; i < q; i++) {
                f[i] = -f[i];
            }
        } else {
            const float k = -dot / (phi0 + 1.0f + CARTAN_EPS) + 0.5f * (bb - a);
            for (uint16_t i = 0; i < q; i++) {
                f[i] = f[i] + th[1 + i] * k;
            }
        }
        c = yc;
        // DiLU on the fiber, except after the last layer
        if (l + 1 < p->n_layers) {
            const float al = p->dilu_alpha;
            for (uint16_t i = 0; i < q; i++) {
                f[i] = (elu1(f[i]) + al * f[i]) / (1.0f + al);
            }
        }
    }
    // Euclidean readout: fiber * exp(c), then head
    const float ec = expf(c);
    for (uint16_t i = 0; i < q; i++) {
        g[i] = f[i] * ec;
    }
    gemv(p->head_W, p->head_b, g, act, p->act_dim, q);
    return 0;
}

int nnmixer_forward(const nnmixer_policy_t *p, const float *obs, float *act)
{
    static float bufA[NNMIXER_MAX_WIDTH];
    static float bufB[NNMIXER_MAX_WIDTH];
    if (p->obs_dim > NNMIXER_MAX_WIDTH || p->dims[0] != p->obs_dim || p->dims[p->n_layers] != p->act_dim) {
        return -1;
    }
    for (uint8_t i = 1; i < p->n_layers; i++) {
        if (p->dims[i] > NNMIXER_MAX_WIDTH) {
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

int nnmixer_forward_int8(const nnmixer_policy_int8_t *p, const float *obs, float *act)
{
    static float bufA[NNMIXER_MAX_WIDTH];
    static float bufB[NNMIXER_MAX_WIDTH];
    if (p == NULL || p->n_layers == 0 || p->n_layers > NNMIXER_MAX_LAYERS) {
        return -1;
    }
    if (p->obs_dim > NNMIXER_MAX_WIDTH || p->dims[0] != p->obs_dim || p->dims[p->n_layers] != p->act_dim) {
        return -1;
    }
    for (uint8_t i = 1; i < p->n_layers; i++) {
        if (p->dims[i] > NNMIXER_MAX_WIDTH) {
            return -1;
        }
    }
    float *x = bufA;
    float *y = bufB;
    for (uint16_t i = 0; i < p->obs_dim; i++) {
        x[i] = (obs[i] - p->obs_mean[i]) / p->obs_std[i];
    }
    for (uint8_t l = 0; l < p->n_layers; l++) {
        const uint16_t n_in = p->dims[l];
        const uint16_t n_out = p->dims[l + 1];
        const int8_t *W = p->W[l];
        const float *scale = p->w_scale[l];
        const float *b = p->b[l];
        float *dst = (l == p->n_layers - 1) ? act : y;
        for (uint16_t o = 0; o < n_out; o++) {
            const int8_t *w = W + (uint32_t)o * n_in;
            float acc = b[o];
            const float s = scale[o];
            for (uint16_t i = 0; i < n_in; i++) {
                acc += (float)w[i] * s * x[i];
            }
            dst[o] = p->act[l] ? elu1(acc) : acc;
        }
        if (dst != act) {
            float *t = x; x = y; y = t;
        }
    }
    return 0;
}
