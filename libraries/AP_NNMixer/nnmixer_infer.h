// Author: Roberto Navoni, member of the ArduPilot Dev Team
// Contact: r.navoni74@gmail.com
// Developed by Roberto Navoni — DelphyAI LAB
// For information: r.navoni74@gmail.com
// Pure-C float32 forward for the NNMixer MLP policy (baked normalizer + Gemm/ELU stack).
// Shared verbatim between the parity test (tools/parity_check.c) and ArduPilot (libraries/AP_NNMixer).
// No heap, no libm beyond expf. Deterministic; same arithmetic order on host and MCU.
#pragma once
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    uint16_t obs_dim;
    uint16_t act_dim;
    uint8_t n_layers;
    const uint16_t *dims;        // n_layers+1 entries: in, h1, ..., out
    const uint8_t *act;          // n_layers entries: 1 = ELU after layer
    const float *obs_mean;       // obs_dim
    const float *obs_std;        // obs_dim
    const float *const *W;       // n_layers pointers, W[i] is dims[i+1] x dims[i] row-major
    const float *const *b;       // n_layers pointers, b[i] is dims[i+1]
} nnmixer_policy_t;

// Maximum hidden width supported by the static scratch buffers.
#ifndef NNMIXER_MAX_WIDTH
#define NNMIXER_MAX_WIDTH 512
#endif

// obs[obs_dim] raw SI observations -> act[act_dim]. Returns 0 on success, -1 on shape error.
int nnmixer_forward(const nnmixer_policy_t *p, const float *obs, float *act);

// Int8 weights with per-row W scales and float32 bias (matches tools/robots/export_nnm.py).
// W[l] is dims[l+1] x dims[l] row-major int8; w_scale[l] has dims[l+1] floats.
#ifndef NNMIXER_MAX_LAYERS
#define NNMIXER_MAX_LAYERS 8
#endif

typedef struct {
    uint16_t obs_dim;
    uint16_t act_dim;
    uint8_t n_layers;
    uint8_t flags;               // bit0=int8, bit1=per-row scale
    const uint16_t *dims;        // n_layers+1
    const uint8_t *act;          // n_layers: 1 = ELU
    const float *obs_mean;
    const float *obs_std;
    const float *default_pose;   // act_dim (may be null)
    const int8_t *const *W;      // n_layers
    const float *const *w_scale; // n_layers, each dims[l+1]
    const float *const *b;       // n_layers, each dims[l+1] float32
} nnmixer_policy_int8_t;

int nnmixer_forward_int8(const nnmixer_policy_int8_t *p, const float *obs, float *act);

// Cartan Network + DiLU (arXiv:2505.24353), export-safe variant used in training:
// embed -> n_layers x CartanLinear (paint GEMM, left translation beta, fiber rotation theta,
// DiLU on the fiber except after the last layer) -> Euclidean readout -> head.
typedef struct {
    uint16_t obs_dim;
    uint16_t act_dim;
    uint16_t paint;              // fiber size q; manifold dim is q+1
    uint8_t n_layers;
    const float *obs_mean;       // obs_dim
    const float *obs_std;        // obs_dim
    const float *in_W;           // paint x obs_dim row-major
    const float *in_b;           // paint
    const float *const *W;       // n_layers, each paint x paint row-major
    const float *const *b;       // n_layers, each paint
    const float *const *beta;    // n_layers, each paint+1
    const float *const *theta;   // n_layers, each paint+1, unit norm
    const float *head_W;         // act_dim x paint row-major
    const float *head_b;         // act_dim
    float dilu_alpha;            // 0.1
} nnmixer_cartan_t;

int nnmixer_cartan_forward(const nnmixer_cartan_t *p, const float *obs, float *act);

#ifdef __cplusplus
}
#endif
