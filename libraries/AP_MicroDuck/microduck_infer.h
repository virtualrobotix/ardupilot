// Pure-C float32 forward for the MicroDuck MLP policy (baked normalizer + Gemm/ELU stack).
// Shared verbatim between the parity test (tools/parity_check.c) and ArduPilot (libraries/AP_MicroDuck).
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
} microduck_policy_t;

// Maximum hidden width supported by the static scratch buffers.
#ifndef MICRODUCK_MAX_WIDTH
#define MICRODUCK_MAX_WIDTH 512
#endif

// obs[obs_dim] raw SI observations -> act[act_dim]. Returns 0 on success, -1 on shape error.
int microduck_forward(const microduck_policy_t *p, const float *obs, float *act);

// Cartan Network + DiLU (arXiv:2505.24353), export-safe variant used by mjlab_microduck:
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
} microduck_cartan_t;

int microduck_cartan_forward(const microduck_cartan_t *p, const float *obs, float *act);

#ifdef __cplusplus
}
#endif
