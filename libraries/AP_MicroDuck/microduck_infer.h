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

#ifdef __cplusplus
}
#endif
