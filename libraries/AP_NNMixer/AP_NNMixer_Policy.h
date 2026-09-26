/*
  Author: Roberto Navoni, member of the ArduPilot Dev Team
  Contact: r.navoni74@gmail.com
  Developed by Roberto Navoni — DelphyAI LAB
  For information: r.navoni74@gmail.com

  SD-backed robot topology + dual int8 policy slots for AP_NNMixer.
*/
#pragma once

#include "AP_NNMixer_config.h"

#if AP_NNMIXER_ENABLED

#include "nnmixer_infer.h"
#include <stdint.h>

#ifndef NNM_MAX_JOINTS
#define NNM_MAX_JOINTS 16
#endif
#ifndef NNM_MAX_OBS
#define NNM_MAX_OBS 96
#endif
#ifndef NNMIXER_POLICY_SLOT_BYTES
#define NNMIXER_POLICY_SLOT_BYTES (220u * 1024u)
#endif
#ifndef NNMIXER_FLOAT_SCRATCH
// mean+std+pose + per-row scales/bias for a 512-wide MLP (~2k floats)
#define NNMIXER_FLOAT_SCRATCH 2048
#endif

// Stable boot indices — must match tools/robots/common.py ROBOT_INDEX
enum class NNM_RobotId : uint8_t {
    MICRODUCK = 0,
    MICROBAN = 1,
    ZEROTH = 2,
    BIMO = 3,
    LEGOLAS = 4,
    UPKIE = 5,
    REX = 6,
    YERTLE = 7,
    ALBERT = 8,
    COUNT = 9
};

const char *nnm_robot_name(NNM_RobotId id);

struct NNM_RobotTopology {
    char robot_id[16];
    uint16_t n_joints;
    uint16_t obs_dim;
    uint16_t rate_hz;
    uint16_t servo_fn0;
    float q0[NNM_MAX_JOINTS];
    bool valid;
};

// One RAM-resident int8 policy. blob owns the file bytes; view points into it.
struct NNM_PolicySlot {
    uint8_t *blob;
    uint32_t blob_len;
    uint32_t blob_cap;
    nnmixer_policy_int8_t view;
    uint16_t dims_copy[NNMIXER_MAX_LAYERS + 1];
    uint8_t act_copy[NNMIXER_MAX_LAYERS];
    const int8_t *W_ptrs[NNMIXER_MAX_LAYERS];
    const float *w_scale_ptrs[NNMIXER_MAX_LAYERS];
    const float *b_ptrs[NNMIXER_MAX_LAYERS];
    float float_scratch[NNMIXER_FLOAT_SCRATCH];
    char robot_id[16];
    bool ready;
};

bool nnm_parse_robot_bin(const uint8_t *data, uint32_t len, NNM_RobotTopology &out);

// Copy .nnm into slot.blob and build view. Rejects wrong robot_id / dims.
bool nnm_parse_policy_nnm(const uint8_t *data, uint32_t len,
                          const NNM_RobotTopology &topo,
                          NNM_PolicySlot &slot);

#endif // AP_NNMIXER_ENABLED
