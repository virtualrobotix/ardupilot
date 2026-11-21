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
// Microban has 18 DOF; leave headroom for similar bipeds
#define NNM_MAX_JOINTS 20
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
    OPENDUCK = 9,
    FREENOVE = 10,
    BITTLE = 11,
    BOOSTER_T1 = 12,
    FLYBODY = 13,
    YANSHEE = 14,
    TIENKUNG = 15,
    TODDLERBOT = 16,
    JUMPER = 17,
    PHONEBOT = 18,
    PHONEBOT_REV = 19,
    OPEN_SPRITE = 20,
    XGODUCK = 21,
    Q8BOT = 22,
    COUNT = 23
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

// Optional deployment table, /APM/nnm/<robot>/skills.bin (tools/robots/pack_robot_bin.py):
// limits of the pose-command channels, per-policy settings and timed skills.
#define NNM_MAX_POLICY_CFG 8
#define NNM_MAX_SKILLS 12

enum NNM_PolicyFlags : uint8_t {
    NNM_POLICY_ZERO_POSE = 1,       // pose-command channels forced to zero
    NNM_POLICY_NO_ACCX = 2,         // no accelerometer-x trim (NNM_DEPLOY 1)
};

enum NNM_SkillFlags : uint8_t {
    NNM_SKILL_CIRCLE = 1,           // twist = (cos, sin, 0) of the skill phase
    NNM_SKILL_ZERO_POSE = 2,
    NNM_SKILL_NO_GETUP = 4,         // automatic get-up suspended while the skill runs
    NNM_SKILL_KEEP_HISTORY = 8,     // action history kept on entry
    NNM_SKILL_AUX = 16,             // drives the auxiliary actuator
};

struct NNM_PolicyCfg {
    uint8_t policy;
    uint8_t flags;
    float alpha_leg;
    float alpha_head;
    float heading_kp;               // 0 = default
    float wz_max;                   // 0 = NNM_WZ_MAX
};

struct NNM_Skill {
    uint8_t id;
    uint8_t policy;
    uint8_t flags;
    uint8_t needs;                  // skill id that must be running, 0 = none
    float duration_s;               // 0 = until another skill replaces it
    float cmd[3];
    float aux_deg;
    float aux_until;                // aux_deg while phase < aux_until, then 0
};

// Gain table of the bus servos, skills.bin version 2 (tools/robots/pack_robot_bin.py). One entry per
// bus slot, in MCU order; bit i of servo_mask says slot i has a value (else the servo keeps its EEPROM).
#define NNM_SERVO_GAINS 15

struct NNM_Deploy {
    bool valid;
    bool servo_gains_valid;         // skills.bin version 2 carried a gain table
    uint16_t servo_mask;
    uint8_t n_servo_gains;
    uint8_t servo_kp[NNM_SERVO_GAINS];
    uint8_t servo_kd[NNM_SERVO_GAINS];
    uint8_t head_first;             // joints [head_first, head_first + head_count) use alpha_head
    uint8_t head_count;
    float alpha_leg;
    float alpha_head;
    float pose_limit[8];
    float aux_max;
    uint8_t n_policy;
    uint8_t n_skill;
    NNM_PolicyCfg policy[NNM_MAX_POLICY_CFG];
    NNM_Skill skill[NNM_MAX_SKILLS];
};

bool nnm_parse_skills_bin(const uint8_t *data, uint32_t len, NNM_Deploy &out);

// Copy .nnm into slot.blob and build view. Rejects wrong robot_id / dims.
bool nnm_parse_policy_nnm(const uint8_t *data, uint32_t len,
                          const NNM_RobotTopology &topo,
                          NNM_PolicySlot &slot);

#endif // AP_NNMIXER_ENABLED
