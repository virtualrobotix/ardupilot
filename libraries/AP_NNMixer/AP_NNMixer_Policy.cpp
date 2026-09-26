/*
  Author: Roberto Navoni, member of the ArduPilot Dev Team
  Contact: r.navoni74@gmail.com
  Developed by Roberto Navoni — DelphyAI LAB
  For information: r.navoni74@gmail.com
*/
#include "AP_NNMixer_config.h"

#if AP_NNMIXER_ENABLED

#include "AP_NNMixer_Policy.h"
#include <string.h>

const char *nnm_robot_name(NNM_RobotId id)
{
    switch (id) {
    case NNM_RobotId::MICRODUCK: return "microduck";
    case NNM_RobotId::MICROBAN:  return "microban";
    case NNM_RobotId::ZEROTH:    return "zeroth";
    case NNM_RobotId::BIMO:      return "bimo";
    case NNM_RobotId::LEGOLAS:   return "legolas";
    case NNM_RobotId::UPKIE:     return "upkie";
    case NNM_RobotId::REX:       return "rex";
    case NNM_RobotId::YERTLE:    return "yertle";
    case NNM_RobotId::ALBERT:    return "albert";
    default:                     return nullptr;
    }
}

static uint16_t rd_u16(const uint8_t *p)
{
    return uint16_t(p[0]) | (uint16_t(p[1]) << 8);
}

static float rd_f32(const uint8_t *p)
{
    float f;
    memcpy(&f, p, 4);
    return f;
}

bool nnm_parse_robot_bin(const uint8_t *data, uint32_t len, NNM_RobotTopology &out)
{
    memset(&out, 0, sizeof(out));
    if (data == nullptr || len < 4 + 16 + 8) {
        return false;
    }
    if (memcmp(data, "NNMR", 4) != 0) {
        return false;
    }
    memcpy(out.robot_id, data + 4, 15);
    out.robot_id[15] = 0;
    const uint8_t *p = data + 20;
    out.n_joints = rd_u16(p); p += 2;
    out.obs_dim = rd_u16(p); p += 2;
    out.rate_hz = rd_u16(p); p += 2;
    out.servo_fn0 = rd_u16(p); p += 2;
    if (out.n_joints == 0 || out.n_joints > NNM_MAX_JOINTS) {
        return false;
    }
    if (out.obs_dim == 0 || out.obs_dim > NNM_MAX_OBS) {
        return false;
    }
    if (uint32_t(p - data) + out.n_joints * 4u > len) {
        return false;
    }
    for (uint16_t i = 0; i < out.n_joints; i++) {
        out.q0[i] = rd_f32(p);
        p += 4;
    }
    out.valid = true;
    return true;
}

bool nnm_parse_policy_nnm(const uint8_t *data, uint32_t len,
                          const NNM_RobotTopology &topo,
                          NNM_PolicySlot &slot)
{
    slot.ready = false;
    memset(&slot.view, 0, sizeof(slot.view));
    if (data == nullptr || len < 28 || slot.blob == nullptr || slot.blob_cap < len) {
        return false;
    }
    if (memcmp(data, "NNM1", 4) != 0) {
        return false;
    }
    char rid[16];
    memcpy(rid, data + 4, 15);
    rid[15] = 0;
    if (strncmp(rid, topo.robot_id, 16) != 0) {
        return false;
    }

    const uint8_t *hdr = data + 20;
    const uint16_t obs_dim = rd_u16(hdr); hdr += 2;
    const uint16_t act_dim = rd_u16(hdr); hdr += 2;
    const uint8_t n_layers = *hdr++;
    const uint8_t flags = *hdr++;
    hdr += 2;
    if ((flags & 0x03) != 0x03) {
        return false;
    }
    if (n_layers == 0 || n_layers > NNMIXER_MAX_LAYERS) {
        return false;
    }
    if (obs_dim != topo.obs_dim || act_dim != topo.n_joints) {
        return false;
    }
    if (act_dim > NNM_MAX_JOINTS || obs_dim > NNM_MAX_OBS) {
        return false;
    }

    const uint8_t *dims_src = data + 28;
    uint32_t need = 28;
    need += (n_layers + 1) * 2u + n_layers;
    need += obs_dim * 4u * 2u + act_dim * 4u;
    for (uint8_t l = 0; l < n_layers; l++) {
        const uint16_t n_in = rd_u16(dims_src + l * 2);
        const uint16_t n_out = rd_u16(dims_src + (l + 1) * 2);
        need += n_out * 4u;               // w_scale
        need += n_out * 4u;               // bias
        need += uint32_t(n_out) * n_in;   // W int8
    }
    if (need > len) {
        return false;
    }

    // Aligned float scratch lives on the slot (not carved from the blob).
    uint32_t float_count = uint32_t(obs_dim) * 2u + act_dim;
    for (uint8_t l = 0; l < n_layers; l++) {
        float_count += 2u * rd_u16(dims_src + (l + 1) * 2);
    }
    if (float_count > NNMIXER_FLOAT_SCRATCH) {
        return false;
    }

    memcpy(slot.blob, data, len);
    slot.blob_len = len;
    memcpy(slot.robot_id, rid, sizeof(slot.robot_id));

    const uint8_t *base = slot.blob;
    const uint8_t *p = base + 28;
    for (uint8_t i = 0; i < n_layers + 1; i++) {
        slot.dims_copy[i] = rd_u16(p);
        p += 2;
    }
    for (uint8_t i = 0; i < n_layers; i++) {
        slot.act_copy[i] = *p++;
    }

    float *cursor = slot.float_scratch;

    for (uint16_t i = 0; i < obs_dim; i++, p += 4) {
        cursor[i] = rd_f32(p);
    }
    const float *obs_mean = cursor;
    cursor += obs_dim;
    for (uint16_t i = 0; i < obs_dim; i++, p += 4) {
        cursor[i] = rd_f32(p);
    }
    const float *obs_std = cursor;
    cursor += obs_dim;
    for (uint16_t i = 0; i < act_dim; i++, p += 4) {
        cursor[i] = rd_f32(p);
    }
    const float *default_pose = cursor;
    cursor += act_dim;

    for (uint8_t l = 0; l < n_layers; l++) {
        const uint16_t n_in = slot.dims_copy[l];
        const uint16_t n_out = slot.dims_copy[l + 1];
        for (uint16_t i = 0; i < n_out; i++, p += 4) {
            cursor[i] = rd_f32(p);
        }
        slot.w_scale_ptrs[l] = cursor;
        cursor += n_out;
        for (uint16_t i = 0; i < n_out; i++, p += 4) {
            cursor[i] = rd_f32(p);
        }
        slot.b_ptrs[l] = cursor;
        cursor += n_out;
        slot.W_ptrs[l] = (const int8_t *)p;
        p += uint32_t(n_out) * n_in;
    }
    if (uint32_t(p - base) > len) {
        return false;
    }

    slot.view.obs_dim = obs_dim;
    slot.view.act_dim = act_dim;
    slot.view.n_layers = n_layers;
    slot.view.flags = flags;
    slot.view.dims = slot.dims_copy;
    slot.view.act = slot.act_copy;
    slot.view.obs_mean = obs_mean;
    slot.view.obs_std = obs_std;
    slot.view.default_pose = default_pose;
    slot.view.W = slot.W_ptrs;
    slot.view.w_scale = slot.w_scale_ptrs;
    slot.view.b = slot.b_ptrs;
    slot.ready = true;
    return true;
}

#endif // AP_NNMIXER_ENABLED
