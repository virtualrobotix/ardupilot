/*
  Author: Roberto Navoni, member of the ArduPilot Dev Team
  Contact: r.navoni74@gmail.com
  Developed by Roberto Navoni — DelphyAI LAB
  For information: r.navoni74@gmail.com
 */
#include "AP_NNMixer_config.h"

#if AP_NNMIXER_ENABLED

#include "AP_NNMixer.h"
#include "nnmixer_infer.h"
#if AP_NNMIXER_BAKED_MLP_ENABLED
#include "policy_mlp.h"
#endif
#if AP_NNMIXER_CARTAN_ENABLED
#include "policy_cartan.h"
#endif

#include <AP_HAL/AP_HAL.h>
#include <AP_InertialSensor/AP_InertialSensor.h>
#include <AP_AHRS/AP_AHRS.h>
#include <AP_Logger/AP_Logger.h>
#include <AP_Vehicle/AP_Vehicle.h>
#include <GCS_MAVLink/GCS.h>
#include <RC_Channel/RC_Channel.h>
#include <SRV_Channel/SRV_Channel.h>
#include <AP_Filesystem/AP_Filesystem.h>

#include <stdlib.h>
#include <string.h>

#if AP_NNMIXER_JOINT_FEEDBACK_SITL_ENABLED
#include <SITL/SITL.h>
#endif
#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
#include <time.h>
#endif


extern const AP_HAL::HAL& hal;

#define NNM_PWM_CENTER 1500
#define NNM_RAD_PER_US 0.003f
#define NNM_PWM_MIN 800
#define NNM_PWM_MAX 2200

// Rover mode numbers (Mode::Number) — keep in sync with Rover/mode.h
static const uint8_t kModeManual = 0;
static const uint8_t kModeHold = 4;
static const uint8_t kModeAuto = 10;
static const uint8_t kModeRtl = 11;
static const uint8_t kModeSmartRtl = 12;
static const uint8_t kModeGuided = 15;

const AP_Param::GroupInfo AP_NNMixer::var_info[] = {
    AP_GROUPINFO_FLAGS("ENABLE", 1, AP_NNMixer, _enable, 0, AP_PARAM_FLAG_ENABLE),

    // @Param: ROBOT
    // @DisplayName: Robot topology (boot)
    // @Description: Selects /APM/nnm/<name>/robot.bin. Requires reboot. 0 MicroDuck, 1 Microban, 2 Zeroth, 3 Bimo, 4 Legolas, 5 Upkie, 6 Rex, 7 Yertle
    // @Values: 0:MicroDuck,1:Microban,2:Zeroth,3:Bimo,4:Legolas,5:Upkie,6:Rex,7:Yertle
    // @User: Advanced
    // @RebootRequired: True
    AP_GROUPINFO("ROBOT", 17, AP_NNMixer, _robot, 0),

    // @Param: POLICY
    // @DisplayName: Policy file index for the boot robot
    // @Description: Index into /APM/nnm/<robot>/policies/*.nnm (alphabetical). Switch copies SD→RAM into the free int8 slot. Rejects files whose robot_id does not match the boot robot.
    // @Range: 0 7
    // @User: Advanced
    AP_GROUPINFO("POLICY", 2, AP_NNMixer, _policy, 0),

    AP_GROUPINFO("VX_MAX", 3, AP_NNMixer, _vx_max, 0.4f),
    AP_GROUPINFO("VY_MAX", 4, AP_NNMixer, _vy_max, 0.3f),
    AP_GROUPINFO("WZ_MAX", 5, AP_NNMixer, _wz_max, 1.0f),
    AP_GROUPINFO("WD_MS", 6, AP_NNMixer, _wd_ms, 40),
    AP_GROUPINFO("ATT_SRC", 7, AP_NNMixer, _att_src, 0),
    AP_GROUPINFO("ATT_TAU", 8, AP_NNMixer, _att_tau, 0.5f),
    AP_GROUPINFO("RC_VX", 9, AP_NNMixer, _rc_vx, 2),
    AP_GROUPINFO("RC_VY", 10, AP_NNMixer, _rc_vy, 1),
    AP_GROUPINFO("RC_WZ", 11, AP_NNMixer, _rc_wz, 4),
    AP_GROUPINFO("ACT_MAX", 12, AP_NNMixer, _act_max, 2.0f),
    AP_GROUPINFO("LOG", 13, AP_NNMixer, _log, 1),
    AP_GROUPINFO("HOLD_MODE", 14, AP_NNMixer, _hold_mode, 4),
    AP_GROUPINFO("SRV_FN0", 15, AP_NNMixer, _servo_fn0, 94),
    AP_GROUPINFO("HIL_ATT", 16, AP_NNMixer, _hil_att, 0),

    // @Param: BLEND_MS
    // @DisplayName: Policy switch blend time
    // @Units: ms
    // @User: Advanced
    AP_GROUPINFO("BLEND_MS", 18, AP_NNMixer, _blend_ms, 500),

    AP_GROUPEND
};

AP_NNMixer *AP_NNMixer::_singleton;

static uint32_t wall_micros()
{
#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return uint32_t(uint64_t(ts.tv_sec) * 1000000ULL + uint64_t(ts.tv_nsec) / 1000ULL);
#else
    return AP_HAL::micros();
#endif
}

AP_NNMixer::AP_NNMixer()
{
    if (_singleton != nullptr) {
        AP_HAL::panic("AP_NNMixer must be singleton");
    }
    _singleton = this;
    AP_Param::setup_object_defaults(this, var_info);
    memset(_nav_twist, 0, sizeof(_nav_twist));
    _active_slot = 0;
    _use_sd_policy = false;
    _use_baked_mlp = false;
    _loaded_policy_idx = -1;
    _pending_policy_idx = -1;
    _blending = false;
}

void AP_NNMixer::set_nav_twist(float vx, float vy, float wz)
{
    _nav_twist[0] = vx;
    _nav_twist[1] = vy;
    _nav_twist[2] = wz;
    _nav_twist_ms = AP_HAL::millis();
}

bool AP_NNMixer::ensure_slots_allocated()
{
    for (uint8_t i = 0; i < 2; i++) {
        if (_slot[i].blob != nullptr) {
            continue;
        }
        _slot[i].blob = (uint8_t *)malloc(NNMIXER_POLICY_SLOT_BYTES);
        if (_slot[i].blob == nullptr) {
            GCS_SEND_TEXT(MAV_SEVERITY_ERROR, "NNMixer: slot %u malloc failed", unsigned(i));
            return false;
        }
        _slot[i].blob_cap = NNMIXER_POLICY_SLOT_BYTES;
        _slot[i].blob_len = 0;
        _slot[i].ready = false;
    }
    return true;
}

bool AP_NNMixer::load_robot_topology()
{
    const char *name = nnm_robot_name(NNM_RobotId(_robot.get()));
    if (name == nullptr) {
        return false;
    }
    char path[64];
    hal.util->snprintf(path, sizeof(path), "APM/nnm/%s/robot.bin", name);
    auto *fd = AP::FS().load_file(path);
    if (fd == nullptr) {
        // SITL / alternate mount
        hal.util->snprintf(path, sizeof(path), "/APM/nnm/%s/robot.bin", name);
        fd = AP::FS().load_file(path);
    }
    if (fd == nullptr) {
#if AP_NNMIXER_BAKED_MLP_ENABLED
        if (_robot == 0) {
            // MicroDuck baked fallback when SD has no robot.bin
            memset(&_topo, 0, sizeof(_topo));
            strncpy(_topo.robot_id, "microduck", sizeof(_topo.robot_id) - 1);
            _topo.n_joints = MLP_ACT_DIM;
            _topo.obs_dim = MLP_OBS_DIM;
            _topo.rate_hz = 50;
            _topo.servo_fn0 = uint16_t(_servo_fn0.get());
            for (uint16_t i = 0; i < _topo.n_joints && i < NNM_MAX_JOINTS; i++) {
                _topo.q0[i] = mlp_default_pose[i];
            }
            _topo.valid = true;
            _use_baked_mlp = true;
            _use_sd_policy = false;
            GCS_SEND_TEXT(MAV_SEVERITY_INFO, "NNMixer: baked MicroDuck topology (no SD robot.bin)");
            return true;
        }
#endif
        GCS_SEND_TEXT(MAV_SEVERITY_ERROR, "NNMixer: missing %s", path);
        return false;
    }
    const bool ok = nnm_parse_robot_bin(fd->data, fd->length, _topo);
    delete fd;
    if (!ok) {
        GCS_SEND_TEXT(MAV_SEVERITY_ERROR, "NNMixer: bad robot.bin for %s", name);
        return false;
    }
    // Param servo offset may override the file
    if (_servo_fn0.get() > 0) {
        _topo.servo_fn0 = uint16_t(_servo_fn0.get());
    }
    GCS_SEND_TEXT(MAV_SEVERITY_INFO, "NNMixer: robot %s joints=%u obs=%u",
                  _topo.robot_id, unsigned(_topo.n_joints), unsigned(_topo.obs_dim));
    return true;
}

bool AP_NNMixer::load_policy_index(int8_t index, uint8_t into_slot)
{
    if (!_topo.valid || into_slot > 1) {
        return false;
    }
    if (!ensure_slots_allocated()) {
        return false;
    }
    const char *name = _topo.robot_id;
    char dirpath[64];
    hal.util->snprintf(dirpath, sizeof(dirpath), "APM/nnm/%s/policies", name);

    auto *dir = AP::FS().opendir(dirpath);
    if (dir == nullptr) {
        hal.util->snprintf(dirpath, sizeof(dirpath), "/APM/nnm/%s/policies", name);
        dir = AP::FS().opendir(dirpath);
    }
    if (dir == nullptr) {
#if AP_NNMIXER_BAKED_MLP_ENABLED
        if (_robot == 0 && index == 0) {
            _use_baked_mlp = true;
            _use_sd_policy = false;
            _loaded_policy_idx = 0;
            GCS_SEND_TEXT(MAV_SEVERITY_INFO, "NNMixer: using baked MLP (no SD policies)");
            return true;
        }
#endif
        GCS_SEND_TEXT(MAV_SEVERITY_ERROR, "NNMixer: no policies dir %s", dirpath);
        return false;
    }

    // Collect .nnm names (alphabetical via insertion into small list)
    char names[8][32] {};
    uint8_t nfiles = 0;
    for (struct dirent *de = AP::FS().readdir(dir); de && nfiles < 8; de = AP::FS().readdir(dir)) {
        const char *fn = de->d_name;
        const size_t len = strlen(fn);
        if (len < 5 || len >= 32) {
            continue;
        }
        if (strcmp(fn + len - 4, ".nnm") != 0) {
            continue;
        }
        // insert sorted
        uint8_t at = nfiles;
        while (at > 0 && strcmp(names[at - 1], fn) > 0) {
            memcpy(names[at], names[at - 1], 32);
            at--;
        }
        strncpy(names[at], fn, 31);
        nfiles++;
    }
    AP::FS().closedir(dir);

    if (nfiles == 0) {
#if AP_NNMIXER_BAKED_MLP_ENABLED
        if (_robot == 0 && index == 0) {
            _use_baked_mlp = true;
            _use_sd_policy = false;
            _loaded_policy_idx = 0;
            return true;
        }
#endif
        return false;
    }
    if (index < 0 || index >= int8_t(nfiles)) {
        GCS_SEND_TEXT(MAV_SEVERITY_ERROR, "NNMixer: policy index %d out of range 0..%u",
                      int(index), unsigned(nfiles - 1));
        return false;
    }

    char path[96];
    hal.util->snprintf(path, sizeof(path), "%s/%s", dirpath, names[index]);
    auto *fd = AP::FS().load_file(path);
    if (fd == nullptr) {
        GCS_SEND_TEXT(MAV_SEVERITY_ERROR, "NNMixer: cannot read %s", path);
        return false;
    }
    const bool ok = nnm_parse_policy_nnm(fd->data, fd->length, _topo, _slot[into_slot]);
    delete fd;
    if (!ok) {
        GCS_SEND_TEXT(MAV_SEVERITY_ERROR, "NNMixer: rejected %s (robot/dims mismatch)", path);
        return false;
    }
    _use_sd_policy = true;
    _use_baked_mlp = false;
    GCS_SEND_TEXT(MAV_SEVERITY_INFO, "NNMixer: loaded %s into slot %u", path, unsigned(into_slot));
    return true;
}

void AP_NNMixer::request_policy_switch(int8_t index)
{
    if (index == _loaded_policy_idx && _use_sd_policy) {
        return;
    }
    _pending_policy_idx = index;
}

void AP_NNMixer::service_policy_switch()
{
    if (_pending_policy_idx < 0) {
        return;
    }
    // Only switch while disarmed, HOLD, or near-zero twist (safe)
    const AP_Vehicle *veh = AP::vehicle();
    const uint8_t mode = (veh != nullptr) ? veh->get_mode() : kModeHold;
    const bool disarmed = !hal.util->get_soft_armed();
    const bool hold = (_hold_mode >= 0 && mode == uint8_t(_hold_mode)) || mode == kModeHold;
    if (!disarmed && !hold) {
        // defer
        return;
    }
    const uint8_t free_slot = _active_slot ^ 1;
    const int8_t idx = _pending_policy_idx;
    if (!load_policy_index(idx, free_slot)) {
        _pending_policy_idx = -1;
        return;
    }
    // Start blend from current action toward new policy
    memcpy(_act_blend_from, _last_action, sizeof(_act_blend_from));
    _blend_start_ms = AP_HAL::millis();
    _blending = true;
    _active_slot = free_slot;
    _loaded_policy_idx = idx;
    _pending_policy_idx = -1;
}

void AP_NNMixer::init()
{
    memset(_last_action, 0, sizeof(_last_action));
    _down_body = Vector3f(0, 0, 1);
    _down_valid = false;
    _joint_count = 0;
    _joint_ms = 0;
    _hil_state_ms = 0;
    memset(_hil_gyro_flu, 0, sizeof(_hil_gyro_flu));
    memset(_hil_gravity_flu, 0, sizeof(_hil_gravity_flu));
    _tick = 0;
    _fail_reason = 5;
    _forward_us = 0;
    _last_telem_ms = 0;
    memset(_obs, 0, sizeof(_obs));
    memset(_act, 0, sizeof(_act));
    _blending = false;

    if (!load_robot_topology()) {
        _initialised = true;
        GCS_SEND_TEXT(MAV_SEVERITY_ERROR, "NNMixer: init failed (no topology)");
        return;
    }
    if (_use_baked_mlp) {
        _loaded_policy_idx = 0;
    } else if (load_policy_index(_policy.get(), 0)) {
        _active_slot = 0;
        _loaded_policy_idx = _policy.get();
    } else {
#if AP_NNMIXER_BAKED_MLP_ENABLED
        if (_robot == 0) {
            _use_baked_mlp = true;
            _loaded_policy_idx = 0;
            GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "NNMixer: falling back to baked MLP");
        }
#endif
    }
    _initialised = true;
    GCS_SEND_TEXT(MAV_SEVERITY_INFO, "NNMixer: ready robot=%s policy=%d sd=%d baked=%d",
                  _topo.robot_id, int(_loaded_policy_idx),
                  int(_use_sd_policy), int(_use_baked_mlp));
}

const float *AP_NNMixer::default_pose() const
{
    if (_use_sd_policy && _slot[_active_slot].ready && _slot[_active_slot].view.default_pose != nullptr) {
        return _slot[_active_slot].view.default_pose;
    }
    if (_topo.valid) {
        return _topo.q0;
    }
#if AP_NNMIXER_BAKED_MLP_ENABLED
    return mlp_default_pose;
#else
    return _topo.q0;
#endif
}

void AP_NNMixer::update_attitude()
{
    if (!enabled()) {
        return;
    }
    if (!_initialised) {
        init();
    }
    const AP_InertialSensor &ins = AP::ins();
    const Vector3f gyro = ins.get_gyro();
    const Vector3f accel = ins.get_accel();
    const float dt = ins.get_loop_delta_t();
    const float an = accel.length();
    if (!_down_valid) {
        if (an > 1.0f) {
            _down_body = -accel / an;
            _down_valid = true;
        }
        return;
    }
    Vector3f d = _down_body - (gyro % _down_body) * dt;
    if (an > 0.5f * GRAVITY_MSS && an < 1.5f * GRAVITY_MSS && _att_tau > 0.0f) {
        const float alpha = constrain_float(dt / _att_tau, 0.0f, 1.0f);
        d = d * (1.0f - alpha) + (-accel / an) * alpha;
    }
    const float n = d.length();
    if (n > 1e-6f) {
        _down_body = d / n;
    }
}

void AP_NNMixer::gravity_body_flu(float g[3])
{
    Vector3f down = _down_body;
    if (_att_src == 1) {
        Quaternion q;
        AP::ahrs().get_quat_body_to_ned(q);
        Matrix3f m;
        q.rotation_matrix(m);
        down = m.mul_transpose(Vector3f(0, 0, 1));
    }
    g[0] = down.x;
    g[1] = -down.y;
    g[2] = -down.z;
}

void AP_NNMixer::set_joint_feedback(const float *pos, const float *vel, uint8_t count)
{
    WITH_SEMAPHORE(_joint_sem);
    const uint8_t nmax = _topo.valid ? uint8_t(_topo.n_joints) : uint8_t(NNM_MAX_JOINTS);
    const uint8_t n = MIN(count, nmax);
    for (uint8_t i = 0; i < n; i++) {
        _joint_pos[i] = pos[i];
        _joint_vel[i] = vel[i];
    }
    _joint_count = n;
    _joint_ms = AP_HAL::millis();
}

void AP_NNMixer::set_hil_state(const float *pos, const float *vel,
                                 const float *gyro_flu, const float *gravity_flu)
{
    const uint8_t n = _topo.valid ? uint8_t(_topo.n_joints) : 14;
    set_joint_feedback(pos, vel, n);
    WITH_SEMAPHORE(_joint_sem);
    memcpy(_hil_gyro_flu, gyro_flu, sizeof(_hil_gyro_flu));
    memcpy(_hil_gravity_flu, gravity_flu, sizeof(_hil_gravity_flu));
    _hil_state_ms = AP_HAL::millis();
}

bool AP_NNMixer::read_joint_feedback()
{
    const uint8_t need = _topo.valid ? uint8_t(_topo.n_joints) : 14;
#if AP_NNMIXER_JOINT_FEEDBACK_SITL_ENABLED
    const SITL::SIM *sitl = AP::sitl();
    if (sitl != nullptr && sitl->state.joint_count >= need &&
        sitl->state.joint_time_us != _last_joint_time_us) {
        _last_joint_time_us = sitl->state.joint_time_us;
        set_joint_feedback(sitl->state.joint_pos, sitl->state.joint_vel, need);
    }
#endif
    WITH_SEMAPHORE(_joint_sem);
    if (_joint_count < need) {
        _fail_reason = 2;
        return false;
    }
    if (AP_HAL::millis() - _joint_ms > uint32_t(MAX(_wd_ms.get(), 1))) {
        _fail_reason = 3;
        return false;
    }
    return true;
}

void AP_NNMixer::read_twist(float twist[3])
{
    twist[0] = twist[1] = twist[2] = 0.0f;
    const AP_Vehicle *veh = AP::vehicle();
    const uint8_t mode = (veh != nullptr) ? veh->get_mode() : kModeManual;

    if (_hold_mode >= 0 && mode == uint8_t(_hold_mode)) {
        return;
    }
    if (mode == kModeHold) {
        return;
    }

    // Autopilot modes: use nav twist published by Rover (desired speed / turn rate)
    if (mode == kModeGuided || mode == kModeAuto || mode == kModeRtl || mode == kModeSmartRtl) {
        if (_nav_twist_ms != 0 && AP_HAL::millis() - _nav_twist_ms < 500) {
            twist[0] = constrain_float(_nav_twist[0], -_vx_max, _vx_max);
            twist[1] = constrain_float(_nav_twist[1], -_vy_max, _vy_max);
            twist[2] = constrain_float(_nav_twist[2], -_wz_max, _wz_max);
        }
        return;
    }

    // MANUAL (and other stick-driven modes): RC
    if (!rc().has_valid_input()) {
        return;
    }
    const struct {
        int8_t ch;
        float scale;
        float *dst;
    } map[3] = {
        { int8_t(_rc_vx), _vx_max, &twist[0] },
        { int8_t(_rc_vy), _vy_max, &twist[1] },
        { int8_t(_rc_wz), _wz_max, &twist[2] },
    };
    for (uint8_t i = 0; i < 3; i++) {
        if (map[i].ch < 1) {
            continue;
        }
        const RC_Channel *c = rc().channel(uint8_t(map[i].ch - 1));
        if (c == nullptr) {
            continue;
        }
        const float n = c->norm_input_dz();
        *map[i].dst = constrain_float(n, -1.0f, 1.0f) * map[i].scale;
    }
}

void AP_NNMixer::write_servos(const float *q_target)
{
    const uint8_t n = _topo.valid ? uint8_t(_topo.n_joints) : 0;
    const uint16_t fn0 = _topo.valid ? _topo.servo_fn0 : uint16_t(_servo_fn0.get());
    for (uint8_t i = 0; i < n; i++) {
        const int32_t pwm = NNM_PWM_CENTER + int32_t(roundf(q_target[i] / NNM_RAD_PER_US));
        const uint16_t p = uint16_t(constrain_int32(pwm, NNM_PWM_MIN, NNM_PWM_MAX));
        SRV_Channels::set_output_pwm(SRV_Channel::Function(fn0 + i), p);
    }
}

void AP_NNMixer::write_idle_servos()
{
    const uint8_t n = _topo.valid ? uint8_t(_topo.n_joints) : 0;
    const uint16_t fn0 = _topo.valid ? _topo.servo_fn0 : uint16_t(_servo_fn0.get());
    for (uint8_t i = 0; i < n; i++) {
        SRV_Channels::set_output_pwm(SRV_Channel::Function(fn0 + i), 0);
    }
}

void AP_NNMixer::update()
{
    if (!enabled()) {
        return;
    }
    if (!_initialised) {
        init();
    }
    _tick++;

    // Hot policy index change
    if (_policy.get() != _loaded_policy_idx && _pending_policy_idx < 0) {
        request_policy_switch(_policy.get());
    }
    service_policy_switch();

    if (!_topo.valid) {
        _fail_reason = 5;
        return;
    }

    if (!hal.util->get_soft_armed()) {
        memset(_last_action, 0, sizeof(_last_action));
        _fail_reason = 1;
        write_idle_servos();
        send_telemetry();
        return;
    }

    float twist[3];
    read_twist(twist);

    const bool joints_ok = read_joint_feedback();
    const bool hil_state_ok = _hil_state_ms != 0 &&
                              AP_HAL::millis() - _hil_state_ms <= uint32_t(MAX(_wd_ms.get(), 1));
    const AP_InertialSensor &ins = AP::ins();
    const Vector3f gyro = ins.get_gyro();
    const uint16_t nj = _topo.n_joints;
    const uint16_t od = _topo.obs_dim;

    float *o = _obs;
    memset(o, 0, sizeof(_obs));
    const bool use_hil_att = hil_state_ok && _hil_att != 1;
    if (use_hil_att) {
        WITH_SEMAPHORE(_joint_sem);
        memcpy(&o[0], _hil_gyro_flu, sizeof(_hil_gyro_flu));
        memcpy(&o[3], _hil_gravity_flu, sizeof(_hil_gravity_flu));
    } else {
        o[0] = gyro.x;  o[1] = -gyro.y;  o[2] = -gyro.z;
        gravity_body_flu(&o[3]);
    }
    if (use_hil_att && _hil_att == 2) {
        float board_g[3];
        gravity_body_flu(board_g);
        o[0] += gyro.x;  o[1] += -gyro.y;  o[2] += -gyro.z;
        o[3] += board_g[0];
        o[4] += board_g[1];
        o[5] += board_g[2] + 1.0f;
        const float n = sqrtf(o[3] * o[3] + o[4] * o[4] + o[5] * o[5]);
        if (n > 1e-6f) {
            o[3] /= n;  o[4] /= n;  o[5] /= n;
        }
    }
    const float *q0 = default_pose();
    {
        WITH_SEMAPHORE(_joint_sem);
        for (uint16_t i = 0; i < nj; i++) {
            o[6 + i] = _joint_pos[i] - q0[i];
            o[6 + nj + i] = _joint_vel[i];
        }
    }
    for (uint16_t i = 0; i < nj; i++) {
        o[6 + 2 * nj + i] = _last_action[i];
    }
    const uint16_t twist_off = 6 + 3 * nj;
    if (twist_off + 3 <= od) {
        o[twist_off] = twist[0];
        o[twist_off + 1] = twist[1];
        o[twist_off + 2] = twist[2];
    }

    bool ok = joints_ok && (use_hil_att || _down_valid) && (_use_sd_policy || _use_baked_mlp);
    if (ok) {
        const uint32_t t0 = wall_micros();
        if (_use_sd_policy && _slot[_active_slot].ready) {
            ok = nnmixer_forward_int8(&_slot[_active_slot].view, _obs, _act) == 0;
#if AP_NNMIXER_BAKED_MLP_ENABLED
        } else if (_use_baked_mlp) {
            static const nnmixer_policy_t mlp_policy = {
                MLP_OBS_DIM, MLP_ACT_DIM, MLP_N_LAYERS,
                mlp_dims, mlp_act, mlp_obs_mean, mlp_obs_std, mlp_W, mlp_b,
            };
            ok = nnmixer_forward(&mlp_policy, _obs, _act) == 0;
#endif
        } else {
            ok = false;
        }
        _forward_us = wall_micros() - t0;
        if (!ok) {
            _fail_reason = 4;
        }
    }
    if (ok) {
        _fail_reason = 0;
        for (uint16_t i = 0; i < nj; i++) {
            _act[i] = constrain_float(_act[i], -_act_max, _act_max);
        }
        if (_blending) {
            const uint32_t elapsed = AP_HAL::millis() - _blend_start_ms;
            const uint32_t dur = uint32_t(MAX(_blend_ms.get(), 1));
            float a = constrain_float(float(elapsed) / float(dur), 0.0f, 1.0f);
            for (uint16_t i = 0; i < nj; i++) {
                _act[i] = _act_blend_from[i] * (1.0f - a) + _act[i] * a;
            }
            if (a >= 1.0f) {
                _blending = false;
            }
        }
    } else {
        memset(_act, 0, sizeof(_act));
    }

    float q_target[NNM_MAX_JOINTS];
    for (uint16_t i = 0; i < nj; i++) {
        q_target[i] = q0[i] + _act[i];
        _last_action[i] = _act[i];
    }
    write_servos(q_target);

    if (hil_state_ok) {
        static float hil_action[58] {};
        static char hil_name[10] = "NNM_ACT";
        memcpy(hil_action, q_target, nj * sizeof(float));
        mavlink_msg_debug_float_array_send(MAVLINK_COMM_0, AP_HAL::micros64(),
                                           hil_name, uint16_t(_tick), hil_action);
    }

    log_tick(twist);
    send_telemetry();
}

void AP_NNMixer::log_tick(const float twist[3])
{
#if HAL_LOGGING_ENABLED
    if (_log == 0) {
        return;
    }
    const uint64_t now = AP_HAL::micros64();
    const float *o = _obs;
    AP::logger().WriteStreaming("NNM", "TimeUS,Tick,Fail,FwdUS,GX,GY,GZ,PGX,PGY,PGZ,VX,VY,WZ",
                                "QIBIfffffffff",
                                now, _tick, _fail_reason, _forward_us,
                                o[0], o[1], o[2], o[3], o[4], o[5], twist[0], twist[1], twist[2]);
    if (_topo.n_joints >= 14) {
        AP::logger().WriteStreaming("NNMQ", "TimeUS,Q0,Q1,Q2,Q3,Q4,Q5,Q6,Q7,Q8,Q9,Q10,Q11,Q12,Q13",
                                    "Qffffffffffffff",
                                    now, o[6], o[7], o[8], o[9], o[10], o[11], o[12], o[13], o[14], o[15], o[16], o[17], o[18], o[19]);
        AP::logger().WriteStreaming("NNMV", "TimeUS,V0,V1,V2,V3,V4,V5,V6,V7,V8,V9,V10,V11,V12,V13",
                                    "Qffffffffffffff",
                                    now, o[20], o[21], o[22], o[23], o[24], o[25], o[26], o[27], o[28], o[29], o[30], o[31], o[32], o[33]);
        AP::logger().WriteStreaming("NNMA", "TimeUS,A0,A1,A2,A3,A4,A5,A6,A7,A8,A9,A10,A11,A12,A13",
                                    "Qffffffffffffff",
                                    now, _act[0], _act[1], _act[2], _act[3], _act[4], _act[5], _act[6], _act[7], _act[8], _act[9], _act[10], _act[11], _act[12], _act[13]);
    }
#endif
}

void AP_NNMixer::send_telemetry()
{
    const uint32_t now = AP_HAL::millis();
    if (now - _last_telem_ms < 500) {
        return;
    }
    _last_telem_ms = now;
    gcs().send_named_float("PPO_MS", _forward_us * 1.0e-3f);
    gcs().send_named_float("PPO_PGZ", _obs[5]);
    const uint16_t twist_off = _topo.valid ? uint16_t(6 + 3 * _topo.n_joints) : 48;
    gcs().send_named_float("PPO_VX", (twist_off < NNM_MAX_OBS) ? _obs[twist_off] : 0.0f);
    gcs().send_named_float("PPO_FAIL", float(_fail_reason));
    gcs().send_named_float("PPO_SLOT", float(_active_slot));
}

namespace AP {
    AP_NNMixer *nnmixer()
    {
        return AP_NNMixer::get_singleton();
    }
};

#endif // AP_NNMIXER_ENABLED
