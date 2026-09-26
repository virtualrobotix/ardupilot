/*
  AP_NNMixer: neural-network mixer that runs a PPO locomotion policy
  as an ArduPilot task. Robot topology is fixed at boot; policies for
  that robot are loaded from SD into dual int8 RAM slots.

  Author: Roberto Navoni, member of the ArduPilot Dev Team
  Contact: r.navoni74@gmail.com
  Developed by Roberto Navoni — DelphyAI LAB
  For information: r.navoni74@gmail.com
*/
#pragma once

#include "AP_NNMixer_config.h"

#if AP_NNMIXER_ENABLED

#include "AP_NNMixer_Policy.h"
#include <AP_Param/AP_Param.h>
#include <AP_Math/AP_Math.h>

class AP_NNMixer {
public:
    AP_NNMixer();

    CLASS_NO_COPY(AP_NNMixer);

    static AP_NNMixer *get_singleton() { return _singleton; }

    // policy rate (typically 50 Hz): observation, forward, servos
    void update();

    // IMU-only gravity filter at loop rate
    void update_attitude();

    void set_joint_feedback(const float *pos, const float *vel, uint8_t count);

    void set_hil_state(const float *pos, const float *vel,
                       const float *gyro_flu, const float *gravity_flu);

    // Called by Rover for GUIDED/AUTO/RTL/SMART_RTL desired body twist (SI).
    void set_nav_twist(float vx, float vy, float wz);

    bool enabled() const { return _enable != 0; }

    uint16_t n_joints() const { return _topo.valid ? _topo.n_joints : 0; }
    uint16_t obs_dim() const { return _topo.valid ? _topo.obs_dim : 0; }

    static const struct AP_Param::GroupInfo var_info[];

private:
    static AP_NNMixer *_singleton;

    AP_Int8  _enable;
    AP_Int8  _robot;         // NNM_RobotId, reboot to apply
    AP_Int8  _policy;        // index into /APM/nnm/<id>/policies/*.nnm (0 = default)
    AP_Float _vx_max;
    AP_Float _vy_max;
    AP_Float _wz_max;
    AP_Int16 _wd_ms;
    AP_Int8  _att_src;
    AP_Float _att_tau;
    AP_Int8  _rc_vx;
    AP_Int8  _rc_vy;
    AP_Int8  _rc_wz;
    AP_Float _act_max;
    AP_Int8  _log;
    AP_Int8  _hold_mode;
    AP_Int8  _servo_fn0;
    AP_Int8  _hil_att;
    AP_Int16 _blend_ms;      // cross-fade duration on policy switch

    bool _initialised;
    Vector3f _down_body;
    bool _down_valid;
    float _last_action[NNM_MAX_JOINTS];
    float _joint_pos[NNM_MAX_JOINTS];
    float _joint_vel[NNM_MAX_JOINTS];
    uint32_t _joint_ms;
    uint64_t _last_joint_time_us;
    uint8_t _joint_count;
    HAL_Semaphore _joint_sem;
    float _hil_gyro_flu[3];
    float _hil_gravity_flu[3];
    uint32_t _hil_state_ms;
    float _obs[NNM_MAX_OBS];
    float _act[NNM_MAX_JOINTS];
    float _act_blend_from[NNM_MAX_JOINTS];
    uint32_t _blend_start_ms;
    bool _blending;
    float _nav_twist[3];
    uint32_t _nav_twist_ms;
    uint32_t _forward_us;
    uint32_t _last_telem_ms;
    uint32_t _tick;
    uint8_t _fail_reason;
    int8_t _loaded_policy_idx;
    int8_t _pending_policy_idx;

    NNM_RobotTopology _topo;
    NNM_PolicySlot _slot[2];
    uint8_t _active_slot;   // 0 or 1
    bool _use_sd_policy;    // true when int8 slot is active
    bool _use_baked_mlp;    // float32 fallback (MicroDuck)

    void init();
    bool load_robot_topology();
    bool ensure_slots_allocated();
    bool load_policy_index(int8_t index, uint8_t into_slot);
    void request_policy_switch(int8_t index);
    void service_policy_switch();
    bool read_joint_feedback();
    void read_twist(float twist[3]);
    void gravity_body_flu(float g[3]);
    void write_servos(const float *q_target);
    void write_idle_servos();
    void log_tick(const float twist[3]);
    void send_telemetry();
    const float *default_pose() const;
};

namespace AP {
    AP_NNMixer *nnmixer();
}

#endif // AP_NNMIXER_ENABLED
