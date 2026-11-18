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
#include <AP_JointBus/AP_JointBus.h>

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

    // Pose teleop from GCS (DEBUG_FLOAT_ARRAY name NNM_POSE): up to 8 channels after the clock.
    void set_pose_cmd(const float *values, uint8_t n);

    // Called by Rover for GUIDED/AUTO/RTL/SMART_RTL desired body twist (SI).
    void set_nav_twist(float vx, float vy, float wz);

    // Setup request from the GCS (DEBUG_FLOAT_ARRAY name NNM_SRV):
    // data[0] sequence, data[1] operation, data[2] actuator, data[3..4] arguments.
    // 1 releases torque and 2 restores the disarmed pose; the others are AP_JointBus::Op.
    // The answer goes out as NNM_SRVR.
    void handle_service(const float *data);

    // Auxiliary actuator (the duck's mouth) from the GCS (DEBUG_FLOAT_ARRAY name NNM_AUX), degrees.
    void set_aux_cmd(float deg);

    // Timed skill from the robot's skills.bin (MAV_CMD_USER_1, param1 = skill id).
    bool start_skill(uint8_t id);

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
    AP_Float _clock_hz;      // gesture clock into the extra observation channels (0 = off)
    AP_Int16 _pose_wd_ms;    // pose-command watchdog (ms); stale -> ease to rest
    AP_Float _pose_tau;      // pose-command low-pass time constant (s)
    AP_Int8  _clock_auto;    // 1: gait clock runs only while a twist is commanded
    AP_Int8  _getup_idx;     // policy index for auto get-up (-1 = disabled)
    AP_Float _getup_tilt;    // enter get-up when tilt exceeds this (deg)
    AP_Float _getup_ok;      // leave get-up when tilt is below this (deg)
    AP_Float _getup_s;       // oneshot clip duration (s)
    AP_Float _getup_hold;    // upright hold before returning to previous policy (s)
    AP_Int8  _out;           // 0: servo PWM only, 1: also the joint bus
    AP_Int8  _dis_pose;      // 1: hold the default pose while disarmed
    AP_Int8  _deploy;        // 0: standard chain, 1: XDuck chain
    AP_Float _xd_accx;       // XDuck chain: accelerometer-x trim while a twist is commanded (m/s/s)
    AP_Int8  _gains;         // 1: publish the robot's per-servo KP/KD table to the bus while disarmed

    bool _initialised;
    float _clock_phase;
    bool _clock_oneshot;     // get-up: phase advances 0→π once, then holds
    float _pose_cmd[8];
    float _pose_filt[8];
    uint8_t _pose_n;
    uint32_t _pose_ms;
    HAL_Semaphore _pose_sem;
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
    // timing of the policy task (update) and of the attitude task, reported in the telemetry
    uint32_t _rate_last_us;
    uint32_t _rate_gap_us;
    uint16_t _rate_count;
    float _rate_hz;
    uint32_t _loop_last_us;
    uint32_t _loop_gap_us;
    uint16_t _loop_count;
    uint32_t _tick;
    uint8_t _fail_reason;
    int8_t _loaded_policy_idx;

    // automatic get-up recovery
    enum class GetupState : uint8_t { IDLE, FALLING, ACTIVE, RECOVERED };
    GetupState _getup_state;
    int8_t _getup_prev_idx;       // policy to restore after get-up
    uint32_t _getup_tilt_ms;      // first time tilt exceeded threshold
    uint32_t _getup_ok_ms;        // first time upright after get-up
    uint32_t _getup_start_ms;     // when get-up policy became active
    bool _getup_preload_pending;  // reload getup into the free slot after a manual switch

    // SD -> RAM load of the free slot, done on the IO thread
    enum class LoadState : uint8_t { IDLE, REQUESTED, LOADING, DONE_OK, DONE_FAIL };
    HAL_Semaphore _load_sem;
    LoadState _load_state;
    int8_t _load_idx;
    uint8_t _load_slot;
    int8_t _rejected_idx;    // file refused for this NNM_POLICY value; not retried until it changes
    int8_t _last_want;
    bool _io_registered;
    bool _force_switch;      // get-up may switch while armed in MANUAL

    // per-robot deployment table (skills.bin), optional
    NNM_Deploy _deploy_cfg;

    // output stage: PWM always, the joint bus with NNM_OUT 1
#if AP_JOINTBUS_ENABLED
    AP_JointBus _bus;
#endif
    bool _bus_ready;
    uint32_t _bus_fb_ms;          // timestamp of the last bus sample handed to the observation
    bool _torque_off;             // released on request while disarmed; cleared by arming
    bool _out_active;             // outputs were driven on the previous tick
    bool _out_armed;              // armed state of the last drive_outputs() call
    float _out_q[NNM_MAX_JOINTS]; // last commanded pose
    float _ramp_from[NNM_MAX_JOINTS];
    uint32_t _ramp_start_ms;
    uint16_t _ramp_ms;            // 0 = no ramp running
    int8_t _hold_idx;             // policy whose default pose is being held
    float _aux_cmd;               // degrees
    uint32_t _last_jnt_ms;
    uint32_t _last_vlt_ms;
    bool _gains_done;             // the MCU confirmed the run-gain table since the link came up
    uint32_t _gains_ms;           // last table sent (retry interval)
    uint16_t _gains_seq;

    uint8_t _chain;               // NNM_DEPLOY as latched while disarmed

    // XDuck chain (NNM_DEPLOY 1)
    float _act_filt[NNM_MAX_JOINTS];
    Vector3f _xd_grav;            // unit gravity, robot frame (x forward, y left, z up)
    Vector3f _xd_gyro;            // smoothed rate, same frame
    bool _xd_valid;
    uint32_t _xd_src_us;          // sender time of the last link IMU sample
    uint32_t _xd_src_ms;
    bool _xd_src_valid;
    float _xd_accx_cmd;           // trim applied by the attitude task
    uint32_t _xd_prehold_ms;      // start of the default-pose hold before get-up
    bool _hh_walking;             // heading hold may run
    bool _hh_valid;
    bool _hh_has_target;
    float _hh_yaw;
    float _hh_target;

    // skills
    enum class SkillState : uint8_t { IDLE, STARTING, RUNNING, ENDING };
    SkillState _skill_state;
    uint8_t _skill_idx;           // index into _deploy_cfg.skill while not IDLE
    uint32_t _skill_start_ms;

    NNM_RobotTopology _topo;
    NNM_PolicySlot _slot[2];
    uint8_t _active_slot;   // 0 or 1
    bool _use_sd_policy;    // true when int8 slot is active
    bool _use_baked_mlp;    // float32 fallback (MicroDuck)

    void init();
    bool load_robot_topology();
    bool ensure_slots_allocated();
    bool load_policy_index(int8_t index, uint8_t into_slot);
    void service_policy_switch();
    void service_getup();
    void request_policy_load(int8_t idx, bool force);
    float trunk_tilt_deg() const;
    void io_update();
    bool read_joint_feedback();
    void read_twist(float twist[3]);
    void gravity_body_flu(float g[3]);
    void write_servos(const float *q_target);
    void write_idle_servos();
    void log_tick(const float twist[3]);
    void send_telemetry();
    const float *default_pose() const;
    void load_deploy_table();
    bool bus_ready() const { return _bus_ready; }
    const NNM_PolicyCfg *policy_cfg(int8_t idx) const;
    void update_attitude_xduck(const Vector3f &gyro, const Vector3f &accel, float dt);
    float heading_hold(bool walking, const float twist[3], float kp);
    void service_skill();
    void end_skill();
    void reset_history();
    void drive_outputs(const float *q_des, bool armed);
    void outputs_off();
    void service_bus();
    void publish_run_gains();
    void send_service_reply(uint16_t seq, uint8_t op, bool ok, uint8_t id);
};

namespace AP {
    AP_NNMixer *nnmixer();
}

#endif // AP_NNMIXER_ENABLED
