/*
   AP_MicroDuck: run the MicroDuck PPO locomotion policy (61 obs -> 14 joint
   position offsets, 50 Hz) as an ArduPilot task.

   The policy contract is fixed by training (mjlab / rsl_rl, pollen-robotics
   microduck_rl):
     obs[0:3]   trunk angular velocity, rad/s, trunk FLU frame
     obs[3:6]   gravity direction in trunk frame (unit vector, stand ~ [0,0,-1])
     obs[6:20]  joint position - default pose, rad
     obs[20:34] joint velocity, rad/s
     obs[34:48] previous action (as applied), rad
     obs[48:51] twist command vx, vy (m/s), wz (rad/s)
     obs[51:55] head pose command (rad)      -> 0 here
     obs[55:61] body pose command (m, rad)   -> 0 here
     act[14]    q_target = default_pose + act, rad

   Everything ArduPilot-specific is an adapter: IMU frame (FRD -> FLU), an
   IMU-only gravity estimate (no EKF in the observation), joint feedback,
   sticks -> twist in SI units, and the action history. The network itself is
   pure C (microduck_infer.c) with the training normalizer baked in, so the same
   code runs in SITL and on a microcontroller.
*/
#pragma once

#include "AP_MicroDuck_config.h"

#if AP_MICRODUCK_ENABLED

#include <AP_Param/AP_Param.h>
#include <AP_Math/AP_Math.h>

#define MDK_N_JOINTS 14
#define MDK_OBS_DIM 61

class AP_MicroDuck {
public:
    AP_MicroDuck();

    CLASS_NO_COPY(AP_MicroDuck);

    static AP_MicroDuck *get_singleton() { return _singleton; }

    // 50 Hz: build observation, run the policy, write the 14 servo outputs
    void update();

    // 400 Hz: IMU-only gravity direction filter (gyro propagation + accel correction)
    void update_attitude();

    // joint feedback from the actuator bus (or the SITL plant), rad / rad/s,
    // in the training joint order
    void set_joint_feedback(const float *pos, const float *vel, uint8_t count);

    bool enabled() const { return _enable != 0; }

    static const struct AP_Param::GroupInfo var_info[];

private:
    static AP_MicroDuck *_singleton;

    // parameters
    AP_Int8  _enable;
    AP_Int8  _policy;        // 0 = MLP (Cartan reserved for 1)
    AP_Float _vx_max;        // m/s at full stick
    AP_Float _vy_max;        // m/s at full stick
    AP_Float _wz_max;        // rad/s at full stick
    AP_Int16 _wd_ms;         // joint feedback watchdog
    AP_Int8  _att_src;       // 0 = internal gravity filter, 1 = AHRS quaternion
    AP_Float _att_tau;       // s, accel correction time constant
    AP_Int8  _rc_vx;         // RC channel (1-based) for vx
    AP_Int8  _rc_vy;
    AP_Int8  _rc_wz;
    AP_Float _act_max;       // clip |action| (rad)
    AP_Int8  _log;           // 1 = log obs/actions every tick
    AP_Int8  _hold_mode;     // vehicle mode number that forces twist = 0 (Rover HOLD = 4)
    AP_Int8  _servo_fn0;     // SRV function number of joint 1 (default k_scripting1 = 94)

    // state
    bool _initialised;
    Vector3f _down_body;     // gravity (down) direction estimate, body FRD, unit
    bool _down_valid;
    float _last_action[MDK_N_JOINTS];
    float _joint_pos[MDK_N_JOINTS];
    float _joint_vel[MDK_N_JOINTS];
    uint32_t _joint_ms;
    uint64_t _last_joint_time_us;
    uint8_t _joint_count;
    HAL_Semaphore _joint_sem;
    float _obs[MDK_OBS_DIM];
    float _act[MDK_N_JOINTS];
    uint32_t _forward_us;
    uint32_t _last_telem_ms;
    uint32_t _tick;
    uint8_t _fail_reason;    // 0 ok, 1 disarmed, 2 no joints, 3 stale joints, 4 forward err, 5 disabled

    void init();
    bool read_joint_feedback();
    void read_twist(float twist[3]);
    void gravity_body_flu(float g[3]);
    void write_servos(const float *q_target);
    void write_idle_servos();
    void log_tick(const float twist[3]);
    void send_telemetry();
};

namespace AP {
    AP_MicroDuck *microduck();
};

#endif // AP_MICRODUCK_ENABLED
