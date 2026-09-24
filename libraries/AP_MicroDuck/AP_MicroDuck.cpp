#include "AP_MicroDuck_config.h"

#if AP_MICRODUCK_ENABLED

#include "AP_MicroDuck.h"
#include "microduck_infer.h"
#include "policy_mlp.h"
#if AP_MICRODUCK_CARTAN_ENABLED
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

#if AP_MICRODUCK_JOINT_FEEDBACK_SITL_ENABLED
#include <SITL/SITL.h>
#endif
#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
#include <time.h>
#endif

extern const AP_HAL::HAL& hal;

// servo wire encoding, shared with plant/mujoco_json_plant.py: 1 us = 3 mrad around 1500
#define MDK_PWM_CENTER 1500
#define MDK_RAD_PER_US 0.003f
#define MDK_PWM_MIN 800
#define MDK_PWM_MAX 2200

const AP_Param::GroupInfo AP_MicroDuck::var_info[] = {
    // @Param: ENABLE
    // @DisplayName: MicroDuck policy enable
    // @Description: Run the MicroDuck PPO locomotion policy and drive the 14 joint servos
    // @Values: 0:Disabled,1:Enabled
    // @User: Advanced
    AP_GROUPINFO_FLAGS("ENABLE", 1, AP_MicroDuck, _enable, 0, AP_PARAM_FLAG_ENABLE),

    // @Param: POLICY
    // @DisplayName: Policy network
    // @Description: 0 = MLP (512-256-128 ELU). 1 = Cartan (reserved)
    // @Values: 0:MLP,1:Cartan
    // @User: Advanced
    AP_GROUPINFO("POLICY", 2, AP_MicroDuck, _policy, 0),

    // @Param: VX_MAX
    // @DisplayName: Forward speed at full stick
    // @Units: m/s
    // @User: Advanced
    AP_GROUPINFO("VX_MAX", 3, AP_MicroDuck, _vx_max, 0.4f),

    // @Param: VY_MAX
    // @DisplayName: Lateral speed at full stick
    // @Units: m/s
    // @User: Advanced
    AP_GROUPINFO("VY_MAX", 4, AP_MicroDuck, _vy_max, 0.3f),

    // @Param: WZ_MAX
    // @DisplayName: Yaw rate at full stick
    // @Units: rad/s
    // @User: Advanced
    AP_GROUPINFO("WZ_MAX", 5, AP_MicroDuck, _wz_max, 1.0f),

    // @Param: WD_MS
    // @DisplayName: Joint feedback watchdog
    // @Description: If joint feedback is older than this the action is forced to the stand pose
    // @Units: ms
    // @User: Advanced
    AP_GROUPINFO("WD_MS", 6, AP_MicroDuck, _wd_ms, 40),

    // @Param: ATT_SRC
    // @DisplayName: Gravity direction source
    // @Description: 0 = internal IMU-only complementary filter (as trained), 1 = AHRS quaternion (debug)
    // @Values: 0:IMU filter,1:AHRS
    // @User: Advanced
    AP_GROUPINFO("ATT_SRC", 7, AP_MicroDuck, _att_src, 0),

    // @Param: ATT_TAU
    // @DisplayName: Gravity filter time constant
    // @Units: s
    // @User: Advanced
    AP_GROUPINFO("ATT_TAU", 8, AP_MicroDuck, _att_tau, 0.5f),

    // @Param: RC_VX
    // @DisplayName: RC channel for forward speed
    // @User: Advanced
    AP_GROUPINFO("RC_VX", 9, AP_MicroDuck, _rc_vx, 2),

    // @Param: RC_VY
    // @DisplayName: RC channel for lateral speed
    // @User: Advanced
    AP_GROUPINFO("RC_VY", 10, AP_MicroDuck, _rc_vy, 1),

    // @Param: RC_WZ
    // @DisplayName: RC channel for yaw rate
    // @User: Advanced
    AP_GROUPINFO("RC_WZ", 11, AP_MicroDuck, _rc_wz, 4),

    // @Param: ACT_MAX
    // @DisplayName: Action clip
    // @Units: rad
    // @User: Advanced
    AP_GROUPINFO("ACT_MAX", 12, AP_MicroDuck, _act_max, 2.0f),

    // @Param: LOG
    // @DisplayName: Log observations and actions every tick
    // @Values: 0:Off,1:On
    // @User: Advanced
    AP_GROUPINFO("LOG", 13, AP_MicroDuck, _log, 1),

    // @Param: HOLD_MODE
    // @DisplayName: Vehicle mode that forces zero twist (stand)
    // @Description: Rover HOLD is 4. Set -1 to disable
    // @User: Advanced
    AP_GROUPINFO("HOLD_MODE", 14, AP_MicroDuck, _hold_mode, 4),

    // @Param: SRV_FN0
    // @DisplayName: Servo function of joint 1
    // @Description: Joints 1..14 use this function and the 13 following ones (default Scripting1..14 = 94..107)
    // @User: Advanced
    AP_GROUPINFO("SRV_FN0", 15, AP_MicroDuck, _servo_fn0, 94),

    AP_GROUPEND
};

AP_MicroDuck *AP_MicroDuck::_singleton;

// wall-clock microseconds for profiling the forward pass. In SITL lock-step the
// HAL clock is frozen while the loop runs, so read the host clock there.
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

AP_MicroDuck::AP_MicroDuck()
{
    if (_singleton != nullptr) {
        AP_HAL::panic("AP_MicroDuck must be singleton");
    }
    _singleton = this;
    AP_Param::setup_object_defaults(this, var_info);
}

void AP_MicroDuck::init()
{
    memset(_last_action, 0, sizeof(_last_action));
    _down_body = Vector3f(0, 0, 1);
    _down_valid = false;
    _joint_count = 0;
    _joint_ms = 0;
    _tick = 0;
    _fail_reason = 5;
    _forward_us = 0;
    _last_telem_ms = 0;
    memset(_obs, 0, sizeof(_obs));
    memset(_act, 0, sizeof(_act));
    _initialised = true;
    GCS_SEND_TEXT(MAV_SEVERITY_INFO, "MicroDuck: policy %s, obs %u -> %u joints",
                  (_policy == 1) ? "Cartan" : "MLP", unsigned(MLP_OBS_DIM), unsigned(MDK_N_JOINTS));
}

// -----------------------------------------------------------------------------
// gravity direction, IMU only. Body FRD. Level and static: accel = (0,0,-g) so
// down = -accel/|accel|. Propagation of a world-fixed vector seen from the
// body: d(down)/dt = -gyro x down.
void AP_MicroDuck::update_attitude()
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
    // gyro propagation
    Vector3f d = _down_body - (gyro % _down_body) * dt;
    // accel correction (only when |a| is plausibly gravity)
    if (an > 0.5f * GRAVITY_MSS && an < 1.5f * GRAVITY_MSS && _att_tau > 0.0f) {
        const float alpha = constrain_float(dt / _att_tau, 0.0f, 1.0f);
        d = d * (1.0f - alpha) + (-accel / an) * alpha;
    }
    const float n = d.length();
    if (n > 1e-6f) {
        _down_body = d / n;
    }
}

// gravity direction in the trunk FLU frame (training convention): FRD -> FLU is (x, -y, -z)
void AP_MicroDuck::gravity_body_flu(float g[3])
{
    Vector3f down = _down_body;
    if (_att_src == 1) {
        Quaternion q;
        AP::ahrs().get_quat_body_to_ned(q);
        Matrix3f m;
        q.rotation_matrix(m);
        down = m.mul_transpose(Vector3f(0, 0, 1));
    }
    // FLU gravity = -up; down_frd -> flu: (x, -y, -z)
    g[0] = down.x;
    g[1] = -down.y;
    g[2] = -down.z;
}

// -----------------------------------------------------------------------------
void AP_MicroDuck::set_joint_feedback(const float *pos, const float *vel, uint8_t count)
{
    WITH_SEMAPHORE(_joint_sem);
    const uint8_t n = MIN(count, uint8_t(MDK_N_JOINTS));
    for (uint8_t i = 0; i < n; i++) {
        _joint_pos[i] = pos[i];
        _joint_vel[i] = vel[i];
    }
    _joint_count = n;
    _joint_ms = AP_HAL::millis();
}

bool AP_MicroDuck::read_joint_feedback()
{
#if AP_MICRODUCK_JOINT_FEEDBACK_SITL_ENABLED
    // SITL backend: the physics plant reports the joints through SIM_JSON into sitl->state
    const SITL::SIM *sitl = AP::sitl();
    if (sitl != nullptr && sitl->state.joint_count >= MDK_N_JOINTS &&
        sitl->state.joint_time_us != _last_joint_time_us) {
        // only a NEW sample counts as feedback, so the watchdog sees a frozen plant
        _last_joint_time_us = sitl->state.joint_time_us;
        set_joint_feedback(sitl->state.joint_pos, sitl->state.joint_vel, MDK_N_JOINTS);
    }
#endif
    WITH_SEMAPHORE(_joint_sem);
    if (_joint_count < MDK_N_JOINTS) {
        _fail_reason = 2;
        return false;
    }
    if (AP_HAL::millis() - _joint_ms > uint32_t(MAX(_wd_ms.get(), 1))) {
        _fail_reason = 3;
        return false;
    }
    return true;
}

// -----------------------------------------------------------------------------
void AP_MicroDuck::read_twist(float twist[3])
{
    twist[0] = twist[1] = twist[2] = 0.0f;
    const AP_Vehicle *veh = AP::vehicle();
    if (_hold_mode >= 0 && veh != nullptr && veh->get_mode() == uint8_t(_hold_mode)) {
        return;
    }
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
        // norm_input_dz: -1..1 with the channel dead zone; MAVProxy "rc N 1750" -> +0.5.
        // Stick direction conventions are handled with RCn_REVERSED, not here.
        const float n = c->norm_input_dz();
        *map[i].dst = constrain_float(n, -1.0f, 1.0f) * map[i].scale;
    }
}

// -----------------------------------------------------------------------------
void AP_MicroDuck::write_servos(const float *q_target)
{
    for (uint8_t i = 0; i < MDK_N_JOINTS; i++) {
        const int32_t pwm = MDK_PWM_CENTER + int32_t(roundf(q_target[i] / MDK_RAD_PER_US));
        const uint16_t p = uint16_t(constrain_int32(pwm, MDK_PWM_MIN, MDK_PWM_MAX));
        SRV_Channels::set_output_pwm(SRV_Channel::Function(_servo_fn0 + i), p);
    }
}

void AP_MicroDuck::write_idle_servos()
{
    // 0 us = "not driving" for the plant (keeps the duck pinned); a real bus would torque-off
    for (uint8_t i = 0; i < MDK_N_JOINTS; i++) {
        SRV_Channels::set_output_pwm(SRV_Channel::Function(_servo_fn0 + i), 0);
    }
}

// -----------------------------------------------------------------------------
void AP_MicroDuck::update()
{
    if (!enabled()) {
        return;
    }
    if (!_initialised) {
        init();
    }
    _tick++;

    if (!hal.util->get_soft_armed()) {
        // disarmed: stand pose target, zero history
        memset(_last_action, 0, sizeof(_last_action));
        _fail_reason = 1;
        write_idle_servos();
        send_telemetry();
        return;
    }

    float twist[3];
    read_twist(twist);

    const bool joints_ok = read_joint_feedback();
    const AP_InertialSensor &ins = AP::ins();
    const Vector3f gyro = ins.get_gyro();

    // ---- observation, trunk FLU frame, SI units, training order
    float *o = _obs;
    o[0] = gyro.x;  o[1] = -gyro.y;  o[2] = -gyro.z;
    gravity_body_flu(&o[3]);
    {
        WITH_SEMAPHORE(_joint_sem);
        for (uint8_t i = 0; i < MDK_N_JOINTS; i++) {
            o[6 + i]  = _joint_pos[i] - mlp_default_pose[i];
            o[20 + i] = _joint_vel[i];
        }
    }
    for (uint8_t i = 0; i < MDK_N_JOINTS; i++) {
        o[34 + i] = _last_action[i];
    }
    o[48] = twist[0]; o[49] = twist[1]; o[50] = twist[2];
    for (uint8_t i = 51; i < MDK_OBS_DIM; i++) {
        o[i] = 0.0f;
    }

    // ---- policy (both networks share the observation contract and default pose)
    static const microduck_policy_t mlp_policy = {
        MLP_OBS_DIM, MLP_ACT_DIM, MLP_N_LAYERS,
        mlp_dims, mlp_act, mlp_obs_mean, mlp_obs_std, mlp_W, mlp_b,
    };
#if AP_MICRODUCK_CARTAN_ENABLED
    static const microduck_cartan_t cartan_policy = {
        CARTAN_OBS_DIM, CARTAN_ACT_DIM, CARTAN_PAINT, CARTAN_N_LAYERS,
        cartan_obs_mean, cartan_obs_std, cartan_in_W, cartan_in_b,
        cartan_W, cartan_b, cartan_beta, cartan_theta, cartan_head_W, cartan_head_b, 0.1f,
    };
#endif
    bool ok = joints_ok && _down_valid;
    if (ok) {
        const uint32_t t0 = wall_micros();
#if AP_MICRODUCK_CARTAN_ENABLED
        if (_policy == 1) {
            ok = microduck_cartan_forward(&cartan_policy, _obs, _act) == 0;
        } else
#endif
        {
            ok = microduck_forward(&mlp_policy, _obs, _act) == 0;
        }
        _forward_us = wall_micros() - t0;
        if (!ok) {
            _fail_reason = 4;
        }
    }
    if (ok) {
        _fail_reason = 0;
        for (uint8_t i = 0; i < MDK_N_JOINTS; i++) {
            _act[i] = constrain_float(_act[i], -_act_max, _act_max);
        }
    } else {
        // failsafe: stand pose, and the history must see what was actually applied
        memset(_act, 0, sizeof(_act));
    }

    float q_target[MDK_N_JOINTS];
    for (uint8_t i = 0; i < MDK_N_JOINTS; i++) {
        q_target[i] = mlp_default_pose[i] + _act[i];
        _last_action[i] = _act[i];
    }
    write_servos(q_target);

    log_tick(twist);
    send_telemetry();
}

// -----------------------------------------------------------------------------
void AP_MicroDuck::log_tick(const float twist[3])
{
#if HAL_LOGGING_ENABLED
    if (_log == 0) {
        return;
    }
    const uint64_t now = AP_HAL::micros64();
    const float *o = _obs;
// @LoggerMessage: MDK
// @Description: MicroDuck policy tick summary
// @Field: TimeUS: Time since system startup
// @Field: Tick: policy tick counter
// @Field: Fail: 0 ok,1 disarmed,2 no joints,3 stale joints,4 forward error
// @Field: FwdUS: network forward time
// @Field: GX: trunk gyro x (FLU)
// @Field: GY: trunk gyro y (FLU)
// @Field: GZ: trunk gyro z (FLU)
// @Field: PGX: projected gravity x
// @Field: PGY: projected gravity y
// @Field: PGZ: projected gravity z
// @Field: VX: twist vx command
// @Field: VY: twist vy command
// @Field: WZ: twist wz command
    AP::logger().WriteStreaming("MDK", "TimeUS,Tick,Fail,FwdUS,GX,GY,GZ,PGX,PGY,PGZ,VX,VY,WZ",
                                "QIBIfffffffff",
                                now, _tick, _fail_reason, _forward_us,
                                o[0], o[1], o[2], o[3], o[4], o[5], twist[0], twist[1], twist[2]);
// @LoggerMessage: MDKQ
// @Description: MicroDuck joint positions relative to default pose (obs[6:20])
    AP::logger().WriteStreaming("MDKQ", "TimeUS,Q0,Q1,Q2,Q3,Q4,Q5,Q6,Q7,Q8,Q9,Q10,Q11,Q12,Q13",
                                "Qffffffffffffff",
                                now, o[6], o[7], o[8], o[9], o[10], o[11], o[12], o[13], o[14], o[15], o[16], o[17], o[18], o[19]);
// @LoggerMessage: MDKV
// @Description: MicroDuck joint velocities (obs[20:34])
    AP::logger().WriteStreaming("MDKV", "TimeUS,V0,V1,V2,V3,V4,V5,V6,V7,V8,V9,V10,V11,V12,V13",
                                "Qffffffffffffff",
                                now, o[20], o[21], o[22], o[23], o[24], o[25], o[26], o[27], o[28], o[29], o[30], o[31], o[32], o[33]);
// @LoggerMessage: MDKA
// @Description: MicroDuck actions (rad offsets from default pose)
    AP::logger().WriteStreaming("MDKA", "TimeUS,A0,A1,A2,A3,A4,A5,A6,A7,A8,A9,A10,A11,A12,A13",
                                "Qffffffffffffff",
                                now, _act[0], _act[1], _act[2], _act[3], _act[4], _act[5], _act[6], _act[7], _act[8], _act[9], _act[10], _act[11], _act[12], _act[13]);
#endif
}

void AP_MicroDuck::send_telemetry()
{
    const uint32_t now = AP_HAL::millis();
    if (now - _last_telem_ms < 500) {
        return;
    }
    _last_telem_ms = now;
    gcs().send_named_float("PPO_MS", _forward_us * 1.0e-3f);
    gcs().send_named_float("PPO_PGZ", _obs[5]);
    gcs().send_named_float("PPO_VX", _obs[48]);
    gcs().send_named_float("PPO_FAIL", float(_fail_reason));
}

namespace AP {
    AP_MicroDuck *microduck()
    {
        return AP_MicroDuck::get_singleton();
    }
};

#endif // AP_MICRODUCK_ENABLED
